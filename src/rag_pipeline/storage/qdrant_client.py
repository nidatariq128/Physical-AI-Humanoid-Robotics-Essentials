"""Qdrant Cloud connection utilities for the RAG Knowledge Ingestion Pipeline."""

from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from src.rag_pipeline.base_models import DocumentChunk, EmbeddingVector
from src.rag_pipeline.config import config
from src.rag_pipeline.logging import logger
from src.rag_pipeline.exceptions import StorageError


class QdrantStorage:
    """Class to handle storage operations in Qdrant Cloud."""

    def __init__(self):
        """Initialize Qdrant client with configuration."""
        try:
            # Initialize Qdrant client with cloud configuration
            self.client = QdrantClient(
                url=config.qdrant_url,
                api_key=config.qdrant_api_key,
                prefer_grpc=True  # Use gRPC for better performance if available
            )
            self.collection_name = config.qdrant_collection_name or "document_chunks"
            self._ensure_collection_exists()
        except Exception as e:
            logger.error(f"Failed to initialize Qdrant client: {str(e)}")
            raise StorageError(f"Failed to connect to Qdrant: {str(e)}")

    def _ensure_collection_exists(self):
        """Ensure the collection exists with proper configuration."""
        try:
            # Check if collection exists
            collection_info = None
            try:
                collection_info = self.client.get_collection(self.collection_name)
            except:
                # Collection doesn't exist, we'll create it
                pass

            if collection_info is None:
                # Create collection with appropriate vector configuration
                # Using 1024-dimensional vectors for Cohere multilingual v3 model
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=1024,  # Size for Cohere embeddings
                        distance=models.Distance.COSINE
                    )
                )
                logger.info(f"Created Qdrant collection: {self.collection_name} with 1024-dimensional vectors")
            else:
                # Check if the vector size matches what we expect
                # If not, we need to recreate the collection
                # Access the vector configuration from the collection info
                vectors_config = collection_info.config.params.vectors
                if isinstance(vectors_config, models.VectorParams) and vectors_config.size != 1024:
                    logger.warning(f"Collection {self.collection_name} has {vectors_config.size}-dimensional vectors, "
                                 f"but we need 1024-dimensional vectors. Recreating collection...")

                    # Delete the existing collection
                    self.client.delete_collection(self.collection_name)

                    # Create new collection with correct dimensions
                    self.client.create_collection(
                        collection_name=self.collection_name,
                        vectors_config=models.VectorParams(
                            size=1024,  # Size for Cohere embeddings
                            distance=models.Distance.COSINE
                        )
                    )
                    logger.info(f"Recreated Qdrant collection: {self.collection_name} with 1024-dimensional vectors")
                else:
                    logger.info(f"Qdrant collection exists: {self.collection_name}")
        except Exception as e:
            logger.error(f"Failed to ensure collection exists: {str(e)}")
            raise StorageError(f"Failed to set up Qdrant collection: {str(e)}")

    def store_chunks(self, chunks: List[DocumentChunk]) -> bool:
        """
        Store document chunks in Qdrant Cloud.

        Args:
            chunks: List of DocumentChunk objects to store

        Returns:
            True if storage was successful, False otherwise
        """
        try:
            # Prepare points for insertion
            points = []
            for chunk in chunks:
                if chunk.embedding is None:
                    logger.warning(f"Chunk {chunk.chunk_id} has no embedding, skipping storage")
                    continue

                # Create payload with chunk data and metadata
                payload = {
                    "text": chunk.text,
                    "url": chunk.source_reference.url,
                    "section_heading": chunk.source_reference.section_heading,
                    "chunk_index": chunk.source_reference.chunk_index,
                    "page_title": chunk.source_reference.page_title,
                    "parent_section": chunk.source_reference.parent_section,
                    "chunk_id": chunk.chunk_id
                }

                # Add any additional metadata
                if chunk.metadata:
                    payload.update(chunk.metadata)

                # Create point for Qdrant
                point = models.PointStruct(
                    id=chunk.chunk_id,
                    vector=chunk.embedding.vector,
                    payload=payload
                )
                points.append(point)

            if not points:
                logger.warning("No points to store - all chunks were missing embeddings")
                return True  # Not an error, just no data to store

            # Upload points to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logger.info(f"Successfully stored {len(points)} chunks in Qdrant")
            return True

        except Exception as e:
            logger.error(f"Failed to store chunks in Qdrant: {str(e)}")
            raise StorageError(f"Failed to store chunks: {str(e)}")

    def search_similar(self, query_embedding: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar chunks using vector similarity.

        Args:
            query_embedding: Embedding vector to search for similar chunks
            top_k: Number of similar chunks to return

        Returns:
            List of similar chunks with their metadata
        """
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k
            )

            # Format results
            formatted_results = []
            for result in results:
                formatted_result = {
                    "id": result.id,
                    "text": result.payload.get("text", ""),
                    "url": result.payload.get("url", ""),
                    "section_heading": result.payload.get("section_heading", ""),
                    "score": result.score,
                    "metadata": {k: v for k, v in result.payload.items()
                               if k not in ["text", "url", "section_heading"]}
                }
                formatted_results.append(formatted_result)

            return formatted_results

        except Exception as e:
            logger.error(f"Failed to search similar chunks in Qdrant: {str(e)}")
            raise StorageError(f"Failed to search similar chunks: {str(e)}")

    def get_chunk_by_id(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific chunk by its ID.

        Args:
            chunk_id: ID of the chunk to retrieve

        Returns:
            Chunk data with metadata or None if not found
        """
        try:
            results = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[chunk_id]
            )

            if results:
                result = results[0]
                return {
                    "id": result.id,
                    "text": result.payload.get("text", ""),
                    "url": result.payload.get("url", ""),
                    "section_heading": result.payload.get("section_heading", ""),
                    "metadata": {k: v for k, v in result.payload.items()
                               if k not in ["text", "url", "section_heading"]}
                }

            return None

        except Exception as e:
            logger.error(f"Failed to retrieve chunk {chunk_id} from Qdrant: {str(e)}")
            raise StorageError(f"Failed to retrieve chunk {chunk_id}: {str(e)}")