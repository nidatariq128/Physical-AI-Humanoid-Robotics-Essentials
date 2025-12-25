"""Local storage implementation for the RAG Knowledge Ingestion Pipeline."""

import json
import os
from typing import List, Dict, Any, Optional
from pathlib import Path
from src.rag_pipeline.base_models import DocumentChunk
from ..config import config
from ..logging import logger
from ..exceptions import StorageError


class LocalStorage:
    """Class to handle local storage operations when external services aren't available."""

    def __init__(self, storage_dir: str = "output"):
        """Initialize local storage with a directory."""
        self.storage_dir = Path(storage_dir)
        self.storage_dir.mkdir(exist_ok=True)
        self.chunks_file = self.storage_dir / "document_chunks.json"

        # Initialize chunks list
        if self.chunks_file.exists():
            with open(self.chunks_file, 'r', encoding='utf-8') as f:
                self.chunks = json.load(f)
        else:
            self.chunks = []

    def store_chunks(self, chunks: List[DocumentChunk]) -> bool:
        """
        Store document chunks in local JSON file.

        Args:
            chunks: List of DocumentChunk objects to store

        Returns:
            True if storage was successful, False otherwise
        """
        try:
            # Convert DocumentChunk objects to dictionaries for JSON serialization
            chunk_dicts = []
            for chunk in chunks:
                chunk_dict = {
                    "chunk_id": chunk.chunk_id,
                    "text": chunk.text,
                    "source_reference": {
                        "url": chunk.source_reference.url,
                        "section_heading": chunk.source_reference.section_heading,
                        "chunk_index": chunk.source_reference.chunk_index,
                        "page_title": chunk.source_reference.page_title,
                        "parent_section": chunk.source_reference.parent_section
                    },
                    "metadata": chunk.metadata
                }

                # Add embedding vector if it exists
                if chunk.embedding:
                    chunk_dict["embedding"] = {
                        "vector_length": len(chunk.embedding.vector),
                        "model_name": chunk.embedding.model_name
                    }

                chunk_dicts.append(chunk_dict)

            # Append new chunks to existing ones
            self.chunks.extend(chunk_dicts)

            # Write all chunks to file
            with open(self.chunks_file, 'w', encoding='utf-8') as f:
                json.dump(self.chunks, f, ensure_ascii=False, indent=2)

            logger.info(f"Successfully stored {len(chunks)} chunks locally in {self.chunks_file}")
            logger.info(f"Total chunks stored locally: {len(self.chunks)}")
            return True

        except Exception as e:
            logger.error(f"Failed to store chunks locally: {str(e)}")
            raise StorageError(f"Failed to store chunks locally: {str(e)}")

    def get_all_chunks(self) -> List[Dict[str, Any]]:
        """Retrieve all stored chunks."""
        return self.chunks

    def search_similar(self, query_embedding: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Mock search function for local mode (returns recent chunks since we can't do real similarity search).

        Args:
            query_embedding: Embedding vector to search for similar chunks (ignored in local mode)
            top_k: Number of similar chunks to return

        Returns:
            List of chunks with their metadata
        """
        logger.warning("Running in local mode - using mock search function")

        # Return the most recent chunks as mock "similar" results
        recent_chunks = self.chunks[-top_k:] if len(self.chunks) >= top_k else self.chunks

        # Format the results to match the expected structure
        formatted_results = []
        for chunk in recent_chunks:
            formatted_result = {
                "id": chunk.get("chunk_id", "unknown"),
                "text": chunk.get("text", "")[:200] + "..." if len(chunk.get("text", "")) > 200 else chunk.get("text", ""),
                "url": chunk.get("source_reference", {}).get("url", ""),
                "section_heading": chunk.get("source_reference", {}).get("section_heading", ""),
                "score": 0.0,  # Mock score
                "metadata": chunk.get("metadata", {})
            }
            formatted_results.append(formatted_result)

        return formatted_results

    def get_chunk_by_id(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific chunk by its ID.

        Args:
            chunk_id: ID of the chunk to retrieve

        Returns:
            Chunk data with metadata or None if not found
        """
        for chunk in self.chunks:
            if chunk.get("chunk_id") == chunk_id:
                return chunk
        return None

    def clear_storage(self) -> bool:
        """Clear all stored chunks."""
        try:
            self.chunks = []
            if self.chunks_file.exists():
                self.chunks_file.unlink()
            logger.info("Local storage cleared")
            return True
        except Exception as e:
            logger.error(f"Failed to clear local storage: {str(e)}")
            return False