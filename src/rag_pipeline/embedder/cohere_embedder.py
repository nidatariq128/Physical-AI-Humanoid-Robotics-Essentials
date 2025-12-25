"""Cohere API client utilities for generating embeddings."""

from typing import List, Optional
import cohere
from src.rag_pipeline.base_models import EmbeddingVector
from src.rag_pipeline.config import config
from src.rag_pipeline.logging import logger
from src.rag_pipeline.exceptions import EmbeddingError


class CohereEmbedder:
    """Class to handle embedding generation using Cohere API."""

    def __init__(self):
        """Initialize Cohere client with API key."""
        try:
            self.client = cohere.Client(config.cohere_api_key)
            # Use a default model, can be configurable later
            self.model = "embed-multilingual-v3.0"  # Good for technical documentation
        except Exception as e:
            logger.error(f"Failed to initialize Cohere client: {str(e)}")
            raise EmbeddingError(f"Failed to initialize Cohere client: {str(e)}")

    def generate_embedding(self, text: str) -> EmbeddingVector:
        """
        Generate embedding for a single text.

        Args:
            text: Text to generate embedding for

        Returns:
            EmbeddingVector object with the generated embedding
        """
        try:
            response = self.client.embed(
                texts=[text],
                model=self.model,
                input_type="search_document"  # Optimize for document search
            )

            if not response.embeddings or len(response.embeddings) == 0:
                raise EmbeddingError("Cohere API returned empty embeddings")

            embedding_vector = response.embeddings[0]
            return EmbeddingVector(
                vector=embedding_vector,
                model_name=self.model
            )

        except Exception as e:
            logger.error(f"Failed to generate embedding for text: {str(e)}")
            raise EmbeddingError(f"Failed to generate embedding: {str(e)}")

    def generate_embeddings_batch(self, texts: List[str], batch_size: int = 96) -> List[EmbeddingVector]:
        """
        Generate embeddings for a batch of texts.

        Args:
            texts: List of texts to generate embeddings for
            batch_size: Size of batches to send to Cohere API (max 96 for free tier)

        Returns:
            List of EmbeddingVector objects
        """
        all_embeddings = []

        # Process in batches to respect API limits
        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            try:
                response = self.client.embed(
                    texts=batch,
                    model=self.model,
                    input_type="search_document"
                )

                if not response.embeddings:
                    raise EmbeddingError(f"Cohere API returned empty embeddings for batch {i//batch_size + 1}")

                # Create EmbeddingVector objects for each result
                for j, embedding_vector in enumerate(response.embeddings):
                    embedding_obj = EmbeddingVector(
                        vector=embedding_vector,
                        model_name=self.model
                    )
                    all_embeddings.append(embedding_obj)

                logger.info(f"Processed batch {i//batch_size + 1}/{(len(texts)-1)//batch_size + 1}")

            except Exception as e:
                logger.error(f"Failed to generate embeddings for batch {i//batch_size + 1}: {str(e)}")
                raise EmbeddingError(f"Failed to generate embeddings for batch: {str(e)}")

        return all_embeddings

    def get_model_info(self) -> dict:
        """
        Get information about the embedding model being used.

        Returns:
            Dictionary with model information
        """
        return {
            "model_name": self.model,
            "dimensions": 1024,  # Cohere multilingual v3 has 1024 dimensions
            "description": "Cohere multilingual embedding model v3"
        }