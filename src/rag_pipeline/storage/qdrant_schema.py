"""Module defining Qdrant collection schema and metadata structure."""

from typing import Dict, Any, Optional
from qdrant_client.http import models
from ..logging import logger


class QdrantSchema:
    """Class to define and manage Qdrant collection schema."""

    def __init__(self):
        """Initialize the schema definition."""
        self.default_vector_size = 1024  # Default for Cohere multilingual v3
        self.collection_name = "document_chunks"

    def get_collection_config(self, vector_size: Optional[int] = None) -> Dict[str, Any]:
        """
        Get the configuration for the Qdrant collection.

        Args:
            vector_size: Size of the embedding vectors (default to 1024 for Cohere)

        Returns:
            Dictionary with collection configuration
        """
        vector_size = vector_size or self.default_vector_size

        config = {
            "vectors_config": models.VectorParams(
                size=vector_size,
                distance=models.Distance.COSINE
            ),
            "optimizers_config": models.OptimizersConfigDiff(
                memmap_threshold=20000,
                indexing_threshold=20000,
            ),
            "on_disk_payload": True  # Store payload on disk to save RAM
        }

        return config

    def get_payload_schema(self) -> Dict[str, models.PayloadSchemaType]:
        """
        Define the schema for payload fields (metadata).

        Returns:
            Dictionary mapping field names to their schema types
        """
        schema = {
            "text": models.PayloadSchemaType.TEXT,
            "url": models.PayloadSchemaType.KEYWORD,
            "section_heading": models.PayloadSchemaType.TEXT,
            "chunk_index": models.PayloadSchemaType.INTEGER,
            "page_title": models.PayloadSchemaType.TEXT,
            "parent_section": models.PayloadSchemaType.TEXT,
            "chunk_id": models.PayloadSchemaType.KEYWORD,
        }

        return schema

    def get_default_payload(self) -> Dict[str, Any]:
        """
        Get default payload structure for document chunks.

        Returns:
            Dictionary with default payload values
        """
        return {
            "text": "",
            "url": "",
            "section_heading": "",
            "chunk_index": 0,
            "page_title": "",
            "parent_section": "",
            "chunk_id": ""
        }

    def validate_payload(self, payload: Dict[str, Any]) -> bool:
        """
        Validate payload structure before storing in Qdrant.

        Args:
            payload: Payload dictionary to validate

        Returns:
            True if payload is valid, False otherwise
        """
        required_fields = [
            "text", "url", "section_heading",
            "chunk_index", "page_title", "chunk_id"
        ]

        for field in required_fields:
            if field not in payload:
                logger.error(f"Missing required field in payload: {field}")
                return False

        # Validate field types
        if not isinstance(payload.get("text"), str):
            logger.error("Payload 'text' field must be string")
            return False

        if not isinstance(payload.get("url"), str):
            logger.error("Payload 'url' field must be string")
            return False

        if not isinstance(payload.get("chunk_index"), int):
            logger.error("Payload 'chunk_index' field must be integer")
            return False

        if not isinstance(payload.get("chunk_id"), str):
            logger.error("Payload 'chunk_id' field must be string")
            return False

        return True

    def get_search_params(self) -> models.SearchParams:
        """
        Get default search parameters for similarity search.

        Returns:
            SearchParams object with default settings
        """
        return models.SearchParams(
            hnsw_ef=128,  # Number of vectors to consider during search
            exact=False    # Use approximate search for better performance
        )

    def get_optimization_config(self) -> models.OptimizersConfigDiff:
        """
        Get optimization configuration for the collection.

        Returns:
            OptimizersConfigDiff object with optimization settings
        """
        return models.OptimizersConfigDiff(
            # Memory optimization
            memmap_threshold=20000,      # Use memory mapping for segments larger than this
            indexing_threshold=20000,    # Index segments larger than this
            # Performance optimization
            max_optimization_threads=1,  # Limit threads to avoid overwhelming the system
            # Cleanup optimization
            deleted_threshold=0.2,       # Clean up deleted vectors when threshold reached
            vacuum_min_vector_number=1000  # Minimum vector count to trigger vacuum
        )