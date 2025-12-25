"""Configuration management for the RAG Knowledge Ingestion Pipeline."""

import os
from typing import Optional
from dataclasses import dataclass
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()


@dataclass
class Config:
    """Configuration class holding all settings for the RAG pipeline."""

    # Cohere API configuration
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")

    # Qdrant Cloud configuration
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "document_chunks")

    # Docusaurus site to crawl
    docusaurus_site_url: str = os.getenv("DOCUSAURUS_SITE_URL", "https://example.com")

    # Chunking configuration
    chunk_size: int = int(os.getenv("CHUNK_SIZE", "1000"))
    chunk_overlap: int = int(os.getenv("CHUNK_OVERLAP", "200"))

    # Logging configuration
    log_level: str = os.getenv("LOG_LEVEL", "INFO")

    # Local mode (skip external API calls)
    local_mode: bool = os.getenv("LOCAL_MODE", "false").lower() == "true"

    # API configuration (for compatibility with retrieval service)
    default_top_k: int = int(os.getenv('DEFAULT_TOP_K', '5'))
    max_top_k: int = int(os.getenv('MAX_TOP_K', '20'))
    max_retrieval_time_ms: int = int(os.getenv('MAX_RETRIEVAL_TIME_MS', '5000'))

    def validate(self) -> bool:
        """Validate required configuration."""
        errors = []

        if not self.local_mode:  # Only validate API keys if not in local mode
            if not self.cohere_api_key:
                errors.append("COHERE_API_KEY environment variable must be set")

            if not self.qdrant_api_key:
                errors.append("QDRANT_API_KEY environment variable must be set")

            if not self.qdrant_url:
                errors.append("QDRANT_URL environment variable must be set")

        return len(errors) == 0  # Return True if no errors

    def get_missing_fields(self):
        """Get list of missing required configuration fields (for compatibility)."""
        missing = []
        if not self.cohere_api_key:
            missing.append('COHERE_API_KEY')
        if not self.qdrant_api_key:
            missing.append('QDRANT_API_KEY')
        if not self.qdrant_url:
            missing.append('QDRANT_URL')
        if not self.qdrant_collection_name:
            missing.append('QDRANT_COLLECTION_NAME')

        return missing


# Global configuration instance
config = Config()