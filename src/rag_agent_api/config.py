"""
Configuration for the RAG Agent API
"""
import os
from typing import Optional
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


class Config:
    """Configuration class for RAG Agent API"""

    def __init__(self):
        # OpenRouter API configuration
        self.openrouter_api_key: str = os.getenv('OPENROUTER_API_KEY', '')
        self.openrouter_model: str = os.getenv('OPENROUTER_MODEL', 'xiaomi/mimo-v2-flash:free')  # Default model for OpenRouter
        self.agent_model: str = os.getenv('AGENT_MODEL', 'xiaomi/mimo-v2-flash:free')  # Model used for agent operations

        # Qdrant configuration
        self.qdrant_url: str = os.getenv('QDRANT_URL', '')
        self.qdrant_api_key: str = os.getenv('QDRANT_API_KEY', '')
        self.qdrant_collection_name: str = os.getenv('QDRANT_COLLECTION_NAME', 'book_content_chunks')

        # API configuration
        self.default_top_k: int = int(os.getenv('DEFAULT_TOP_K', '5'))
        self.max_top_k: int = int(os.getenv('MAX_TOP_K', '20'))
        self.max_retrieval_time_ms: int = int(os.getenv('MAX_RETRIEVAL_TIME_MS', '5000'))

        # Agent configuration
        self.agent_timeout_seconds: int = int(os.getenv('AGENT_TIMEOUT_SECONDS', '30'))
        self.max_tokens: int = int(os.getenv('MAX_TOKENS', '2000'))
        self.temperature: float = float(os.getenv('TEMPERATURE', '0.1'))

    def validate(self) -> bool:
        """
        Validate that all required configuration values are present

        Returns:
            True if all required values are present, False otherwise
        """
        required_fields = [
            self.openrouter_api_key,
            self.qdrant_url,
            self.qdrant_api_key,
            self.qdrant_collection_name
        ]

        return all(field.strip() != '' for field in required_fields)

    def get_missing_fields(self) -> list:
        """
        Get list of missing required configuration fields

        Returns:
            List of missing field names
        """
        missing = []
        if not self.openrouter_api_key:
            missing.append('OPENROUTER_API_KEY')
        if not self.qdrant_url:
            missing.append('QDRANT_URL')
        if not self.qdrant_api_key:
            missing.append('QDRANT_API_KEY')
        if not self.qdrant_collection_name:
            missing.append('QDRANT_COLLECTION_NAME')

        return missing


# Create a global config instance
config = Config()