"""Custom exception classes for the RAG Knowledge Ingestion Pipeline."""


class RAGPipelineError(Exception):
    """Base exception class for RAG pipeline errors."""
    pass


class CrawlerError(RAGPipelineError):
    """Exception raised when crawling operations fail."""
    pass


class ContentExtractionError(RAGPipelineError):
    """Exception raised when content extraction fails."""
    pass


class ChunkingError(RAGPipelineError):
    """Exception raised when content chunking fails."""
    pass


class EmbeddingError(RAGPipelineError):
    """Exception raised when embedding generation fails."""
    pass


class StorageError(RAGPipelineError):
    """Exception raised when storage operations fail."""
    pass


class ValidationError(RAGPipelineError):
    """Exception raised when validation fails."""
    pass


class ConfigurationError(RAGPipelineError):
    """Exception raised when configuration is invalid."""
    pass