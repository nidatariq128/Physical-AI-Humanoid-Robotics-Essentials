"""
Data models for the semantic retrieval pipeline
"""
from typing import List, Dict, Optional
from pydantic import BaseModel


class ContentChunk(BaseModel):
    """
    Represents a segment of book content with associated vector embeddings,
    metadata, and text content
    """
    id: str
    content: str
    metadata: Dict[str, str]  # Contains url, section, page_number, source_document
    vector: Optional[List[float]] = None  # Embedding vector (may not be returned in results)
    score: Optional[float] = None  # Similarity score from retrieval


class Query(BaseModel):
    """
    Represents a text input from the user that gets converted to a vector
    for semantic comparison against stored content chunks
    """
    text: str
    vector: Optional[List[float]] = None  # Will be populated after embedding conversion


class RetrievalResult(BaseModel):
    """
    Represents a ranked list of content chunks with relevance scores,
    metadata, and the original text content
    """
    results: List[ContentChunk]
    query_vector: List[float]
    execution_time_ms: float
    query_text: str


class MetadataFilter(BaseModel):
    """
    Parameters that constrain retrieval results based on document properties
    """
    filters: Dict[str, str]  # Key-value pairs for metadata filtering