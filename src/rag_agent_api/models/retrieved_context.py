"""
Retrieved Context model for the RAG Agent API
"""
from typing import List, Dict, Optional
from pydantic import BaseModel


class RetrievedContext(BaseModel):
    """
    Content chunks retrieved from Qdrant that are relevant to the question
    """
    results: List[Dict]  # List of content chunks with metadata
    query_vector: Optional[List[float]] = None  # The embedding vector used for search
    execution_time_ms: float  # Time taken for retrieval
    query_text: str  # Original query text


class SourceCitation(BaseModel):
    """
    Reference information indicating where in the book content the response information originated
    """
    source: str  # Source identifier (e.g., URL, document ID)
    content: str  # The actual content that was cited
    score: Optional[float] = None  # Relevance score
    metadata: Optional[Dict[str, str]] = None  # Additional metadata like page, section, etc.