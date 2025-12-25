"""Base models for the RAG Knowledge Ingestion Pipeline."""

from dataclasses import dataclass
from typing import List, Dict, Any, Optional
from datetime import datetime
import uuid


@dataclass
class SourceReference:
    """Information linking a chunk back to its origin."""

    url: str
    section_heading: str
    chunk_index: int
    page_title: str = ""
    parent_section: str = ""


@dataclass
class EmbeddingVector:
    """Numerical representation of document chunk content."""

    vector: List[float]
    model_name: str
    created_at: datetime = None

    def __post_init__(self):
        if self.created_at is None:
            self.created_at = datetime.utcnow()


@dataclass
class DocumentChunk:
    """A segment of extracted content with semantic meaning."""

    text: str
    source_reference: SourceReference
    embedding: Optional[EmbeddingVector] = None
    metadata: Dict[str, Any] = None
    chunk_id: Optional[str] = None

    def __post_init__(self):
        if self.metadata is None:
            self.metadata = {}
        if self.chunk_id is None:
            # Generate a proper UUID for Qdrant compatibility
            self.chunk_id = str(uuid.uuid4())