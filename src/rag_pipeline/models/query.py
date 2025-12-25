"""
Query model for the semantic retrieval pipeline
"""
from typing import List, Optional
from pydantic import BaseModel


class Query(BaseModel):
    """
    Represents a text input from the user that gets converted to a vector
    for semantic comparison against stored content chunks
    """
    text: str
    vector: Optional[List[float]] = None  # Will be populated after embedding conversion
    top_k: int = 5  # Number of results to retrieve