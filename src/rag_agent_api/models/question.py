"""
Data models for the RAG Agent API
"""
from typing import Optional
from pydantic import BaseModel, Field
import re


class Question(BaseModel):
    """
    A user query submitted to the RAG agent API, including optional text scope parameters
    """
    question: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="The question to answer (1-2000 characters)"
    )
    text_scope: Optional[str] = Field(
        None,
        max_length=500,
        description="Optional user-selected text to scope the query (max 500 characters)"
    )  # Optional user-selected text to scope the query
    top_k: Optional[int] = Field(
        default=5,
        ge=1,
        le=20,
        description="Number of results to retrieve (1-20, default 5)"
    )  # Number of results to retrieve

    def __init__(self, **data):
        super().__init__(**data)
        # Additional validation
        if self.question and not self.question.strip():
            raise ValueError("Question cannot be empty or just whitespace")
        if self.text_scope and not self.text_scope.strip():
            raise ValueError("Text scope cannot be empty or just whitespace")


class APIResponse(BaseModel):
    """
    The complete response object containing the answer and metadata
    """
    answer: str
    citations: list[dict]  # List of source citations with format {source: str, content: str}
    retrieval_results_count: int
    grounded: bool  # Whether the response is grounded in the retrieved context
    execution_time_ms: float