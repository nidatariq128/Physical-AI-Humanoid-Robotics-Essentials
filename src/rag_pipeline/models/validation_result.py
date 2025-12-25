"""
Validation Result model for the semantic retrieval pipeline
"""
from typing import List, Dict, Any, Optional
from pydantic import BaseModel


class ValidationResult(BaseModel):
    """
    Represents the results of validation testing with accuracy metrics
    """
    accuracy_metrics: Dict[str, float]  # precision, recall, etc.
    detailed_results: List[Dict[str, Any]]  # Detailed per-query results
    performance_metrics: Dict[str, float]  # latency, throughput, etc.
    total_queries: int
    successful_queries: int
    failed_queries: int
    average_response_time: float
    precision_at_k: Optional[Dict[int, float]] = None  # Precision at different k values