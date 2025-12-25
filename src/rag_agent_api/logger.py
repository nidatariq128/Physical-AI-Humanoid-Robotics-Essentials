"""
Logging infrastructure for the RAG Agent API
"""
import logging
import time
from typing import List, Dict, Any, Optional
from datetime import datetime
import json


class AgentLogger:
    """
    Records agent operations, metrics, and errors for the RAG Agent API
    """

    def __init__(self, name: str = "rag_agent"):
        """
        Initialize the agent logger
        """
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.INFO)

        # Add handler if not already present
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)

    def log_question_request(
        self,
        question: str,
        text_scope: Optional[str],
        top_k: int,
        execution_time_ms: float
    ):
        """
        Log incoming question requests

        Args:
            question: The question text
            text_scope: Optional text scope provided by user
            top_k: Number of results requested
            execution_time_ms: Total execution time
        """
        log_data = {
            "timestamp": datetime.utcnow().isoformat(),
            "event": "question_request",
            "question_length": len(question),
            "has_text_scope": text_scope is not None,
            "text_scope_length": len(text_scope) if text_scope else 0,
            "top_k": top_k,
            "execution_time_ms": execution_time_ms
        }

        self.logger.info(f"QUESTION_REQUEST: {json.dumps(log_data)}")

    def log_retrieval_operation(
        self,
        query_text: str,
        results_count: int,
        execution_time_ms: float,
        filters: Optional[Dict[str, str]] = None
    ):
        """
        Log retrieval operations

        Args:
            query_text: The query text used for retrieval
            results_count: Number of results returned
            execution_time_ms: Time taken for retrieval
            filters: Optional filters applied
        """
        log_data = {
            "timestamp": datetime.utcnow().isoformat(),
            "event": "retrieval",
            "query_length": len(query_text),
            "results_count": results_count,
            "execution_time_ms": execution_time_ms,
            "filters_applied": filters or {}
        }

        self.logger.info(f"RETRIEVAL: {json.dumps(log_data)}")

    def log_agent_response(
        self,
        question: str,
        answer_length: int,
        citations_count: int,
        grounded: bool,
        execution_time_ms: float
    ):
        """
        Log agent responses

        Args:
            question: Original question
            answer_length: Length of the generated answer
            citations_count: Number of citations provided
            grounded: Whether response is grounded in context
            execution_time_ms: Total execution time
        """
        log_data = {
            "timestamp": datetime.utcnow().isoformat(),
            "event": "agent_response",
            "question_length": len(question),
            "answer_length": answer_length,
            "citations_count": citations_count,
            "grounded": grounded,
            "execution_time_ms": execution_time_ms
        }

        self.logger.info(f"AGENT_RESPONSE: {json.dumps(log_data)}")

    def log_error(self, operation: str, error: Exception, context: str = None):
        """
        Log errors during agent operations

        Args:
            operation: Name of the operation that failed
            error: The error that occurred
            context: Additional context about the error
        """
        log_data = {
            "timestamp": datetime.utcnow().isoformat(),
            "event": "error",
            "operation": operation,
            "error_type": type(error).__name__,
            "error_message": str(error),
            "context": context
        }

        self.logger.error(f"ERROR: {json.dumps(log_data)}")

    def log_validation_result(
        self,
        response: str,
        is_valid: bool,
        validation_details: Dict[str, Any]
    ):
        """
        Log response validation results

        Args:
            response: The response being validated
            is_valid: Whether the response passed validation
            validation_details: Details about the validation process
        """
        log_data = {
            "timestamp": datetime.utcnow().isoformat(),
            "event": "validation",
            "response_length": len(response),
            "is_valid": is_valid,
            "validation_details": validation_details
        }

        self.logger.info(f"VALIDATION: {json.dumps(log_data)}")


# Global logger instance
logger = AgentLogger()