"""
Error Handler for the RAG Agent API
Manages error cases and ensures graceful degradation
"""
from typing import Dict, Any, Optional
from ..logger import logger


class ErrorCategory:
    """Categories of errors that can occur in the system"""
    RETRIEVAL_ERROR = "retrieval_error"
    AGENT_ERROR = "agent_error"
    VALIDATION_ERROR = "validation_error"
    CONFIGURATION_ERROR = "configuration_error"
    NETWORK_ERROR = "network_error"
    RATE_LIMIT_ERROR = "rate_limit_error"


class ErrorHandler:
    """
    Manage error cases and ensure graceful degradation
    """

    def __init__(self):
        pass

    def handle_error(self, error: Exception, operation: str, context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Handle an error and return appropriate response.

        Args:
            error: The exception that occurred
            operation: The operation where the error occurred
            context: Additional context about the error

        Returns:
            Dictionary with error details and fallback response
        """
        error_type = type(error).__name__
        error_message = str(error)

        # Categorize the error
        category = self._categorize_error(error, operation)

        # Log the error
        logger.log_error(operation, error, str(context))

        # Determine appropriate response based on error category
        fallback_response = self._get_fallback_response(category, error_message)

        return {
            "error_type": error_type,
            "error_message": error_message,
            "category": category,
            "fallback_response": fallback_response,
            "handled": True
        }

    def _categorize_error(self, error: Exception, operation: str) -> str:
        """
        Categorize an error based on its type and context.

        Args:
            error: The exception to categorize
            operation: The operation where the error occurred

        Returns:
            Error category string
        """
        error_type = type(error).__name__
        error_message = str(error).lower()

        # Check for specific error patterns
        if "qdrant" in error_message or "retrieval" in operation or "search" in operation:
            return ErrorCategory.RETRIEVAL_ERROR
        elif "gemini" in error_message or "google" in error_message or "agent" in operation or "rate" in error_message or "quota" in error_message:
            return ErrorCategory.RATE_LIMIT_ERROR
        elif "validation" in operation or "grounding" in operation:
            return ErrorCategory.VALIDATION_ERROR
        elif "config" in error_message or "env" in error_message:
            return ErrorCategory.CONFIGURATION_ERROR
        elif "connection" in error_message or "timeout" in error_message or "network" in error_message:
            return ErrorCategory.NETWORK_ERROR
        else:
            return ErrorCategory.AGENT_ERROR

    def _get_fallback_response(self, category: str, error_message: str) -> str:
        """
        Get an appropriate fallback response based on error category.

        Args:
            category: The error category
            error_message: The original error message

        Returns:
            Fallback response string
        """
        fallback_responses = {
            ErrorCategory.RETRIEVAL_ERROR: (
                "I'm currently unable to retrieve information from the knowledge base. "
                "This may be due to a temporary issue with the search system. "
                "Please try your question again later."
            ),
            ErrorCategory.RATE_LIMIT_ERROR: (
                "I've reached my usage limits and can't process your request right now. "
                "Please try again in a few minutes when my capacity is refreshed."
            ),
            ErrorCategory.VALIDATION_ERROR: (
                "I was unable to verify that my response is fully supported by the knowledge base. "
                "To maintain accuracy, I cannot provide an answer for this query at this time."
            ),
            ErrorCategory.CONFIGURATION_ERROR: (
                "The system is experiencing configuration issues and cannot process your request. "
                "Please contact the system administrator."
            ),
            ErrorCategory.NETWORK_ERROR: (
                "I'm experiencing connectivity issues and cannot access the required services. "
                "Please try again later."
            ),
            ErrorCategory.AGENT_ERROR: (
                "An unexpected error occurred while processing your request. "
                "Please try again or rephrase your question."
            )
        }

        return fallback_responses.get(category,
            "An unexpected error occurred. Please try rephrasing your question or try again later."
        )

    def handle_empty_retrieval(self, query: str) -> Dict[str, Any]:
        """
        Handle the case where no relevant content is found in the knowledge base.

        Args:
            query: The original query that yielded no results

        Returns:
            Dictionary with appropriate response for empty retrieval
        """
        logger.logger.info(f"No relevant content found for query: {query}")

        return {
            "answer": (
                "I couldn't find any relevant information in the knowledge base to answer your question: "
                f"'{query}'. The topic might not be covered in the available documents, "
                "or the wording of your question might not match the content in the knowledge base. "
                "Please try rephrasing your question or ask about a different topic."
            ),
            "citations": [],
            "grounded": True,  # Technically grounded since we're truthfully saying no info exists
            "retrieval_results_count": 0,
            "handled": True
        }

    def should_degrade_gracefully(self, error_category: str) -> bool:
        """
        Determine if the system should attempt graceful degradation for this error type.

        Args:
            error_category: The category of error

        Returns:
            True if graceful degradation is appropriate, False otherwise
        """
        # These error types are temporary and the system can still function
        degrade_gracefully = [
            ErrorCategory.RATE_LIMIT_ERROR,
            ErrorCategory.NETWORK_ERROR,
            ErrorCategory.RETRIEVAL_ERROR
        ]

        return error_category in degrade_gracefully

    def generate_degraded_response(self, original_query: str, error_category: str) -> Dict[str, Any]:
        """
        Generate a degraded response when full functionality isn't available.

        Args:
            original_query: The original user query
            error_category: The category of error that triggered degradation

        Returns:
            Dictionary with degraded response
        """
        if error_category == ErrorCategory.RATE_LIMIT_ERROR:
            response_text = (
                "I've reached my usage limits and can't fully process your question: "
                f"'{original_query}'. I can only provide limited assistance at this time. "
                "Please try again later when my capacity is refreshed."
            )
        elif error_category == ErrorCategory.NETWORK_ERROR:
            response_text = (
                "Due to connectivity issues, I can only provide basic assistance for your question: "
                f"'{original_query}'. Some features are temporarily unavailable."
            )
        elif error_category == ErrorCategory.RETRIEVAL_ERROR:
            response_text = (
                "I'm having trouble accessing the knowledge base to answer your question: "
                f"'{original_query}'. I can only provide limited assistance at this time."
            )
        else:
            response_text = (
                "I'm experiencing issues and can only provide basic assistance for your question: "
                f"'{original_query}'. Some features are temporarily unavailable."
            )

        return {
            "answer": response_text,
            "citations": [],
            "grounded": True,
            "retrieval_results_count": 0,
            "degraded_mode": True
        }