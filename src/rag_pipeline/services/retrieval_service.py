"""
Main Retrieval Service for the semantic retrieval pipeline
"""
import time
from typing import List, Dict, Optional, Any
from ..models.content_chunk import ContentChunk, RetrievalResult
from ..models.query import Query
from ..models.metadata_filter import MetadataFilter
from .query_processor import QueryProcessor
from .qdrant_client import QdrantClientInterface, QdrantConfig
from .result_formatter import ResultFormatter
from .filter_engine import FilterEngine
from ..config import config
from ..logging import logger


class RetrievalService:
    """
    Main service that orchestrates the semantic retrieval process
    """

    def __init__(self):
        """
        Initialize the retrieval service with all required components
        """
        # Initialize components
        self.query_processor = QueryProcessor()
        self.result_formatter = ResultFormatter()
        self.filter_engine = FilterEngine()

        # Initialize Qdrant client
        qdrant_config = QdrantConfig(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            collection_name=config.qdrant_collection_name
        )
        self.qdrant_client = QdrantClientInterface(qdrant_config)

        # Validate configuration
        if not config.validate():
            missing_fields = config.get_missing_fields()
            logger.error(f"Missing configuration fields: {missing_fields}")
            raise ValueError(f"Configuration validation failed. Missing fields: {missing_fields}")

        # Verify Qdrant connection
        if not self.qdrant_client.verify_connection():
            logger.error("Failed to connect to Qdrant")
            raise ConnectionError("Could not establish connection to Qdrant")

        logger.info("Retrieval Service initialized successfully")

    def retrieve(
        self,
        query_text: str,
        top_k: Optional[int] = None,
        filters: Optional[Dict[str, str]] = None
    ) -> RetrievalResult:
        """
        Main retrieval function that performs semantic search against Qdrant

        Args:
            query_text: Text query to search for
            top_k: Number of results to retrieve (defaults to config default)
            filters: Optional metadata filters to apply

        Returns:
            RetrievalResult object with results and metadata
        """
        start_time = time.time()

        try:
            # Validate inputs
            if not query_text or not query_text.strip():
                raise ValueError("Query text cannot be empty")

            # Set default top_k if not provided
            if top_k is None:
                top_k = config.default_top_k
            else:
                # Validate top_k is within acceptable range
                if top_k <= 0:
                    raise ValueError(f"top_k must be positive, got {top_k}")
                if top_k > config.max_top_k:
                    logger.warning(f"top_k {top_k} exceeds maximum {config.max_top_k}, using maximum")
                    top_k = config.max_top_k

            # Process the query to get embedding vector
            query_obj = self.query_processor.process_query(query_text)
            query_vector = query_obj.vector

            # Apply filters if provided
            qdrant_filters = filters if filters else {}

            # Execute search against Qdrant
            raw_results = self.qdrant_client.search(
                query_vector=query_vector,
                top_k=top_k,
                filters=qdrant_filters
            )

            # Calculate execution time
            execution_time_ms = (time.time() - start_time) * 1000

            # Format results
            retrieval_result = self.result_formatter.format_retrieval_result(
                query_text=query_text,
                query_vector=query_vector,
                raw_results=raw_results,
                execution_time_ms=execution_time_ms
            )

            # Log successful retrieval
            logger.info(
                f"Retrieval completed: query_length={len(query_text)}, "
                f"results_returned={len(raw_results)}, execution_time_ms={execution_time_ms:.2f}"
            )

            # Check performance against requirements
            if execution_time_ms > config.max_retrieval_time_ms:
                logger.warning(
                    f"Retrieval exceeded time requirement: {execution_time_ms:.2f}ms > "
                    f"{config.max_retrieval_time_ms}ms"
                )

            return retrieval_result

        except Exception as e:
            execution_time_ms = (time.time() - start_time) * 1000
            logger.error(f"Retrieval failed after {execution_time_ms:.2f}ms: {str(e)}")
            raise

    def retrieve_batch(
        self,
        query_texts: List[str],
        top_k: Optional[int] = None,
        filters: Optional[Dict[str, str]] = None
    ) -> List[RetrievalResult]:
        """
        Retrieve results for multiple queries

        Args:
            query_texts: List of text queries to search for
            top_k: Number of results to retrieve per query
            filters: Optional metadata filters to apply to all queries

        Returns:
            List of RetrievalResult objects
        """
        results = []
        for query_text in query_texts:
            result = self.retrieve(query_text, top_k, filters)
            results.append(result)

        return results

    def get_collection_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the Qdrant collection

        Returns:
            Dictionary with collection statistics
        """
        total_points = self.qdrant_client.get_total_points()
        model_info = self.query_processor.get_embedding_model_info()

        stats = {
            "total_documents": total_points,
            "embedding_model": model_info.get("model_name"),
            "vector_dimensions": model_info.get("dimensions"),
            "default_top_k": config.default_top_k,
            "max_top_k": config.max_top_k
        }

        return stats

    def validate_query(self, query_text: str) -> List[str]:
        """
        Validate a query before processing

        Args:
            query_text: Query text to validate

        Returns:
            List of validation errors (empty if valid)
        """
        errors = []

        if not query_text or not query_text.strip():
            errors.append("Query text cannot be empty")

        if len(query_text.strip()) < 3:
            errors.append("Query text is too short (minimum 3 characters)")

        if len(query_text) > 1000:
            errors.append("Query text is too long (maximum 1000 characters)")

        return errors