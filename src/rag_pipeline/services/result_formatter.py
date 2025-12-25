"""
Result Formatter service for the semantic retrieval pipeline
"""
from typing import List
from ..models.content_chunk import ContentChunk, RetrievalResult
from ..logging import logger


class ResultFormatter:
    """
    Transform raw Qdrant responses into standardized retrieval results
    """

    def format_retrieval_result(
        self,
        query_text: str,
        query_vector: List[float],
        raw_results: List[ContentChunk],
        execution_time_ms: float
    ) -> RetrievalResult:
        """
        Format raw Qdrant results into standardized RetrievalResult object

        Args:
            query_text: Original query text
            query_vector: Query embedding vector
            raw_results: Raw ContentChunk results from Qdrant
            execution_time_ms: Time taken for retrieval

        Returns:
            Standardized RetrievalResult object
        """
        # Sort results by score in descending order
        sorted_results = sorted(
            raw_results,
            key=lambda x: x.score if x.score is not None else 0,
            reverse=True
        )

        # Create RetrievalResult object
        retrieval_result = RetrievalResult(
            results=sorted_results,
            query_vector=query_vector,
            execution_time_ms=execution_time_ms,
            query_text=query_text
        )

        # Log the formatted results
        self._log_formatting_results(query_text, len(raw_results), execution_time_ms)

        return retrieval_result

    def _log_formatting_results(self, query_text: str, result_count: int, execution_time_ms: float):
        """
        Log information about the formatting operation
        """
        logger.info(
            f"Formatted retrieval results: query_length={len(query_text)}, "
            f"result_count={result_count}, execution_time_ms={execution_time_ms:.2f}"
        )

    def normalize_scores(self, results: List[ContentChunk], method: str = "linear") -> List[ContentChunk]:
        """
        Normalize scores to a standard range (e.g., 0-1)

        Args:
            results: List of ContentChunk objects with scores
            method: Normalization method ('linear', 'softmax', etc.)

        Returns:
            List of ContentChunk objects with normalized scores
        """
        if not results or all(r.score is None for r in results):
            return results

        # Filter out results without scores
        valid_results = [r for r in results if r.score is not None]
        if not valid_results:
            return results

        if method == "linear":
            # Linear normalization to [0, 1] range
            scores = [r.score for r in valid_results]
            min_score = min(scores)
            max_score = max(scores)

            if max_score == min_score:
                # All scores are the same, assign 1.0 to all
                for r in valid_results:
                    r.score = 1.0
            else:
                # Normalize to [0, 1] range
                range_score = max_score - min_score
                for r in valid_results:
                    r.score = (r.score - min_score) / range_score

        # Return the list with normalized scores (maintaining original order)
        normalized_results = []
        for original in results:
            if original in valid_results:
                normalized_results.append(original)
            else:
                normalized_results.append(original)

        return normalized_results

    def extract_metadata_values(self, results: List[ContentChunk], key: str) -> List[str]:
        """
        Extract all values for a specific metadata key from results

        Args:
            results: List of ContentChunk objects
            key: Metadata key to extract

        Returns:
            List of values for the specified key
        """
        values = []
        for result in results:
            if key in result.metadata:
                value = result.metadata[key]
                if value not in values:  # Avoid duplicates
                    values.append(value)
        return values