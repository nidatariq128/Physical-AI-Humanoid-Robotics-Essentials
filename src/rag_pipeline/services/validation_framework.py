"""
Validation Framework for the semantic retrieval pipeline
"""
import time
import random
from typing import List, Dict, Any, Tuple
from ..models.validation_result import ValidationResult
from .retrieval_service import RetrievalService
from ..logging import logger


class ValidationFramework:
    """
    Provides automated testing with known queries to validate retrieval accuracy
    """

    def __init__(self, retrieval_service: RetrievalService):
        """
        Initialize Validation Framework with retrieval service

        Args:
            retrieval_service: Instance of RetrievalService to test
        """
        self.retrieval_service = retrieval_service
        self.test_queries = self._load_test_queries()

    def _load_test_queries(self) -> List[Dict[str, Any]]:
        """
        Load or create test queries with expected results
        In a real implementation, this would load from a test data file

        Returns:
            List of test queries with expected results
        """
        # This is a basic set of test queries - in practice, these would come from
        # actual book content and validated results
        return [
            {
                "query": "What are the main principles of AI robotics?",
                "expected_content_keywords": ["AI", "robotics", "principles", "artificial intelligence"],
                "expected_metadata_keys": ["url", "section"]
            },
            {
                "query": "Explain the ethical considerations in AI development",
                "expected_content_keywords": ["ethical", "considerations", "AI", "development"],
                "expected_metadata_keys": ["url", "section"]
            },
            {
                "query": "How does machine learning differ from traditional programming?",
                "expected_content_keywords": ["machine learning", "traditional programming", "differences"],
                "expected_metadata_keys": ["url", "section"]
            }
        ]

    def validate_retrieval_accuracy(
        self,
        test_queries: List[Dict[str, Any]] = None,
        top_k: int = 5
    ) -> ValidationResult:
        """
        Validate retrieval accuracy using known queries and expected results

        Args:
            test_queries: List of test queries with expected results (uses default if None)
            top_k: Number of results to retrieve for validation

        Returns:
            ValidationResult object with accuracy metrics
        """
        start_time = time.time()

        if test_queries is None:
            test_queries = self.test_queries

        total_queries = len(test_queries)
        successful_queries = 0
        failed_queries = 0
        detailed_results = []
        response_times = []

        for i, test_query in enumerate(test_queries):
            try:
                query_text = test_query["query"]
                expected_keywords = test_query.get("expected_content_keywords", [])
                expected_metadata_keys = test_query.get("expected_metadata_keys", [])

                # Execute retrieval
                retrieval_start = time.time()
                result = self.retrieval_service.retrieve(query_text, top_k=top_k)
                retrieval_time = (time.time() - retrieval_start) * 1000
                response_times.append(retrieval_time)

                # Evaluate results
                precision, recall = self._evaluate_results(
                    result.results,
                    expected_keywords,
                    expected_metadata_keys
                )

                detailed_result = {
                    "query": query_text,
                    "precision": precision,
                    "recall": recall,
                    "result_count": len(result.results),
                    "response_time_ms": retrieval_time,
                    "passed": precision > 0.5  # Consider it passed if precision > 50%
                }

                detailed_results.append(detailed_result)

                if detailed_result["passed"]:
                    successful_queries += 1
                else:
                    failed_queries += 1

                logger.info(
                    f"Validation query {i+1}/{total_queries}: precision={precision:.2f}, "
                    f"recall={recall:.2f}, time={retrieval_time:.2f}ms"
                )

            except Exception as e:
                logger.error(f"Validation failed for query {i+1}: {str(e)}")
                failed_queries += 1

                detailed_result = {
                    "query": test_query["query"],
                    "precision": 0.0,
                    "recall": 0.0,
                    "result_count": 0,
                    "response_time_ms": -1,  # Error indicator
                    "passed": False,
                    "error": str(e)
                }
                detailed_results.append(detailed_result)

        # Calculate overall metrics
        execution_time = (time.time() - start_time) * 1000
        avg_response_time = sum(response_times) / len(response_times) if response_times else 0

        # Calculate overall accuracy metrics
        overall_precision = sum(r.get("precision", 0) for r in detailed_results if r.get("response_time_ms", -1) > 0) / \
                           max(1, len([r for r in detailed_results if r.get("response_time_ms", -1) > 0]))

        accuracy_metrics = {
            "precision": overall_precision,
            "recall": sum(r.get("recall", 0) for r in detailed_results if r.get("response_time_ms", -1) > 0) / \
                      max(1, len([r for r in detailed_results if r.get("response_time_ms", -1) > 0])),
            "success_rate": successful_queries / total_queries if total_queries > 0 else 0
        }

        # Calculate precision at different k values
        precision_at_k = {}
        for k in [1, 3, 5]:
            if k <= top_k:
                k_precision = self._calculate_precision_at_k(detailed_results, k)
                precision_at_k[k] = k_precision

        # Create validation result
        validation_result = ValidationResult(
            accuracy_metrics=accuracy_metrics,
            detailed_results=detailed_results,
            performance_metrics={
                "total_time_ms": execution_time,
                "average_response_time_ms": avg_response_time,
                "queries_per_second": len([r for r in detailed_results if r.get("response_time_ms", -1) > 0]) / (execution_time / 1000) if execution_time > 0 else 0
            },
            total_queries=total_queries,
            successful_queries=successful_queries,
            failed_queries=failed_queries,
            average_response_time=avg_response_time,
            precision_at_k=precision_at_k
        )

        # Log validation summary
        logger.info(
            f"Validation completed: total={total_queries}, successful={successful_queries}, "
            f"failed={failed_queries}, avg_time={avg_response_time:.2f}ms"
        )

        return validation_result

    def _evaluate_results(
        self,
        results: List[Any],
        expected_keywords: List[str],
        expected_metadata_keys: List[str]
    ) -> Tuple[float, float]:
        """
        Evaluate retrieval results against expected values

        Args:
            results: Retrieved results to evaluate
            expected_keywords: Expected keywords that should appear in results
            expected_metadata_keys: Expected metadata keys that should be present

        Returns:
            Tuple of (precision, recall) scores
        """
        if not results:
            return 0.0, 0.0

        # Count how many results contain expected keywords
        relevant_results = 0
        for result in results:
            content = getattr(result, 'content', '').lower()
            # Check if any expected keyword appears in the content
            if any(keyword.lower() in content for keyword in expected_keywords):
                relevant_results += 1

        # Calculate precision and recall
        precision = relevant_results / len(results) if results else 0.0
        recall = relevant_results / max(1, len(expected_keywords))  # Simplified recall calculation

        return precision, recall

    def _calculate_precision_at_k(self, detailed_results: List[Dict], k: int) -> float:
        """
        Calculate precision at k positions

        Args:
            detailed_results: List of detailed result dictionaries
            k: Position to calculate precision at

        Returns:
            Precision at k value
        """
        valid_results = [r for r in detailed_results if r.get("response_time_ms", -1) > 0 and len(r.get("results", [])) >= k]

        if not valid_results:
            return 0.0

        # Calculate precision at k for each valid result
        precisions_at_k = []
        for result in valid_results:
            # This is a simplified calculation - in practice would need to check top-k results
            if "precision" in result:
                precisions_at_k.append(result["precision"])

        return sum(precisions_at_k) / len(precisions_at_k) if precisions_at_k else 0.0

    def run_performance_benchmarks(
        self,
        query_patterns: List[str] = None,
        top_k_values: List[int] = [1, 3, 5, 10]
    ) -> Dict[str, Any]:
        """
        Run performance benchmarks with different query patterns and top-k values

        Args:
            query_patterns: List of query patterns to test (uses random if None)
            top_k_values: List of top-k values to benchmark

        Returns:
            Dictionary with performance benchmark results
        """
        if query_patterns is None:
            # Use some generic queries for benchmarking
            query_patterns = [
                "What is artificial intelligence?",
                "Explain machine learning concepts",
                "How do neural networks work?",
                "What are the applications of robotics?",
                "Describe ethical AI principles"
            ]

        benchmark_results = {}

        for top_k in top_k_values:
            logger.info(f"Running benchmark for top_k={top_k}")

            # Run multiple queries to get average performance
            times = []
            for query in query_patterns:
                start_time = time.time()
                try:
                    self.retrieval_service.retrieve(query, top_k=top_k)
                    elapsed = (time.time() - start_time) * 1000
                    times.append(elapsed)
                except Exception as e:
                    logger.error(f"Benchmark query failed: {str(e)}")

            if times:
                avg_time = sum(times) / len(times)
                min_time = min(times)
                max_time = max(times)

                benchmark_results[f"top_k_{top_k}"] = {
                    "avg_response_time_ms": avg_time,
                    "min_response_time_ms": min_time,
                    "max_response_time_ms": max_time,
                    "query_count": len(times),
                    "p95_response_time_ms": self._calculate_percentile(times, 95) if times else 0
                }

        return benchmark_results

    def _calculate_percentile(self, data: List[float], percentile: float) -> float:
        """
        Calculate percentile of response times

        Args:
            data: List of response times
            percentile: Percentile to calculate (e.g., 95 for p95)

        Returns:
            Percentile value
        """
        if not data:
            return 0.0

        sorted_data = sorted(data)
        index = int(len(sorted_data) * percentile / 100)
        return sorted_data[min(index, len(sorted_data) - 1)]

    def generate_test_queries_from_content(self, sample_content: List[str], count: int = 10) -> List[Dict[str, Any]]:
        """
        Generate test queries based on actual content

        Args:
            sample_content: List of content samples to generate queries from
            count: Number of test queries to generate

        Returns:
            List of test queries with expected results
        """
        test_queries = []

        for i in range(min(count, len(sample_content))):
            content = sample_content[i]
            # Extract key terms from content to use as expected keywords
            words = content.split()
            key_terms = list(set(word for word in words if len(word) > 5))[:5]  # Take up to 5 key terms

            # Create a query that would likely match this content
            query = f"What is {' '.join(key_terms[:2])}?"

            test_queries.append({
                "query": query,
                "expected_content_keywords": key_terms,
                "expected_metadata_keys": ["url", "section"]
            })

        return test_queries