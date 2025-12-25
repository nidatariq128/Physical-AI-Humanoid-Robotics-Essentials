"""Module for validating the RAG ingestion pipeline with sample similarity queries."""

from typing import List, Dict, Any, Optional
from src.rag_pipeline.storage.qdrant_client import QdrantStorage
from src.rag_pipeline.embedder.cohere_embedder import CohereEmbedder
from src.rag_pipeline.logging import logger
from src.rag_pipeline.exceptions import ValidationError


class IngestionValidator:
    """Class to validate the ingestion pipeline with sample queries."""

    def __init__(self):
        """Initialize the validator with required components."""
        self.qdrant_storage = QdrantStorage()
        self.embedder = CohereEmbedder()

    def validate_ingestion(self, sample_queries: Optional[List[str]] = None) -> Dict[str, Any]:
        """
        Validate the ingestion by running sample similarity queries.

        Args:
            sample_queries: Optional list of sample queries to test

        Returns:
            Dictionary with validation results
        """
        if sample_queries is None:
            sample_queries = [
                "What is the main topic of this documentation?",
                "How do I get started with this system?",
                "What are the key features described?",
                "Can you explain the architecture?",
                "What are the prerequisites?"
            ]

        results = {
            "total_queries": len(sample_queries),
            "successful_queries": 0,
            "failed_queries": 0,
            "query_results": [],
            "validation_passed": False
        }

        logger.info(f"Starting ingestion validation with {len(sample_queries)} sample queries")

        for i, query in enumerate(sample_queries):
            try:
                logger.info(f"Running validation query {i+1}/{len(sample_queries)}: '{query}'")

                # Generate embedding for the query
                query_embedding = self.embedder.generate_embedding(query)

                # Search for similar content in Qdrant
                search_results = self.qdrant_storage.search_similar(
                    query_embedding.vector,
                    top_k=3
                )

                query_result = {
                    "query": query,
                    "results_count": len(search_results),
                    "results": search_results,
                    "status": "success"
                }

                results["query_results"].append(query_result)
                results["successful_queries"] += 1

                logger.info(f"Query '{query}' returned {len(search_results)} results")

            except Exception as e:
                logger.error(f"Validation query failed: {str(e)}")
                query_result = {
                    "query": query,
                    "results_count": 0,
                    "results": [],
                    "status": "failed",
                    "error": str(e)
                }

                results["query_results"].append(query_result)
                results["failed_queries"] += 1

        # Determine if validation passed based on success rate
        success_rate = results["successful_queries"] / results["total_queries"]
        results["success_rate"] = success_rate
        results["validation_passed"] = success_rate >= 0.8  # 80% success rate required

        logger.info(f"Ingestion validation completed. Success rate: {success_rate:.2%}")
        return results

    def validate_data_integrity(self) -> Dict[str, Any]:
        """
        Validate the integrity of stored data in Qdrant.

        Returns:
            Dictionary with integrity validation results
        """
        results = {
            "validation_passed": False,
            "total_chunks": 0,
            "chunks_with_text": 0,
            "chunks_with_urls": 0,
            "chunks_with_embeddings": 0,
            "integrity_issues": []
        }

        logger.info("Starting data integrity validation")

        try:
            # Get a sample of chunks to validate
            # Note: This is a simplified check - in a real implementation,
            # you might want to validate the entire collection
            sample_query = "sample"
            sample_embedding = self.embedder.generate_embedding(sample_query)

            # Get some results to validate
            sample_results = self.qdrant_storage.search_similar(
                sample_embedding.vector,
                top_k=10
            )

            results["total_chunks"] = len(sample_results)

            for result in sample_results:
                text = result.get("text", "")
                url = result.get("url", "")

                if text.strip():
                    results["chunks_with_text"] += 1
                else:
                    results["integrity_issues"].append(f"Chunk {result.get('id')} has empty text")

                if url.strip():
                    results["chunks_with_urls"] += 1
                else:
                    results["integrity_issues"].append(f"Chunk {result.get('id')} has empty URL")

            # For this validation, assume if we got results, the basic integrity is OK
            results["validation_passed"] = len(sample_results) > 0 and len(results["integrity_issues"]) == 0

            logger.info(f"Data integrity validation completed: {results}")
            return results

        except Exception as e:
            logger.error(f"Data integrity validation failed: {str(e)}")
            results["integrity_issues"].append(f"Validation failed with error: {str(e)}")
            return results

    def validate_metadata_completeness(self) -> Dict[str, Any]:
        """
        Validate that metadata fields are properly stored.

        Returns:
            Dictionary with metadata validation results
        """
        results = {
            "validation_passed": False,
            "required_fields_check": {},
            "metadata_completeness": 0.0,
            "total_samples_checked": 0
        }

        logger.info("Starting metadata completeness validation")

        try:
            # Test with a sample query
            sample_query = "metadata validation"
            sample_embedding = self.embedder.generate_embedding(sample_query)

            sample_results = self.qdrant_storage.search_similar(
                sample_embedding.vector,
                top_k=5
            )

            if not sample_results:
                logger.warning("No results found for metadata validation")
                results["validation_passed"] = False
                return results

            required_fields = ["text", "url", "section_heading", "chunk_index", "page_title"]

            for field in required_fields:
                results["required_fields_check"][field] = 0

            for result in sample_results:
                results["total_samples_checked"] += 1

                for field in required_fields:
                    if result.get(field) is not None and str(result[field]).strip():
                        results["required_fields_check"][field] += 1

            # Calculate completeness percentage
            total_field_checks = len(required_fields) * results["total_samples_checked"]
            total_filled_fields = sum(results["required_fields_check"].values())

            if total_field_checks > 0:
                results["metadata_completeness"] = total_filled_fields / total_field_checks
                results["validation_passed"] = results["metadata_completeness"] >= 0.9  # 90% completeness required

            logger.info(f"Metadata validation completed: {results}")
            return results

        except Exception as e:
            logger.error(f"Metadata validation failed: {str(e)}")
            return results

    def run_complete_validation(self) -> Dict[str, Any]:
        """
        Run complete validation of the ingestion pipeline.

        Returns:
            Dictionary with complete validation results
        """
        logger.info("Starting complete ingestion validation")

        validation_results = {
            "timestamp": __import__('datetime').datetime.utcnow().isoformat(),
            "ingestion_validation": self.validate_ingestion(),
            "data_integrity_validation": self.validate_data_integrity(),
            "metadata_validation": self.validate_metadata_completeness(),
            "overall_validation_passed": False
        }

        # Overall validation passes if all individual validations pass
        all_passed = (
            validation_results["ingestion_validation"]["validation_passed"] and
            validation_results["data_integrity_validation"]["validation_passed"] and
            validation_results["metadata_validation"]["validation_passed"]
        )

        validation_results["overall_validation_passed"] = all_passed

        logger.info(f"Complete validation finished. Overall result: {'PASSED' if all_passed else 'FAILED'}")
        return validation_results