"""
Filter Engine service for the semantic retrieval pipeline
"""
from typing import Dict, Any, Optional
from ..models.metadata_filter import MetadataFilter
from ..logging import logger


class FilterEngine:
    """
    Apply metadata-based filtering to constrain search results
    """

    def __init__(self):
        """
        Initialize Filter Engine
        """
        pass

    def validate_filters(self, filters: Dict[str, str]) -> bool:
        """
        Validate filter parameters

        Args:
            filters: Dictionary of filter key-value pairs

        Returns:
            True if filters are valid, False otherwise
        """
        if not filters:
            return True

        # Check that all filter keys and values are strings and not empty
        for key, value in filters.items():
            if not isinstance(key, str) or not isinstance(value, str):
                logger.error(f"Invalid filter type: key={type(key)}, value={type(value)}")
                return False
            if not key.strip() or not value.strip():
                logger.error(f"Empty filter key or value: key='{key}', value='{value}'")
                return False

        return True

    def build_qdrant_filter_conditions(self, filters: Dict[str, str]) -> Dict[str, Any]:
        """
        Build Qdrant filter conditions from metadata filters

        Args:
            filters: Dictionary of filter key-value pairs

        Returns:
            Dictionary representing Qdrant filter conditions
        """
        if not filters:
            return {}

        # Validate filters first
        if not self.validate_filters(filters):
            raise ValueError("Invalid filter parameters")

        # Build filter conditions structure compatible with Qdrant
        conditions = {"must": []}

        for key, value in filters.items():
            condition = {
                "key": key,
                "match": {
                    "value": value
                }
            }
            conditions["must"].append(condition)

        return conditions

    def apply_filters(self, results: list, filters: Dict[str, str]) -> list:
        """
        Apply filters to results (client-side filtering as fallback)
        Note: In production, filtering should primarily happen server-side via Qdrant

        Args:
            results: List of result objects to filter
            filters: Dictionary of filter key-value pairs

        Returns:
            Filtered list of results
        """
        if not filters or not results:
            return results

        # Validate filters
        if not self.validate_filters(filters):
            raise ValueError("Invalid filter parameters")

        filtered_results = []
        for result in results:
            # Assuming result has metadata attribute with filterable fields
            metadata = getattr(result, 'metadata', {})

            # Check if result matches all filter conditions
            matches_all = True
            for key, value in filters.items():
                if metadata.get(key) != value:
                    matches_all = False
                    break

            if matches_all:
                filtered_results.append(result)

        logger.info(f"Applied filters: {filters}, original_count={len(results)}, filtered_count={len(filtered_results)}")

        return filtered_results

    def validate_metadata_schema(self, metadata: Dict[str, Any], required_keys: Optional[list] = None) -> bool:
        """
        Validate that metadata conforms to expected schema

        Args:
            metadata: Metadata dictionary to validate
            required_keys: List of required keys (optional)

        Returns:
            True if metadata is valid, False otherwise
        """
        if not isinstance(metadata, dict):
            return False

        if required_keys:
            for key in required_keys:
                if key not in metadata:
                    return False

        # Additional validation can be added here based on specific requirements
        return True

    def create_filter_from_query_params(self, **kwargs) -> Optional[MetadataFilter]:
        """
        Create a MetadataFilter object from query parameters

        Args:
            **kwargs: Query parameters that can be converted to filters

        Returns:
            MetadataFilter object or None if no valid filters found
        """
        # Define which parameters should be treated as filters
        filter_params = {}
        valid_filter_keys = ['url', 'section', 'page_number', 'source_document', 'category', 'type']

        for key, value in kwargs.items():
            if key in valid_filter_keys and value is not None:
                if isinstance(value, (str, int, float)):
                    filter_params[key] = str(value)
                else:
                    logger.warning(f"Skipping non-string filter value for key '{key}': {type(value)}")

        if filter_params:
            return MetadataFilter(filters=filter_params)

        return None