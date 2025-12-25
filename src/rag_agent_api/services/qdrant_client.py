"""
Qdrant Client Interface for the RAG Agent API
"""
import os
import time
from typing import List, Dict, Optional, Any
from qdrant_client import QdrantClient
from qdrant_client.http.models import (
    PointStruct,
    VectorParams,
    Distance,
    Filter,
    FieldCondition,
    MatchValue,
    HasIdCondition
)
from pydantic import BaseModel
from ..config import config


class QdrantConfig(BaseModel):
    """Configuration for Qdrant connection"""
    url: str
    api_key: str
    collection_name: str
    vector_size: int = 1024  # Default size for Cohere embeddings
    distance: str = "Cosine"


class QdrantClientInterface:
    """
    Manages connections to Qdrant Cloud and executes vector similarity searches
    """

    def __init__(self):
        """
        Initialize Qdrant client with configuration from global config
        """
        self.qdrant_config = QdrantConfig(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            collection_name=config.qdrant_collection_name
        )

        self.client = QdrantClient(
            url=self.qdrant_config.url,
            api_key=self.qdrant_config.api_key,
            # Use HTTPS for secure connection
            https=True
        )
        self.collection_name = self.qdrant_config.collection_name

    def search(
        self,
        query_vector: List[float],
        top_k: int = 5,
        filters: Optional[Dict[str, str]] = None
    ) -> List[Dict[str, Any]]:
        """
        Execute vector similarity search against Qdrant collection

        Args:
            query_vector: The embedding vector to search for
            top_k: Number of results to return
            filters: Optional metadata filters to apply

        Returns:
            List of result dictionaries with content, metadata, and scores
        """
        # Build filter conditions if provided
        qdrant_filter = None
        if filters:
            qdrant_filter = self._build_filter(filters)

        # Execute search
        start_time = time.time()
        search_response = self.client.query_points(
            collection_name=self.collection_name,
            query=query_vector,
            query_filter=qdrant_filter,
            limit=top_k,
            with_payload=True,
            with_vectors=False  # Don't return vectors to save bandwidth
        )
        execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

        # Convert results to dictionaries
        results = []
        # In the newer Qdrant client, query_points returns a Response object
        # The actual results are in search_response.points
        search_results = search_response.points if hasattr(search_response, 'points') else search_response

        for result in search_results:
            # Extract content and metadata from payload
            # The ingestion pipeline stores content as 'text', but this expects 'content'
            payload = result.payload if hasattr(result, 'payload') else result
            content = payload.get('content', payload.get('text', ''))
            metadata = {}
            for k, v in payload.items():
                if k not in ['content', 'text']:
                    # Convert all values to strings to match expected format
                    metadata[k] = str(v) if not isinstance(v, str) else v

            result_dict = {
                'id': str(result.id if hasattr(result, 'id') else ''),
                'content': content,
                'metadata': metadata,
                'score': result.score if hasattr(result, 'score') else 0.0
            }
            results.append(result_dict)

        return {
            'results': results,
            'query_vector': query_vector,
            'execution_time_ms': execution_time,
            'retrieval_count': len(results)
        }

    def _build_filter(self, filters: Dict[str, str]) -> Filter:
        """
        Build Qdrant filter from metadata filters
        """
        conditions = []

        for key, value in filters.items():
            condition = FieldCondition(
                key=key,
                match=MatchValue(value=value)
            )
            conditions.append(condition)

        return Filter(must=conditions) if conditions else Filter()

    def verify_connection(self) -> bool:
        """
        Verify that we can connect to Qdrant and access the collection
        """
        try:
            # Try to get collection info
            collection_info = self.client.get_collection(self.collection_name)
            return True
        except Exception as e:
            print(f"Failed to connect to Qdrant collection: {e}")
            return False

    def get_total_points(self) -> int:
        """
        Get the total number of points in the collection
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return collection_info.points_count
        except Exception as e:
            print(f"Failed to get collection info: {e}")
            return 0