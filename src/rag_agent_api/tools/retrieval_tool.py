"""
Retrieval Tool for the RAG Agent API
Callable by the agent to fetch relevant content from Qdrant
"""
import json
from typing import Dict, Any, List
from pydantic import BaseModel, Field

from ..services.qdrant_client import QdrantClientInterface
import sys
import os
# Add the src directory to the path to allow absolute imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from src.rag_pipeline.services.query_processor import QueryProcessor


class RetrievalToolArgs(BaseModel):
    """Arguments for the retrieval tool"""
    query: str = Field(..., description="The search query to find relevant content")
    top_k: int = Field(default=5, description="Number of results to retrieve")
    filters: Dict[str, str] = Field(default={}, description="Metadata filters to apply")


class RetrievalTool:
    """
    Callable function that accepts query text and returns content chunks with metadata
    """

    def __init__(self):
        self.qdrant_client = QdrantClientInterface()
        self.query_processor = QueryProcessor()

    def __call__(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute the retrieval tool with the provided arguments

        Args:
            args: Dictionary containing query, top_k, and optional filters

        Returns:
            Dictionary with results, query_vector, and execution_time_ms
        """
        # Validate arguments using Pydantic model
        retrieval_args = RetrievalToolArgs(**args)

        # Process the query to get embedding vector
        query_obj = self.query_processor.process_query(retrieval_args.query)
        query_vector = query_obj.vector

        # Perform the search in Qdrant
        results = self.qdrant_client.search(
            query_vector=query_vector,
            top_k=retrieval_args.top_k,
            filters=retrieval_args.filters
        )

        return {
            "results": results['results'],
            "query_vector": query_vector,
            "execution_time_ms": results['execution_time_ms']
        }

    def get_function_definition(self):
        """
        Return the function definition for tool calling
        """
        return {
            "type": "function",
            "function": {
                "name": "retrieve_content",
                "description": "Retrieve relevant content from the knowledge base based on a query",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "The search query to find relevant content"
                        },
                        "top_k": {
                            "type": "integer",
                            "description": "Number of results to retrieve (default: 5)",
                            "default": 5
                        },
                        "filters": {
                            "type": "object",
                            "additionalProperties": {
                                "type": "string"
                            },
                            "description": "Metadata filters to apply (e.g., {'url': 'specific-page.com'})"
                        }
                    },
                    "required": ["query"]
                }
            }
        }


# Global instance of the retrieval tool
retrieval_tool = RetrievalTool()