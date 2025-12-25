"""
Query Processor service for the semantic retrieval pipeline
"""
from typing import List
from ..models.query import Query
from ..embedder.cohere_embedder import CohereEmbedder
from src.rag_pipeline.base_models import EmbeddingVector


class QueryProcessor:
    """
    Handles incoming text queries and converts them to embedding vectors
    using the same model as the ingestion pipeline
    """

    def __init__(self):
        """
        Initialize Query Processor with Cohere embedder
        """
        self.embedder = CohereEmbedder()

    def process_query(self, query_text: str) -> Query:
        """
        Convert text query to embedding vector using the same model as ingestion

        Args:
            query_text: The text query to process

        Returns:
            Query object with text and embedding vector
        """
        # Generate embedding for the query text
        embedding_vector: EmbeddingVector = self.embedder.generate_embedding(query_text)

        # Create and return Query object
        query = Query(
            text=query_text,
            vector=embedding_vector.vector
        )

        return query

    def process_query_batch(self, query_texts: List[str]) -> List[Query]:
        """
        Process multiple query texts in batch

        Args:
            query_texts: List of query texts to process

        Returns:
            List of Query objects with text and embedding vectors
        """
        # Generate embeddings for all query texts
        embedding_vectors = self.embedder.generate_embeddings_batch(query_texts)

        # Create Query objects for each text-vector pair
        queries = []
        for text, embedding in zip(query_texts, embedding_vectors):
            query = Query(
                text=text,
                vector=embedding.vector
            )
            queries.append(query)

        return queries

    def get_embedding_model_info(self) -> dict:
        """
        Get information about the embedding model being used

        Returns:
            Dictionary with model information
        """
        return self.embedder.get_model_info()