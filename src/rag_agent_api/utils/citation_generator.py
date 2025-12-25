"""
Citation Generation Utilities for the RAG Agent API
Creates proper citations for source materials
"""
from typing import List, Dict, Any
from ..models.retrieved_context import SourceCitation


class CitationGenerator:
    """
    Generate proper citations for source materials used in responses
    """

    def __init__(self):
        pass

    def generate_citations(self, retrieved_results: List[Dict[str, Any]]) -> List[SourceCitation]:
        """
        Generate a list of SourceCitation objects from retrieved results.

        Args:
            retrieved_results: List of retrieved results from Qdrant

        Returns:
            List of SourceCitation objects
        """
        citations = []
        for result in retrieved_results:
            content = result.get('content', '')
            metadata = result.get('metadata', {})
            score = result.get('score', 0.0)
            result_id = result.get('id', 'unknown')

            citation = self.create_citation(content, metadata, score, result_id)
            citations.append(citation)

        return citations

    def create_citation(self, content: str, metadata: Dict[str, Any], score: float = 0.0, result_id: str = 'unknown') -> SourceCitation:
        """
        Create a single SourceCitation object.

        Args:
            content: The content that was cited
            metadata: Metadata about the source
            score: Relevance score
            result_id: ID of the result

        Returns:
            SourceCitation object
        """
        # Determine the source from metadata
        source = self._extract_source_from_metadata(metadata, result_id)

        # Create citation with appropriate content length
        citation_content = self._format_citation_content(content)

        return SourceCitation(
            source=source,
            content=citation_content,
            score=score,
            metadata=metadata
        )

    def _extract_source_from_metadata(self, metadata: Dict[str, Any], result_id: str) -> str:
        """
        Extract the source identifier from metadata.

        Args:
            metadata: Metadata dictionary from the result
            result_id: ID of the result

        Returns:
            Source identifier string
        """
        # Try different common metadata fields for source identification
        source_fields = ['url', 'source', 'document_url', 'page_url', 'location', 'path']

        for field in source_fields:
            if field in metadata and metadata[field]:
                return metadata[field]

        # If no specific source found, use the result ID
        return result_id

    def _format_citation_content(self, content: str) -> str:
        """
        Format the citation content to an appropriate length.

        Args:
            content: Raw content to format

        Returns:
            Formatted content string
        """
        # Limit the content length for citations to avoid overly long citations
        max_length = 500  # Maximum length for citation content
        if len(content) <= max_length:
            return content
        else:
            # Truncate and add ellipsis
            truncated = content[:max_length].rsplit(' ', 1)[0]  # Break at word boundary
            return truncated + "..."

    def format_citation_text(self, citation: SourceCitation) -> str:
        """
        Format a citation as human-readable text.

        Args:
            citation: SourceCitation object to format

        Returns:
            Formatted citation string
        """
        source = citation.source
        content_preview = citation.content[:100] + "..." if len(citation.content) > 100 else citation.content

        return f"Source: {source}, Content: {content_preview}"

    def validate_citation(self, citation: SourceCitation) -> bool:
        """
        Validate that a citation has the required information.

        Args:
            citation: SourceCitation object to validate

        Returns:
            True if citation is valid, False otherwise
        """
        return bool(citation.source and citation.content)