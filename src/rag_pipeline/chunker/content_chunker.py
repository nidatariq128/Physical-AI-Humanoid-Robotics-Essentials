"""Module for normalizing and chunking content using a fixed strategy."""

import re
from typing import List
from src.rag_pipeline.base_models import DocumentChunk, SourceReference
from src.rag_pipeline.logging import logger
from src.rag_pipeline.exceptions import ChunkingError


class ContentChunker:
    """Class to handle content chunking with a deterministic strategy."""

    def __init__(self):
        """Initialize the content chunker."""
        pass

    def chunk_content(
        self,
        content: str,
        source_url: str,
        page_title: str = "",
        chunk_size: int = 1000,
        chunk_overlap: int = 200,
        parent_section: str = ""
    ) -> List[DocumentChunk]:
        """
        Chunk content using a semantic-aware strategy.

        Args:
            content: Content to chunk
            source_url: URL where content originated
            page_title: Title of the source page
            chunk_size: Target size of each chunk in characters
            chunk_overlap: Number of characters to overlap between chunks
            parent_section: Parent section name for context

        Returns:
            List of DocumentChunk objects
        """
        if chunk_overlap >= chunk_size:
            raise ChunkingError("Chunk overlap must be less than chunk size")

        # Split content into sentences to maintain semantic coherence
        sentences = self._split_into_sentences(content)

        chunks = []
        current_chunk = ""
        current_length = 0
        chunk_index = 0

        for sentence in sentences:
            sentence_length = len(sentence)

            # If adding this sentence would exceed chunk size
            if current_length + sentence_length > chunk_size and current_chunk:
                # Finalize current chunk
                chunk_ref = SourceReference(
                    url=source_url,
                    section_heading=f"Chunk {chunk_index}",
                    chunk_index=chunk_index,
                    page_title=page_title,
                    parent_section=parent_section
                )

                chunk = DocumentChunk(
                    text=current_chunk.strip(),
                    source_reference=chunk_ref
                )
                chunks.append(chunk)

                # Start new chunk with overlap from current chunk
                if chunk_overlap > 0:
                    # Get the end portion of the current chunk for overlap
                    overlap_start = max(0, len(current_chunk) - chunk_overlap)
                    current_chunk = current_chunk[overlap_start:]
                    current_length = len(current_chunk)
                else:
                    current_chunk = ""
                    current_length = 0

                chunk_index += 1

            # Add sentence to current chunk
            current_chunk += " " + sentence if current_chunk else sentence
            current_length += sentence_length + (1 if current_chunk != sentence else 0)

        # Add the final chunk if it has content
        if current_chunk.strip():
            chunk_ref = SourceReference(
                url=source_url,
                section_heading=f"Chunk {chunk_index}",
                chunk_index=chunk_index,
                page_title=page_title,
                parent_section=parent_section
            )

            chunk = DocumentChunk(
                text=current_chunk.strip(),
                source_reference=chunk_ref
            )
            chunks.append(chunk)

        logger.info(f"Created {len(chunks)} chunks from content with size ~{chunk_size}")
        return chunks

    def _split_into_sentences(self, text: str) -> List[str]:
        """
        Split text into sentences while preserving abbreviations and decimal numbers.

        Args:
            text: Text to split into sentences

        Returns:
            List of sentences
        """
        # Handle common abbreviations to avoid false sentence breaks
        abbreviations = [
            'Dr.', 'Mr.', 'Mrs.', 'Ms.', 'Prof.', 'Sr.', 'Jr.',
            'vs.', 'etc.', 'i.e.', 'e.g.', 'U.S.', 'U.K.', 'U.N.',
            'Jan.', 'Feb.', 'Mar.', 'Apr.', 'Jun.', 'Jul.', 'Aug.',
            'Sep.', 'Oct.', 'Nov.', 'Dec.', 'Ph.D.', 'M.D.', 'B.A.'
        ]

        # Temporarily replace abbreviations to avoid splitting on them
        temp_text = text
        abbreviation_map = {}
        for i, abbr in enumerate(abbreviations):
            placeholder = f"__ABBR_{i}__"
            temp_text = temp_text.replace(abbr, placeholder)
            abbreviation_map[placeholder] = abbr

        # Split on sentence endings, but keep the punctuation
        sentence_pattern = r'([.!?]+["\']*)(\s+)(?=[A-Z])'
        parts = re.split(f'({sentence_pattern})', temp_text)

        # Reconstruct sentences
        sentences = []
        current_sentence = ""

        for part in parts:
            if re.match(sentence_pattern, part):
                # This is the punctuation and whitespace that ends a sentence
                current_sentence += part
                sentences.append(current_sentence.strip())
                current_sentence = ""
            else:
                current_sentence += part

        # Add any remaining content as a sentence if it's not empty
        if current_sentence.strip():
            sentences.append(current_sentence.strip())

        # Restore abbreviations
        for i, sentence in enumerate(sentences):
            for placeholder, abbr in abbreviation_map.items():
                sentence = sentence.replace(placeholder, abbr)
            sentences[i] = sentence

        # Filter out empty sentences
        sentences = [s for s in sentences if s.strip()]

        return sentences

    def chunk_by_headers(
        self,
        content: str,
        source_url: str,
        page_title: str = "",
        max_chunk_size: int = 2000
    ) -> List[DocumentChunk]:
        """
        Chunk content based on headers, creating semantic chunks.

        Args:
            content: Content to chunk
            source_url: URL where content originated
            page_title: Title of the source page
            max_chunk_size: Maximum size of each chunk in characters

        Returns:
            List of DocumentChunk objects
        """
        # Split content by markdown headers
        header_pattern = r'^(#{1,6})\s+(.+)$'
        lines = content.split('\n')

        chunks = []
        current_chunk = ""
        current_header = "Introduction"
        chunk_index = 0

        i = 0
        while i < len(lines):
            line = lines[i].strip()

            if re.match(header_pattern, line):
                # If we have content in the current chunk and it's getting large
                if current_chunk and len(current_chunk) > max_chunk_size:
                    chunk_ref = SourceReference(
                        url=source_url,
                        section_heading=current_header,
                        chunk_index=chunk_index,
                        page_title=page_title
                    )

                    chunk = DocumentChunk(
                        text=current_chunk.strip(),
                        source_reference=chunk_ref
                    )
                    chunks.append(chunk)

                    current_chunk = ""
                    chunk_index += 1

                # This is a new header
                match = re.match(header_pattern, line)
                if match:
                    current_header = match.group(2).strip()
                    current_chunk += f"\n{line}\n"  # Add header to chunk
            else:
                # Regular content line
                current_chunk += f"{line}\n"

            i += 1

        # Add the final chunk if it has content
        if current_chunk.strip():
            chunk_ref = SourceReference(
                url=source_url,
                section_heading=current_header,
                chunk_index=chunk_index,
                page_title=page_title
            )

            chunk = DocumentChunk(
                text=current_chunk.strip(),
                source_reference=chunk_ref
            )
            chunks.append(chunk)

        logger.info(f"Created {len(chunks)} header-based chunks from content")
        return chunks

    def validate_chunk_quality(
        self,
        chunks: List[DocumentChunk],
        min_size: int = 50,
        max_size_variation: float = 0.5
    ) -> bool:
        """
        Validate the quality of chunks based on size and other metrics.

        Args:
            chunks: List of DocumentChunk objects to validate
            min_size: Minimum acceptable chunk size
            max_size_variation: Maximum variation in chunk sizes (0.0-1.0)

        Returns:
            True if chunks meet quality criteria, False otherwise
        """
        if not chunks:
            logger.warning("No chunks to validate")
            return False

        sizes = [len(chunk.text) for chunk in chunks]
        avg_size = sum(sizes) / len(sizes)

        # Check if any chunks are too small
        small_chunks = [i for i, size in enumerate(sizes) if size < min_size]
        if small_chunks:
            logger.warning(f"Found {len(small_chunks)} chunks smaller than minimum size {min_size}")

        # Check size variation
        if avg_size > 0:
            size_variation = max(sizes) / avg_size if avg_size > 0 else 0
            if size_variation > (1 + max_size_variation):
                logger.warning(f"High variation in chunk sizes detected (max/avg: {size_variation:.2f})")

        # Return True if no major quality issues detected
        return len(small_chunks) == 0