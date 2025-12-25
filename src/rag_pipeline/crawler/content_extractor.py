"""Module for extracting clean markdown/text content from crawled pages."""

import re
from typing import Dict, List
from bs4 import BeautifulSoup
from markdown import markdown
from src.rag_pipeline.base_models import DocumentChunk, SourceReference
from src.rag_pipeline.logging import logger
from src.rag_pipeline.exceptions import ContentExtractionError


class ContentExtractor:
    """Class to handle extraction of clean content from crawled HTML."""

    def __init__(self):
        """Initialize the content extractor."""
        pass

    def extract_markdown_from_html(self, html: str) -> str:
        """
        Convert HTML to clean markdown.

        Args:
            html: HTML content to convert

        Returns:
            Clean markdown content
        """
        try:
            # Use BeautifulSoup to clean the HTML first
            soup = BeautifulSoup(html, 'html.parser')

            # Remove script and style elements
            for script in soup(["script", "style"]):
                script.decompose()

            # Get text and handle common HTML entities
            text = str(soup)

            # Convert HTML to markdown
            # Note: This is a basic conversion - for more sophisticated conversion
            # you might want to use a library like 'html2text' or 'pandoc'
            markdown_content = markdown(text)

            # Clean up the markdown
            markdown_content = self._clean_markdown(markdown_content)

            return markdown_content

        except Exception as e:
            logger.error(f"Failed to convert HTML to markdown: {str(e)}")
            raise ContentExtractionError(f"Failed to convert HTML to markdown: {str(e)}")

    def _clean_markdown(self, markdown_text: str) -> str:
        """
        Clean up markdown content by removing extra whitespace and invalid patterns.

        Args:
            markdown_text: Raw markdown content to clean

        Returns:
            Cleaned markdown content
        """
        # Remove extra whitespace and newlines
        lines = markdown_text.split('\n')
        cleaned_lines = []

        for line in lines:
            line = line.strip()
            if line:  # Only add non-empty lines
                cleaned_lines.append(line)

        # Join with single newlines
        cleaned_content = '\n'.join(cleaned_lines)

        # Remove multiple consecutive newlines (more than 2)
        cleaned_content = re.sub(r'\n{3,}', '\n\n', cleaned_content)

        # Clean up common artifacts
        cleaned_content = re.sub(r'\s+', ' ', cleaned_content)  # Multiple spaces to single
        cleaned_content = re.sub(r'\[ \]', '', cleaned_content)  # Remove spurious [ ]

        return cleaned_content.strip()

    def extract_text_from_html(self, html: str) -> str:
        """
        Extract clean text content from HTML.

        Args:
            html: HTML content to extract text from

        Returns:
            Clean text content
        """
        try:
            soup = BeautifulSoup(html, 'html.parser')

            # Remove script and style elements
            for script in soup(["script", "style"]):
                script.decompose()

            # Get text content
            text = soup.get_text(separator=' ', strip=True)

            # Clean up the text
            text = self._clean_text(text)

            return text

        except Exception as e:
            logger.error(f"Failed to extract text from HTML: {str(e)}")
            raise ContentExtractionError(f"Failed to extract text from HTML: {str(e)}")

    def _clean_text(self, text: str) -> str:
        """
        Clean up text content by removing extra whitespace and invalid patterns.

        Args:
            text: Raw text content to clean

        Returns:
            Cleaned text content
        """
        # Replace multiple whitespace characters with a single space
        text = re.sub(r'\s+', ' ', text)

        # Remove extra newlines (more than 2 consecutive)
        text = re.sub(r'\n{3,}', '\n\n', text)

        # Remove leading/trailing whitespace
        text = text.strip()

        return text

    def extract_sections(self, content: str) -> List[Dict[str, str]]:
        """
        Extract sections from content based on headers.

        Args:
            content: Content to extract sections from

        Returns:
            List of dictionaries with 'title' and 'content' keys
        """
        sections = []
        lines = content.split('\n')
        current_section = {'title': 'Introduction', 'content': ''}

        for line in lines:
            # Check if line is a header (starts with #)
            if line.strip().startswith('#'):
                # Save the previous section if it has content
                if current_section['content'].strip():
                    sections.append(current_section.copy())

                # Start new section
                header_level = len(line) - len(line.lstrip('#'))
                header_text = line.strip()[header_level:].strip()
                current_section = {'title': header_text, 'content': ''}
            else:
                current_section['content'] += line + '\n'

        # Add the last section
        if current_section['content'].strip():
            sections.append(current_section)

        return sections

    def extract_metadata_from_html(self, html: str) -> Dict[str, str]:
        """
        Extract metadata (title, description, etc.) from HTML.

        Args:
            html: HTML content to extract metadata from

        Returns:
            Dictionary of metadata
        """
        try:
            soup = BeautifulSoup(html, 'html.parser')
            metadata = {}

            # Extract title
            title_tag = soup.find('title')
            if title_tag:
                metadata['title'] = title_tag.get_text().strip()

            # Extract meta description
            desc_tag = soup.find('meta', attrs={'name': 'description'})
            if desc_tag:
                metadata['description'] = desc_tag.get('content', '').strip()

            # Extract meta keywords
            keywords_tag = soup.find('meta', attrs={'name': 'keywords'})
            if keywords_tag:
                metadata['keywords'] = keywords_tag.get('content', '').strip()

            # Extract any Open Graph tags
            og_title = soup.find('meta', property='og:title')
            if og_title:
                metadata['og_title'] = og_title.get('content', '').strip()

            og_description = soup.find('meta', property='og:description')
            if og_description:
                metadata['og_description'] = og_description.get('content', '').strip()

            return metadata

        except Exception as e:
            logger.warning(f"Failed to extract metadata: {str(e)}")
            return {}