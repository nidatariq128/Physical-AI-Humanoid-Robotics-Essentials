"""Module for crawling Docusaurus sites and extracting content."""

import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
from typing import Dict, List, Set
import time
import re
from src.rag_pipeline.base_models import SourceReference
from src.rag_pipeline.config import config
from src.rag_pipeline.logging import logger
from src.rag_pipeline.exceptions import CrawlerError, ContentExtractionError


class SiteCrawler:
    """Class to handle crawling Docusaurus sites and extracting clean content."""

    def __init__(self, delay: float = 1.0):
        """
        Initialize the site crawler.

        Args:
            delay: Delay in seconds between requests to be respectful to the server
        """
        self.delay = delay
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'RAG-Knowledge-Ingestion-Pipeline/1.0'
        })

    def _is_valid_url(self, url: str, base_domain: str) -> bool:
        """
        Check if a URL is valid for crawling (same domain, HTML content).

        Args:
            url: URL to check
            base_domain: Base domain to restrict crawling to

        Returns:
            True if URL is valid for crawling, False otherwise
        """
        try:
            parsed = urlparse(url)
            # Only crawl URLs from the same domain
            if parsed.netloc != urlparse(base_domain).netloc:
                return False

            # Skip non-HTML content
            if any(url.lower().endswith(ext) for ext in ['.pdf', '.jpg', '.png', '.gif', '.zip', '.exe']):
                return False

            return True
        except Exception:
            return False

    def _get_page_content(self, url: str) -> str:
        """
        Fetch content of a single page.

        Args:
            url: URL of the page to fetch

        Returns:
            HTML content of the page
        """
        try:
            response = self.session.get(url, timeout=30)
            response.raise_for_status()

            # Respectful delay between requests
            time.sleep(self.delay)

            return response.text
        except requests.RequestException as e:
            logger.error(f"Failed to fetch {url}: {str(e)}")
            raise CrawlerError(f"Failed to fetch {url}: {str(e)}")

    def _extract_links(self, html: str, base_url: str) -> Set[str]:
        """
        Extract all valid links from HTML content.

        Args:
            html: HTML content to extract links from
            base_url: Base URL to resolve relative links

        Returns:
            Set of absolute URLs found in the HTML
        """
        soup = BeautifulSoup(html, 'html.parser')
        links = set()

        for link_tag in soup.find_all('a', href=True):
            href = link_tag['href']
            absolute_url = urljoin(base_url, href)

            # Normalize URL by removing fragments
            normalized_url = absolute_url.split('#')[0]

            if self._is_valid_url(normalized_url, base_url):
                links.add(normalized_url)

        return links

    def crawl_site(self, base_url: str, max_pages: int = 1000) -> Dict[str, str]:
        """
        Crawl a Docusaurus site starting from the base URL.

        Args:
            base_url: Starting URL for crawling
            max_pages: Maximum number of pages to crawl

        Returns:
            Dictionary mapping URLs to their HTML content
        """
        logger.info(f"Starting to crawl site: {base_url}")

        # Normalize base URL
        if not base_url.endswith('/'):
            base_url += '/'

        visited_urls: Set[str] = set()
        urls_to_visit: List[str] = [base_url]
        crawled_content: Dict[str, str] = {}

        while urls_to_visit and len(visited_urls) < max_pages:
            current_url = urls_to_visit.pop(0)

            # Skip if already visited
            if current_url in visited_urls:
                continue

            logger.info(f"Crawling: {current_url}")

            try:
                html_content = self._get_page_content(current_url)
                crawled_content[current_url] = html_content
                visited_urls.add(current_url)

                # Extract links from the current page
                new_links = self._extract_links(html_content, base_url)

                # Add new links to visit (if not already visited)
                for link in new_links:
                    if link not in visited_urls and link not in urls_to_visit:
                        urls_to_visit.append(link)

            except CrawlerError:
                # Continue with other URLs even if one fails
                continue
            except Exception as e:
                logger.error(f"Unexpected error crawling {current_url}: {str(e)}")
                continue

        logger.info(f"Crawling completed. Visited {len(visited_urls)} pages.")
        return crawled_content

    def extract_title(self, html: str) -> str:
        """
        Extract the title from HTML content.

        Args:
            html: HTML content to extract title from

        Returns:
            Page title or empty string if not found
        """
        try:
            soup = BeautifulSoup(html, 'html.parser')
            title_tag = soup.find('title')
            if title_tag:
                return title_tag.get_text().strip()
            return ""
        except Exception as e:
            logger.warning(f"Failed to extract title: {str(e)}")
            return ""

    def extract_content(self, html: str) -> str:
        """
        Extract clean text content from HTML, removing navigation, headers, etc.

        Args:
            html: HTML content to extract text from

        Returns:
            Clean text content
        """
        try:
            soup = BeautifulSoup(html, 'html.parser')

            # Remove navigation elements, headers, footers, and other non-content elements
            for element in soup(['nav', 'header', 'footer', 'script', 'style', 'noscript']):
                element.decompose()

            # Remove elements with common class names for navigation/sidebars
            selectors_to_remove = [
                '.nav', '.navbar', '.navigation', '.sidebar',
                '.menu', '.header', '.footer', '.breadcrumb',
                '.search', '.toc', '.table-of-contents'
            ]

            for selector in selectors_to_remove:
                for element in soup.select(selector):
                    element.decompose()

            # Try to find main content area (common selectors for documentation sites)
            main_content = None
            content_selectors = [
                'main',
                'article',
                '.main-content',
                '.content',
                '.doc-content',
                '.documentation-content',
                '[role="main"]',
                '.container',
                '.wrapper'
            ]

            for selector in content_selectors:
                main_content = soup.select_one(selector)
                if main_content:
                    break

            # If no main content area found, use the body
            if not main_content:
                main_content = soup.find('body')

            if main_content:
                # Get text content and clean it up
                text = main_content.get_text(separator=' ', strip=True)
                # Clean up multiple spaces and newlines
                text = re.sub(r'\s+', ' ', text)
                return text
            else:
                # Fallback to extracting all text if no main content found
                text = soup.get_text(separator=' ', strip=True)
                text = re.sub(r'\s+', ' ', text)
                return text

        except Exception as e:
            logger.error(f"Failed to extract content: {str(e)}")
            raise ContentExtractionError(f"Failed to extract content: {str(e)}")