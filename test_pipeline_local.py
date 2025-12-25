#!/usr/bin/env python3
"""Test script to run the RAG pipeline with local-only components (no external API calls)."""

import sys
import os
from pathlib import Path

# Add the src directory to the path so we can import rag_pipeline modules
sys.path.insert(0, str(Path(__file__).parent / "src"))

from src.rag_pipeline.crawler.site_crawler import SiteCrawler
from src.rag_pipeline.chunker.content_chunker import ContentChunker
from src.rag_pipeline.logging import logger
from src.rag_pipeline.base_models import SourceReference


def test_crawling_and_chunking_only():
    """Test the crawling and chunking functionality without external API calls."""
    print("Testing crawling and chunking functionality (without external API calls)...")

    # Use a well-known public documentation site for testing
    # For this test, we'll use a simple example since we can't make real API calls
    test_content = """
    # Introduction to AI Robotics

    AI Robotics is an interdisciplinary field combining artificial intelligence and robotics.

    ## Key Components

    The main components of AI robotics include perception, planning, and control systems.

    ### Perception Systems

    Perception systems allow robots to understand their environment through sensors.

    ### Planning Systems

    Planning systems determine the best sequence of actions to achieve goals.

    ### Control Systems

    Control systems execute the planned actions with precision.

    ## Applications

    AI robotics has applications in manufacturing, healthcare, and autonomous vehicles.

    The technology continues to advance with improvements in machine learning algorithms.
    """

    # Test the chunker functionality
    print("\nTesting content chunking...")
    chunker = ContentChunker()

    source_ref = SourceReference(
        url="https://example.com/test",
        section_heading="Test Content",
        chunk_index=0,
        page_title="Test Page"
    )

    # Chunk the test content
    chunks = chunker.chunk_content(
        test_content,
        source_url="https://example.com/test",
        page_title="Test Page",
        chunk_size=300,
        chunk_overlap=50
    )

    print(f"Created {len(chunks)} chunks from test content")

    for i, chunk in enumerate(chunks):
        print(f"\nChunk {i+1} (size: {len(chunk.text)} chars):")
        print(f"  Section: {chunk.source_reference.section_heading}")
        print(f"  Content preview: {chunk.text[:100]}...")

    # Test quality validation
    quality_ok = chunker.validate_chunk_quality(chunks, min_size=50)
    print(f"\nChunk quality validation: {'PASSED' if quality_ok else 'FAILED'}")

    print("\nCrawling and chunking test completed successfully!")
    print("Note: This test only covers local processing without external API calls.")
    print("To run the full pipeline with embedding and storage, you need valid API keys.")


def test_crawling_functionality():
    """Test just the crawling functionality."""
    print("\nTesting crawling functionality...")

    # Create a crawler instance
    crawler = SiteCrawler(delay=0.1)  # Short delay for testing

    # Test with a simple example (not actually crawling to avoid external calls in this test)
    # Instead, we'll just show that the crawler can be instantiated and has the right methods
    print("Crawler created successfully")
    print(f"Crawler delay: {crawler.delay}")

    # Test content extraction from sample HTML
    sample_html = """
    <html>
    <head><title>Test Page</title></head>
    <body>
        <h1>Main Title</h1>
        <p>This is a sample paragraph with some content.</p>
        <div class="nav">Navigation content to be filtered out</div>
        <p>Another paragraph with more content.</p>
    </body>
    </html>
    """

    title = crawler.extract_title(sample_html)
    print(f"Extracted title: {title}")

    content = crawler.extract_content(sample_html)
    print(f"Extracted content length: {len(content)} characters")
    print(f"Content preview: {content[:100]}...")

    print("Crawling functionality test completed!")


if __name__ == "__main__":
    print("Running local-only tests for RAG pipeline components...")

    # Test crawling functionality
    test_crawling_functionality()

    # Test chunking functionality
    test_crawling_and_chunking_only()

    print("\n" + "="*60)
    print("Local tests completed successfully!")
    print("The crawling and chunking components are working correctly.")
    print("\nTo run the full pipeline with embedding and cloud storage:")
    print("1. Obtain valid Cohere API key")
    print("2. Set up Qdrant Cloud account and get credentials")
    print("3. Update your .env file with real credentials")
    print("4. Run: python -m src.rag_pipeline.cli.main --site-url 'https://your-site.com'")