"""Main CLI entry point for the RAG Knowledge Ingestion and Retrieval Pipeline."""

import argparse
import sys
from pathlib import Path

# Add the src directory to the path so we can import rag_pipeline modules
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.rag_pipeline.config import config
from src.rag_pipeline.logging import logger
from src.rag_pipeline.crawler.site_crawler import SiteCrawler
from src.rag_pipeline.chunker.content_chunker import ContentChunker
from src.rag_pipeline.exceptions import RAGPipelineError

# Import storage and embedder based on mode
if not config.local_mode:
    from src.rag_pipeline.embedder.cohere_embedder import CohereEmbedder
    from src.rag_pipeline.storage.qdrant_client import QdrantStorage
else:
    # Mock classes for local mode
    class MockEmbedder:
        def generate_embedding(self, text):
            # Return a mock embedding (simple length-based vector)
            return type('MockEmbedding', (), {
                'vector': [len(text) % 1000 / 1000.0] * 10,  # Simple mock vector
                'model_name': 'mock-embedding-model'
            })()

        def generate_embeddings_batch(self, texts, batch_size=96):
            return [self.generate_embedding(text) for text in texts]

    class MockStorage:
        def __init__(self):
            from src.rag_pipeline.storage.local_storage import LocalStorage
            self.storage = LocalStorage()

        def store_chunks(self, chunks):
            return self.storage.store_chunks(chunks)

        def search_similar(self, query_embedding, top_k=5):
            return self.storage.search_similar(query_embedding, top_k)

    CohereEmbedder = MockEmbedder
    QdrantStorage = MockStorage


def create_pipeline():
    """Create and return all ingestion pipeline components."""
    # Initialize components
    crawler = SiteCrawler()
    chunker = ContentChunker()
    embedder = CohereEmbedder()
    storage = QdrantStorage()

    return crawler, chunker, embedder, storage


def create_retrieval_pipeline():
    """Create and return retrieval pipeline components."""
    from src.rag_pipeline.services.retrieval_service import RetrievalService
    from src.rag_pipeline.services.validation_framework import ValidationFramework

    retrieval_service = RetrievalService()
    validation_framework = ValidationFramework(retrieval_service)

    return retrieval_service, validation_framework


def run_ingestion_pipeline(
    site_url: str,
    chunk_size: int,
    chunk_overlap: int
) -> bool:
    """
    Run the complete RAG ingestion pipeline.

    Args:
        site_url: URL of the Docusaurus site to crawl
        chunk_size: Size of text chunks
        chunk_overlap: Overlap between chunks

    Returns:
        True if pipeline completed successfully, False otherwise
    """
    logger.info("Starting RAG Knowledge Ingestion Pipeline...")

    try:
        # Create pipeline components
        crawler, chunker, embedder, storage = create_pipeline()

        # Step 1: Crawl the site
        logger.info(f"Crawling site: {site_url}")
        raw_content = crawler.crawl_site(site_url)
        logger.info(f"Crawled {len(raw_content)} pages successfully")

        # Step 2: Extract and clean content
        logger.info("Extracting clean content from crawled pages...")
        clean_content = []
        for url, raw_html in raw_content.items():
            extracted_text = crawler.extract_content(raw_html)
            clean_content.append({
                'url': url,
                'title': crawler.extract_title(raw_html),
                'text': extracted_text
            })

        logger.info(f"Extracted content from {len(clean_content)} pages")

        # Step 3: Chunk the content
        logger.info(f"Chunking content (size: {chunk_size}, overlap: {chunk_overlap})...")
        all_chunks = []
        for page_data in clean_content:
            page_chunks = chunker.chunk_content(
                page_data['text'],
                page_data['url'],
                page_data['title'],
                chunk_size=chunk_size,
                chunk_overlap=chunk_overlap
            )
            all_chunks.extend(page_chunks)

        logger.info(f"Created {len(all_chunks)} content chunks")

        # Step 4: Generate embeddings
        logger.info("Generating embeddings for chunks...")
        # Extract just the text content for embedding generation
        texts_to_embed = [chunk.text for chunk in all_chunks]
        embeddings = embedder.generate_embeddings_batch(texts_to_embed)

        # Associate embeddings with chunks
        for chunk, embedding in zip(all_chunks, embeddings):
            chunk.embedding = embedding

        logger.info(f"Generated embeddings for {len(all_chunks)} chunks")

        # Step 5: Store in Qdrant
        logger.info("Storing embeddings in Qdrant Cloud...")
        success = storage.store_chunks(all_chunks)
        if success:
            logger.info("Successfully stored all chunks in Qdrant Cloud")

        logger.info("RAG Knowledge Ingestion Pipeline completed successfully!")
        return True

    except RAGPipelineError as e:
        logger.error(f"Pipeline failed due to RAG-specific error: {str(e)}")
        return False
    except Exception as e:
        logger.error(f"Pipeline failed due to unexpected error: {str(e)}")
        return False


def main():
    """Main CLI function."""
    parser = argparse.ArgumentParser(
        description="RAG Knowledge Ingestion and Retrieval Pipeline for Docusaurus Documentation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run ingestion pipeline
  python -m src.rag_pipeline.cli.main --site-url https://example.com

  # Retrieve semantically similar content
  python -m src.rag_pipeline.cli.main retrieve "What are AI ethics?"

  # Retrieve with filters and custom top-k
  python -m src.rag_pipeline.cli.main retrieve "machine learning" --top-k 10 --filter url=https://example.com/page

  # Validate retrieval accuracy
  python -m src.rag_pipeline.cli.main validate

  # Run validation with benchmarks
  python -m src.rag_pipeline.cli.main validate --benchmark
        """
    )

    # Add subparsers for different commands
    subparsers = parser.add_subparsers(dest='command', help='Available commands')

    # Ingestion command (default behavior)
    ingestion_parser = subparsers.add_parser(
        'ingest',
        help='Run the RAG ingestion pipeline (crawl, chunk, embed, store)'
    )
    ingestion_parser.add_argument(
        '--site-url',
        type=str,
        default=config.docusaurus_site_url,
        help='URL of the Docusaurus site to crawl (default from config)'
    )
    ingestion_parser.add_argument(
        '--chunk-size',
        type=int,
        default=config.chunk_size,
        help='Size of text chunks (default from config)'
    )
    ingestion_parser.add_argument(
        '--chunk-overlap',
        type=int,
        default=config.chunk_overlap,
        help='Overlap between chunks (default from config)'
    )
    ingestion_parser.add_argument(
        '--validate',
        action='store_true',
        help='Run validation after ingestion'
    )

    # Retrieve command
    retrieve_parser = subparsers.add_parser(
        'retrieve',
        help='Perform semantic retrieval against Qdrant'
    )
    retrieve_parser.add_argument(
        'query',
        help='Text query to search for'
    )
    retrieve_parser.add_argument(
        '--top-k',
        type=int,
        default=5,
        help='Number of results to retrieve (default: 5)'
    )
    retrieve_parser.add_argument(
        '--filter',
        action='append',
        metavar='KEY=VALUE',
        help='Metadata filter in format key=value (can be used multiple times)'
    )
    retrieve_parser.add_argument(
        '--format',
        choices=['json', 'text'],
        default='text',
        help='Output format (default: text)'
    )
    retrieve_parser.add_argument(
        '--verbose',
        action='store_true',
        help='Show detailed information'
    )

    # Validate command
    validate_parser = subparsers.add_parser(
        'validate',
        help='Run validation tests on the retrieval system'
    )
    validate_parser.add_argument(
        '--top-k',
        type=int,
        default=5,
        help='Number of results to retrieve for validation (default: 5)'
    )
    validate_parser.add_argument(
        '--format',
        choices=['json', 'text'],
        default='text',
        help='Output format (default: text)'
    )
    validate_parser.add_argument(
        '--benchmark',
        action='store_true',
        help='Run performance benchmarks in addition to accuracy validation'
    )
    validate_parser.add_argument(
        '--verbose',
        action='store_true',
        help='Show detailed information'
    )

    args = parser.parse_args()

    # If no command is provided, default to ingestion
    if args.command is None:
        # Run the ingestion pipeline with the original behavior
        success = run_ingestion_pipeline(
            site_url=args.site_url if hasattr(args, 'site_url') else config.docusaurus_site_url,
            chunk_size=args.chunk_size if hasattr(args, 'chunk_size') else config.chunk_size,
            chunk_overlap=args.chunk_overlap if hasattr(args, 'chunk_overlap') else config.chunk_overlap
        )
    elif args.command == 'ingest':
        # Run the ingestion pipeline
        success = run_ingestion_pipeline(
            site_url=args.site_url,
            chunk_size=args.chunk_size,
            chunk_overlap=args.chunk_overlap
        )
    elif args.command == 'retrieve':
        # Handle retrieve command
        success = handle_retrieve_command(args)
    elif args.command == 'validate':
        # Handle validate command
        success = handle_validate_command(args)
    else:
        parser.print_help()
        sys.exit(1)

    if success:
        logger.info("Pipeline completed successfully!")
        sys.exit(0)
    else:
        logger.error("Pipeline failed!")
        sys.exit(1)


def handle_retrieve_command(args):
    """Handle the retrieve command."""
    from src.rag_pipeline.services.retrieval_service import RetrievalService
    from src.rag_pipeline.cli.retrieve import parse_filter_args

    try:
        # Initialize retrieval service
        retrieval_service = RetrievalService()

        # Parse filters
        filters = {}
        if args.filter:
            filters = parse_filter_args(args.filter)

        # Validate query
        validation_errors = retrieval_service.validate_query(args.query)
        if validation_errors:
            print("Query validation errors:")
            for error in validation_errors:
                print(f"  - {error}")
            return False

        # Perform retrieval
        result = retrieval_service.retrieve(
            query_text=args.query,
            top_k=args.top_k,
            filters=filters
        )

        # Format and display results
        if args.format == 'json':
            import json
            output = {
                "query": result.query_text,
                "execution_time_ms": result.execution_time_ms,
                "result_count": len(result.results),
                "results": [
                    {
                        "id": r.id,
                        "content": r.content[:200] + "..." if len(r.content) > 200 else r.content,  # Truncate long content
                        "score": r.score,
                        "metadata": r.metadata
                    }
                    for r in result.results
                ]
            }
            print(json.dumps(output, indent=2))
        else:  # text format
            print(f"\nQuery: {result.query_text}")
            print(f"Execution time: {result.execution_time_ms:.2f}ms")
            print(f"Results found: {len(result.results)}")
            print("-" * 80)

            for i, r in enumerate(result.results, 1):
                print(f"\n{i}. Score: {r.score:.4f}")
                print(f"   Content: {r.content[:200] + '...' if len(r.content) > 200 else r.content}")
                print(f"   Metadata: {dict(list(r.metadata.items())[:3])}")  # Show first 3 metadata items

                if args.verbose and len(r.metadata) > 3:
                    remaining = len(r.metadata) - 3
                    print(f"   (and {remaining} more metadata fields)")

        return True

    except Exception as e:
        print(f"Error during retrieval: {str(e)}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return False


def handle_validate_command(args):
    """Handle the validate command."""
    try:
        # Initialize services
        retrieval_service, validation_framework = create_retrieval_pipeline()

        print("Starting validation tests...")

        # Run accuracy validation
        validation_result = validation_framework.validate_retrieval_accuracy(top_k=args.top_k)

        # Prepare output data
        import json
        output = {
            "accuracy_metrics": validation_result.accuracy_metrics,
            "performance_metrics": validation_result.performance_metrics,
            "summary": {
                "total_queries": validation_result.total_queries,
                "successful_queries": validation_result.successful_queries,
                "failed_queries": validation_result.failed_queries,
                "average_response_time": validation_result.average_response_time
            }
        }

        # Run benchmarks if requested
        if args.benchmark:
            print("Running performance benchmarks...")
            benchmark_results = validation_framework.run_performance_benchmarks()
            output["benchmark_results"] = benchmark_results

        # Format and display results
        if args.format == 'json':
            print(json.dumps(output, indent=2))
        else:  # text format
            print("\n" + "="*60)
            print("VALIDATION RESULTS")
            print("="*60)

            print(f"\nAccuracy Metrics:")
            for key, value in validation_result.accuracy_metrics.items():
                print(f"  {key}: {value:.4f}")

            print(f"\nPerformance Metrics:")
            for key, value in validation_result.performance_metrics.items():
                if isinstance(value, float):
                    print(f"  {key}: {value:.2f}")
                else:
                    print(f"  {key}: {value}")

            print(f"\nSummary:")
            print(f"  Total Queries: {validation_result.total_queries}")
            print(f"  Successful: {validation_result.successful_queries}")
            print(f"  Failed: {validation_result.failed_queries}")
            print(f"  Average Response Time: {validation_result.average_response_time:.2f}ms")

            if args.benchmark and "benchmark_results" in output:
                print(f"\nBenchmark Results:")
                for k, metrics in output["benchmark_results"].items():
                    print(f"  {k}:")
                    for metric, value in metrics.items():
                        if isinstance(value, float):
                            print(f"    {metric}: {value:.2f}")
                        else:
                            print(f"    {metric}: {value}")

            # Success/failure indicator
            success_rate = validation_result.accuracy_metrics.get("success_rate", 0)
            print(f"\nOverall Success Rate: {success_rate:.2%}")
            if success_rate >= 0.8:
                print("✅ VALIDATION PASSED")
            else:
                print("❌ VALIDATION FAILED")

        return True

    except Exception as e:
        print(f"Error during validation: {str(e)}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return False


if __name__ == "__main__":
    main()