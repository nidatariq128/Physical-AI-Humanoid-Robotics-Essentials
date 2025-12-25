#!/usr/bin/env python3
"""
Verification script for the complete RAG Knowledge Ingestion and Retrieval Pipeline
"""
import os
import sys
import argparse
from pathlib import Path

# Add the src directory to the path
sys.path.insert(0, str(Path(__file__).parent))

from src.rag_pipeline.config import config
from src.rag_pipeline.logging import logger


def verify_configuration():
    """Verify that the configuration is properly set up"""
    print("🔍 Verifying configuration...")

    try:
        config.validate()
        print("✅ Configuration validation passed")
        return True
    except ValueError as e:
        print(f"❌ Configuration validation failed: {e}")
        return False


def verify_qdrant_connection():
    """Verify connection to Qdrant"""
    print("🔗 Verifying Qdrant connection...")

    try:
        from src.rag_pipeline.services.qdrant_client import QdrantClientInterface, QdrantConfig

        qdrant_config = QdrantConfig(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            collection_name=config.qdrant_collection_name
        )
        client = QdrantClientInterface(qdrant_config)

        if client.verify_connection():
            print(f"✅ Qdrant connection successful to {config.qdrant_url}")
            print(f"   Collection: {config.qdrant_collection_name}")
            total_points = client.get_total_points()
            print(f"   Total points in collection: {total_points}")
            return True
        else:
            print(f"❌ Qdrant connection failed")
            return False

    except Exception as e:
        print(f"❌ Qdrant connection failed: {e}")
        return False


def verify_embedding_model():
    """Verify that the embedding model is accessible"""
    print("🧠 Verifying embedding model...")

    try:
        from src.rag_pipeline.embedder.cohere_embedder import CohereEmbedder

        embedder = CohereEmbedder()
        model_info = embedder.get_model_info()

        print(f"✅ Embedding model: {model_info['model_name']}")
        print(f"   Dimensions: {model_info['dimensions']}")
        return True

    except Exception as e:
        print(f"❌ Embedding model verification failed: {e}")
        return False


def verify_retrieval_service():
    """Verify that retrieval service can be initialized"""
    print("🔍 Verifying retrieval service...")

    try:
        from src.rag_pipeline.services.retrieval_service import RetrievalService

        service = RetrievalService()
        stats = service.get_collection_stats()

        print(f"✅ Retrieval service initialized successfully")
        print(f"   Total documents: {stats['total_documents']}")
        print(f"   Embedding model: {stats['embedding_model']}")
        print(f"   Vector dimensions: {stats['vector_dimensions']}")
        return True

    except Exception as e:
        print(f"❌ Retrieval service verification failed: {e}")
        return False


def run_sample_retrieval():
    """Run a sample retrieval to test end-to-end functionality"""
    print("🧪 Running sample retrieval...")

    try:
        from src.rag_pipeline.services.retrieval_service import RetrievalService

        service = RetrievalService()

        # Use a simple test query
        result = service.retrieve(
            query_text="artificial intelligence",
            top_k=2
        )

        print(f"✅ Sample retrieval successful")
        print(f"   Query: {result.query_text}")
        print(f"   Results: {len(result.results)}")
        print(f"   Execution time: {result.execution_time_ms:.2f}ms")

        if result.results:
            print(f"   Top result score: {result.results[0].score:.4f}")

        return True

    except Exception as e:
        print(f"❌ Sample retrieval failed: {e}")
        return False


def verify_validation_framework():
    """Verify that the validation framework can be initialized"""
    print("✅ Verifying validation framework...")

    try:
        from src.rag_pipeline.services.retrieval_service import RetrievalService
        from src.rag_pipeline.services.validation_framework import ValidationFramework

        retrieval_service = RetrievalService()
        validation_framework = ValidationFramework(retrieval_service)

        # Get basic stats without running full validation to avoid long execution
        test_queries = validation_framework._load_test_queries()
        print(f"✅ Validation framework initialized with {len(test_queries)} test queries")
        return True

    except Exception as e:
        print(f"❌ Validation framework verification failed: {e}")
        return False


def test_imports():
    """Test that all main modules can be imported."""
    print("Testing imports...")

    try:
        from rag_pipeline.config import config
        print("+ Config module imported successfully")

        from rag_pipeline.logging import logger
        print("+ Logging module imported successfully")

        from rag_pipeline.exceptions import RAGPipelineError, CrawlerError
        print("+ Exceptions module imported successfully")

        from rag_pipeline.base_models import DocumentChunk, SourceReference, EmbeddingVector
        print("+ Base models module imported successfully")

        from rag_pipeline.crawler.site_crawler import SiteCrawler
        print("+ Site crawler module imported successfully")

        from rag_pipeline.chunker.content_chunker import ContentChunker
        print("+ Content chunker module imported successfully")

        from rag_pipeline.embedder.cohere_embedder import CohereEmbedder
        print("+ Cohere embedder module imported successfully")

        from rag_pipeline.storage.qdrant_client import QdrantStorage
        print("+ Qdrant client module imported successfully")

        from rag_pipeline.validators.ingestion_validator import IngestionValidator
        print("+ Ingestion validator module imported successfully")

        # New modules for retrieval pipeline
        from rag_pipeline.models.content_chunk import ContentChunk, RetrievalResult
        print("+ Content chunk models imported successfully")

        from rag_pipeline.models.query import Query
        print("+ Query model imported successfully")

        from rag_pipeline.models.metadata_filter import MetadataFilter
        print("+ Metadata filter model imported successfully")

        from rag_pipeline.models.validation_result import ValidationResult
        print("+ Validation result model imported successfully")

        from rag_pipeline.services.query_processor import QueryProcessor
        print("+ Query processor service imported successfully")

        from rag_pipeline.services.result_formatter import ResultFormatter
        print("+ Result formatter service imported successfully")

        from rag_pipeline.services.filter_engine import FilterEngine
        print("+ Filter engine service imported successfully")

        from rag_pipeline.services.retrieval_service import RetrievalService
        print("+ Retrieval service imported successfully")

        from rag_pipeline.services.validation_framework import ValidationFramework
        print("+ Validation framework imported successfully")

        print("+ All modules imported successfully!")
        return True

    except ImportError as e:
        print(f"X Import error: {e}")
        return False
    except Exception as e:
        print(f"X Unexpected error during imports: {e}")
        return False


def main():
    """Main verification function"""
    parser = argparse.ArgumentParser(description="Verify RAG Pipeline")
    parser.add_argument('--skip-connection', action='store_true',
                       help='Skip Qdrant connection test (for offline verification)')
    parser.add_argument('--verbose', action='store_true',
                       help='Show detailed output')

    args = parser.parse_args()

    print("🚀 Starting RAG Pipeline Verification...")
    print(f"   Configuration file: {config.__class__.__name__}")
    print(f"   Qdrant URL: {config.qdrant_url}")
    print(f"   Collection: {config.qdrant_collection_name}")
    print()

    # Run verifications
    results = []

    results.append(("Import Modules", test_imports()))
    results.append(("Configuration", verify_configuration()))
    results.append(("Embedding Model", verify_embedding_model()))
    results.append(("Retrieval Service", verify_retrieval_service()))
    results.append(("Validation Framework", verify_validation_framework()))

    if not args.skip_connection:
        results.append(("Qdrant Connection", verify_qdrant_connection()))
        results.append(("Sample Retrieval", run_sample_retrieval()))
    else:
        print("⏭️  Skipping connection tests as requested")

    # Summary
    print("\n" + "="*50)
    print("📊 VERIFICATION SUMMARY")
    print("="*50)

    passed = 0
    total = len(results)

    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{test_name:<25} {status}")
        if result:
            passed += 1

    print("-" * 50)
    print(f"TOTAL: {passed}/{total} tests passed")

    if passed == total:
        print("🎉 All verifications passed! Pipeline is ready.")
        print("\nTo use the pipeline:")
        print("1. Ingest data: python run_local_pipeline.py ingest --site-url 'https://example.com'")
        print("2. Retrieve: python run_local_pipeline.py retrieve 'your query here'")
        print("3. Validate: python run_local_pipeline.py validate")
        return 0
    else:
        print(f"💥 {total - passed} test(s) failed. Please check the configuration.")
        return 1


if __name__ == "__main__":
    sys.exit(main())