#!/usr/bin/env python3
"""
Test script for the semantic retrieval pipeline
"""
import os
import sys
from pathlib import Path

# Add the src directory to the path
sys.path.insert(0, str(Path(__file__).parent))

def test_retrieval():
    """Test the retrieval functionality"""
    try:
        from src.rag_pipeline.services.retrieval_service import RetrievalService

        # Initialize the service
        print("Initializing retrieval service...")
        retrieval_service = RetrievalService()

        # Test a simple query
        print("Testing retrieval...")
        result = retrieval_service.retrieve(
            query_text="What are the main principles of AI robotics?",
            top_k=3
        )

        print(f"Query: {result.query_text}")
        print(f"Execution time: {result.execution_time_ms:.2f}ms")
        print(f"Results found: {len(result.results)}")

        for i, r in enumerate(result.results, 1):
            print(f"\n{i}. Score: {r.score:.4f}")
            print(f"   Content: {r.content[:100]}...")
            print(f"   Metadata: {dict(list(r.metadata.items())[:3])}")

        print("\n✅ Retrieval test passed!")
        return True

    except Exception as e:
        print(f"❌ Retrieval test failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def test_validation():
    """Test the validation functionality"""
    try:
        from src.rag_pipeline.services.retrieval_service import RetrievalService
        from src.rag_pipeline.services.validation_framework import ValidationFramework

        print("\nInitializing validation framework...")
        retrieval_service = RetrievalService()
        validation_framework = ValidationFramework(retrieval_service)

        print("Running validation tests...")
        result = validation_framework.validate_retrieval_accuracy(top_k=3)

        print(f"Total queries: {result.total_queries}")
        print(f"Successful queries: {result.successful_queries}")
        print(f"Accuracy: {result.accuracy_metrics['success_rate']:.2%}")
        print(f"Average response time: {result.average_response_time:.2f}ms")

        print("\n✅ Validation test passed!")
        return True

    except Exception as e:
        print(f"❌ Validation test failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    print("Testing Semantic Retrieval Pipeline...")

    # Check if required environment variables are set
    required_vars = ['QDRANT_URL', 'QDRANT_API_KEY', 'COHERE_API_KEY']
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        print(f"⚠️  Warning: Missing environment variables: {missing_vars}")
        print("   Please set these variables to test against real Qdrant instance.")
        print("   For testing without a real instance, you can run in local mode.")

    # Run tests
    retrieval_ok = test_retrieval()
    validation_ok = test_validation()

    if retrieval_ok and validation_ok:
        print("\n🎉 All tests passed!")
        sys.exit(0)
    else:
        print("\n💥 Some tests failed!")
        sys.exit(1)