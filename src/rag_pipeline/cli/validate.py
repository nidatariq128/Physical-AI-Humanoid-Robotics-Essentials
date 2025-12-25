"""
CLI command for validation testing
"""
import argparse
import json
from typing import Dict, Any
from ..services.retrieval_service import RetrievalService
from ..services.validation_framework import ValidationFramework
from ..config import config


def create_validate_parser(subparsers):
    """
    Create argument parser for validate command
    """
    parser = subparsers.add_parser(
        'validate',
        help='Run validation tests on the retrieval system'
    )

    parser.add_argument(
        '--top-k',
        type=int,
        default=5,
        help='Number of results to retrieve for validation (default: 5)'
    )

    parser.add_argument(
        '--format',
        choices=['json', 'text'],
        default='text',
        help='Output format (default: text)'
    )

    parser.add_argument(
        '--benchmark',
        action='store_true',
        help='Run performance benchmarks in addition to accuracy validation'
    )

    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Show detailed information'
    )

    return parser


def handle_validate(args):
    """
    Handle the validate command
    """
    try:
        # Initialize services
        retrieval_service = RetrievalService()
        validation_framework = ValidationFramework(retrieval_service)

        print("Starting validation tests...")

        # Run accuracy validation
        validation_result = validation_framework.validate_retrieval_accuracy(top_k=args.top_k)

        # Prepare output data
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

        return 0

    except Exception as e:
        print(f"Error during validation: {str(e)}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


def main():
    """
    Main function for validate CLI command
    """
    parser = argparse.ArgumentParser(description='Validation Testing CLI')
    validate_parser = create_validate_parser(parser)

    args = parser.parse_args()
    return handle_validate(args)


if __name__ == "__main__":
    exit(main())