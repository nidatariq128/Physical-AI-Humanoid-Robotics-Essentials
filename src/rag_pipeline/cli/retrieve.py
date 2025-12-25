"""
CLI command for semantic retrieval
"""
import argparse
import json
from typing import Dict, Any
from ..services.retrieval_service import RetrievalService
from ..config import config


def create_retrieve_parser(subparsers):
    """
    Create argument parser for retrieve command
    """
    parser = subparsers.add_parser(
        'retrieve',
        help='Perform semantic retrieval against Qdrant'
    )

    parser.add_argument(
        'query',
        help='Text query to search for'
    )

    parser.add_argument(
        '--top-k',
        type=int,
        default=config.default_top_k,
        help=f'Number of results to retrieve (default: {config.default_top_k})'
    )

    parser.add_argument(
        '--filter',
        action='append',
        metavar='KEY=VALUE',
        help='Metadata filter in format key=value (can be used multiple times)'
    )

    parser.add_argument(
        '--format',
        choices=['json', 'text'],
        default='text',
        help='Output format (default: text)'
    )

    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Show detailed information'
    )

    return parser


def parse_filter_args(filter_args):
    """
    Parse filter arguments from command line

    Args:
        filter_args: List of filter strings in format "key=value"

    Returns:
        Dictionary of filters
    """
    if not filter_args:
        return {}

    filters = {}
    for filter_str in filter_args:
        if '=' in filter_str:
            key, value = filter_str.split('=', 1)
            filters[key.strip()] = value.strip()
        else:
            print(f"Warning: Invalid filter format '{filter_str}', skipping...")

    return filters


def handle_retrieve(args):
    """
    Handle the retrieve command
    """
    try:
        # Initialize retrieval service
        retrieval_service = RetrievalService()

        # Parse filters
        filters = parse_filter_args(args.filter)

        # Validate query
        validation_errors = retrieval_service.validate_query(args.query)
        if validation_errors:
            print("Query validation errors:")
            for error in validation_errors:
                print(f"  - {error}")
            return 1

        # Perform retrieval
        result = retrieval_service.retrieve(
            query_text=args.query,
            top_k=args.top_k,
            filters=filters
        )

        # Format and display results
        if args.format == 'json':
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

        return 0

    except Exception as e:
        print(f"Error during retrieval: {str(e)}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


def main():
    """
    Main function for retrieve CLI command
    """
    parser = argparse.ArgumentParser(description='Semantic Retrieval CLI')
    retrieve_parser = create_retrieve_parser(parser)

    args = parser.parse_args()
    return handle_retrieve(args)


if __name__ == "__main__":
    exit(main())