#!/usr/bin/env python3
"""Main entry point for the RAG Knowledge Ingestion and Retrieval Pipeline.

This script provides a unified interface for:
1. Ingesting Docusaurus documentation into Qdrant
2. Retrieving semantically similar content from Qdrant
3. Validating the retrieval system
"""

import os
import sys
from pathlib import Path

# Set local mode environment variable before importing config if needed
if 'LOCAL_MODE' not in os.environ:
    os.environ['LOCAL_MODE'] = 'true'

# Add the src directory to the path so we can import rag_pipeline modules
sys.path.insert(0, str(Path(__file__).parent))

from src.rag_pipeline.cli.main import main


if __name__ == "__main__":
    # Run the main CLI
    sys.exit(main())