"""Logging infrastructure for the RAG Knowledge Ingestion Pipeline."""

import logging
import sys
from typing import Optional
from .config import config


def setup_logging(level: Optional[str] = None) -> logging.Logger:
    """
    Set up logging configuration for the application.

    Args:
        level: Logging level (e.g., 'DEBUG', 'INFO', 'WARNING', 'ERROR')

    Returns:
        Configured logger instance
    """
    # Use the level from config if not provided
    log_level = level or config.log_level

    # Convert string level to logging constant
    numeric_level = getattr(logging, log_level.upper(), logging.INFO)

    # Create a custom logger
    logger = logging.getLogger('rag_pipeline')
    logger.setLevel(numeric_level)

    # Clear any existing handlers to avoid duplicates
    logger.handlers.clear()

    # Create handlers
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(numeric_level)

    # Create formatters and add them to handlers
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    console_handler.setFormatter(formatter)

    # Add handlers to the logger
    logger.addHandler(console_handler)

    return logger


# Global logger instance
logger = setup_logging()