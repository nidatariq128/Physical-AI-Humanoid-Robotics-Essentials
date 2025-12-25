"""
Metadata Filter model for the semantic retrieval pipeline
"""
from typing import Dict
from pydantic import BaseModel


class MetadataFilter(BaseModel):
    """
    Parameters that constrain retrieval results based on document properties (URL, section, etc.)
    """
    filters: Dict[str, str]  # Key-value pairs for metadata filtering