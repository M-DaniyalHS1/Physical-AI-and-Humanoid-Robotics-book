"""
Qdrant Vector Database Client

This module provides a client for interacting with Qdrant vector database
for RAG (Retrieval Augmented Generation) functionality.
"""
from typing import List, Dict, Optional, Any
from uuid import uuid4
import logging

# Try to import qdrant_client, but provide fallback for testing
try:
    from qdrant_client import QdrantClient
    from qdrant_client.http import models
    from qdrant_client.http.models import Distance, VectorParams
    QDRANT_AVAILABLE = True
except ImportError:
    QDRANT_AVAILABLE = False
    # Define placeholder classes for testing when qdrant is not available
    class QdrantClient:
        pass
    class Distance:
        COSINE = "cosine"
    class VectorParams:
        pass
    models = None

from ..core.config import settings

logger = logging.getLogger(__name__)

class QdrantClientSingleton:
    """
    Singleton class to manage Qdrant client connection
    """
    _instance = None
    _client = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialize_client()
        return cls._instance

    def _initialize_client(self):
        """
        Initialize the Qdrant client based on configuration
        """
        if not QDRANT_AVAILABLE:
            logger.warning("Qdrant not available, creating mock client")
            self._client = None
            return

        if not settings.qdrant_url:
            raise ValueError("QDRANT_URL must be set in environment variables")

        if settings.qdrant_api_key:
            self._client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                timeout=10
            )
        else:
            # For local instances without API key
            self._client = QdrantClient(
                url=settings.qdrant_url,
                timeout=10
            )

    @property
    def client(self) -> QdrantClient:
        """
        Get the Qdrant client instance
        """
        return self._client


def get_qdrant_client() -> QdrantClient:
    """
    Get the Qdrant client instance
    """
    return QdrantClientSingleton().client