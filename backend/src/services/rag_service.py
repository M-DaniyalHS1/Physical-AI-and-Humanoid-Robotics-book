"""
RAG Service for AI Textbook Platform

This module provides RAG (Retrieval Augmented Generation) functionality
using Qdrant vector database to store and retrieve textbook content.
"""
from typing import List, Dict, Optional, Tuple
from uuid import uuid4
import logging

# Try to import qdrant_client, but provide fallback for testing
try:
    from qdrant_client.http import models
    from qdrant_client import QdrantClient
    from qdrant_client.http.models import PointStruct, Distance, VectorParams
    import numpy as np
    QDRANT_AVAILABLE = True
except ImportError:
    QDRANT_AVAILABLE = False
    # Define placeholder classes for testing when qdrant is not available
    class PointStruct:
        pass
    class Distance:
        COSINE = "cosine"
    class VectorParams:
        pass
    models = None

from ..core.config import settings
from .qdrant_client import get_qdrant_client

logger = logging.getLogger(__name__)

# Collection name for textbook content vectors
TEXTBOOK_CONTENT_COLLECTION = "textbook_content_vectors"

class RAGService:
    """
    Service class for RAG (Retrieval Augmented Generation) functionality
    using Qdrant vector database
    """

    def __init__(self):
        if QDRANT_AVAILABLE:
            self.client = get_qdrant_client()
        else:
            self.client = None
        self.collection_name = TEXTBOOK_CONTENT_COLLECTION

    def ensure_collection_exists(self, vector_size: int = 1536) -> bool:
        """
        Ensure that the textbook content collection exists in Qdrant
        Default vector size is 1536 (matching OpenAI embeddings)
        """
        if not QDRANT_AVAILABLE:
            logger.warning("Qdrant not available, skipping collection creation")
            return False

        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [collection.name for collection in collections.collections]

            if self.collection_name not in collection_names:
                # Create collection with specified vector size
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE),
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
                return True
            else:
                logger.info(f"Qdrant collection {self.collection_name} already exists")
                return False
        except Exception as e:
            logger.error(f"Error ensuring collection exists: {e}")
            raise

    def add_content_to_vector_store(
        self,
        content_id: str,
        content_text: str,
        vector: List[float],
        metadata: Optional[Dict[str, any]] = None
    ) -> bool:
        """
        Add textbook content to the vector store

        Args:
            content_id: Unique ID for the content (matches the one in PostgreSQL)
            content_text: The actual text content
            vector: The embedding vector for the content
            metadata: Additional metadata to store with the content

        Returns:
            bool: True if successful, False otherwise
        """
        if not QDRANT_AVAILABLE:
            logger.warning("Qdrant not available, skipping content addition")
            return False

        try:
            # Ensure collection exists before adding content
            self.ensure_collection_exists(len(vector))

            # Prepare metadata
            if metadata is None:
                metadata = {}

            # Add content to vector store
            points = [
                PointStruct(
                    id=content_id,
                    vector=vector,
                    payload={
                        "content_id": content_id,
                        "content_text": content_text,
                        "metadata": metadata
                    }
                )
            ]

            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logger.info(f"Added content {content_id} to vector store")
            return True

        except Exception as e:
            logger.error(f"Error adding content to vector store: {e}")
            return False

    def search_similar_content(
        self,
        query_vector: List[float],
        limit: int = 5,
        filters: Optional[Dict] = None
    ) -> List[Dict]:
        """
        Search for similar content in the vector store based on a query vector

        Args:
            query_vector: The embedding vector to search for similar content
            limit: Maximum number of results to return
            filters: Optional filters to apply to the search

        Returns:
            List of similar content with scores
        """
        if not QDRANT_AVAILABLE:
            logger.warning("Qdrant not available, returning empty results")
            # Return mock results for testing
            return []

        try:
            # Prepare filters if provided
            search_filter = None
            if filters and models:
                conditions = []
                for key, value in filters.items():
                    conditions.append(
                        models.FieldCondition(
                            key=f"metadata.{key}",
                            match=models.MatchValue(value=value)
                        )
                    )

                if conditions:
                    search_filter = models.Filter(must=conditions)

            # Perform search
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit,
                query_filter=search_filter
            )

            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    "id": result.id,
                    "content_text": result.payload.get("content_text", ""),
                    "metadata": result.payload.get("metadata", {}),
                    "score": result.score
                })

            logger.info(f"Found {len(formatted_results)} similar content items")
            return formatted_results

        except Exception as e:
            logger.error(f"Error searching for similar content: {e}")
            return []

    def search_content_by_text_similarity(
        self,
        query_text: str,
        query_vector: List[float],
        limit: int = 5,
        filters: Optional[Dict] = None
    ) -> List[Dict]:
        """
        Search for content similar to the query text using its vector representation

        Args:
            query_text: The query text (for potential use in metadata)
            query_vector: The embedding vector of the query text
            limit: Maximum number of results to return
            filters: Optional filters to apply to the search

        Returns:
            List of similar content with scores
        """
        return self.search_similar_content(query_vector, limit, filters)

    def get_content_by_id(self, content_id: str) -> Optional[Dict]:
        """
        Retrieve specific content by its ID

        Args:
            content_id: The ID of the content to retrieve

        Returns:
            Content data if found, None otherwise
        """
        if not QDRANT_AVAILABLE:
            logger.warning("Qdrant not available, returning None")
            return None

        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[content_id]
            )

            if records:
                record = records[0]
                return {
                    "id": record.id,
                    "content_text": record.payload.get("content_text", ""),
                    "metadata": record.payload.get("metadata", {}),
                }

            return None
        except Exception as e:
            logger.error(f"Error retrieving content by ID: {e}")
            return None

    def delete_content_by_id(self, content_id: str) -> bool:
        """
        Delete content from the vector store by its ID

        Args:
            content_id: The ID of the content to delete

        Returns:
            bool: True if successful, False otherwise
        """
        if not QDRANT_AVAILABLE:
            logger.warning("Qdrant not available, returning False")
            return False

        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=[content_id]
            )
            logger.info(f"Deleted content {content_id} from vector store")
            return True
        except Exception as e:
            logger.error(f"Error deleting content from vector store: {e}")
            return False

    def update_content_vector(self,
                            content_id: str,
                            new_vector: List[float],
                            new_content_text: str = None,
                            new_metadata: Optional[Dict] = None) -> bool:
        """
        Update the vector for existing content

        Args:
            content_id: The ID of the content to update
            new_vector: The new embedding vector
            new_content_text: Optionally update the content text
            new_metadata: Optionally update the metadata

        Returns:
            bool: True if successful, False otherwise
        """
        if not QDRANT_AVAILABLE:
            logger.warning("Qdrant not available, returning False")
            return False

        try:
            # First, get existing record to update its payload
            existing_record = self.get_content_by_id(content_id)
            if not existing_record:
                logger.error(f"Content with ID {content_id} not found for update")
                return False

            # Prepare updated payload
            payload = existing_record.get("metadata", {})
            if new_content_text:
                payload["content_text"] = new_content_text
            if new_metadata:
                payload.update(new_metadata)

            # Update the record
            points = [
                PointStruct(
                    id=content_id,
                    vector=new_vector,
                    payload={
                        "content_id": content_id,
                        "content_text": new_content_text or existing_record.get("content_text", ""),
                        "metadata": payload
                    }
                )
            ]

            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logger.info(f"Updated vector for content {content_id}")
            return True
        except Exception as e:
            logger.error(f"Error updating content vector: {e}")
            return False


# Singleton instance of RAG Service
rag_service = RAGService()