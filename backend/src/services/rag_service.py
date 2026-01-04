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
    from qdrant_client.http.models import PointStruct, Distance, VectorParams, Filter, FieldCondition, MatchValue, Range
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
    class Filter:
        pass
    class FieldCondition:
        pass
    class MatchValue:
        pass
    class Range:
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

    def batch_add_content_to_vector_store(
        self,
        content_list: List[Dict[str, any]]
    ) -> bool:
        """
        Add multiple textbook contents to the vector store in a batch operation

        Args:
            content_list: List of dictionaries containing content_id, content_text, vector, and metadata

        Returns:
            bool: True if successful, False otherwise
        """
        if not QDRANT_AVAILABLE:
            logger.warning("Qdrant not available, skipping batch content addition")
            return False

        try:
            if not content_list:
                logger.warning("Empty content list provided for batch addition")
                return True

            # Ensure collection exists before adding content
            self.ensure_collection_exists(len(content_list[0]["vector"]))

            # Prepare points for batch insertion
            points = []
            for content in content_list:
                content_id = content["content_id"]
                content_text = content["content_text"]
                vector = content["vector"]
                metadata = content.get("metadata", {})

                point = PointStruct(
                    id=content_id,
                    vector=vector,
                    payload={
                        "content_id": content_id,
                        "content_text": content_text,
                        "metadata": metadata
                    }
                )
                points.append(point)

            # Perform batch upsert
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logger.info(f"Added {len(content_list)} content items to vector store in batch")
            return True

        except Exception as e:
            logger.error(f"Error adding content to vector store in batch: {e}")
            return False

    def search_similar_content(
        self,
        query_vector: List[float],
        limit: int = 5,
        filters: Optional[Dict] = None,
        min_score: Optional[float] = None
    ) -> List[Dict]:
        """
        Search for similar content in the vector store based on a query vector

        Args:
            query_vector: The embedding vector to search for similar content
            limit: Maximum number of results to return
            filters: Optional filters to apply to the search
            min_score: Optional minimum score threshold for results

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
                    if isinstance(value, dict) and 'range' in value:
                        # Handle range filters
                        range_val = value['range']
                        range_condition = FieldCondition(
                            key=f"metadata.{key}",
                            range=Range(
                                gte=range_val.get('gte'),
                                lte=range_val.get('lte'),
                                gt=range_val.get('gt'),
                                lt=range_val.get('lt')
                            )
                        )
                        conditions.append(range_condition)
                    else:
                        # Handle exact match filters
                        conditions.append(
                            FieldCondition(
                                key=f"metadata.{key}",
                                match=MatchValue(value=value)
                            )
                        )

                if conditions:
                    search_filter = Filter(must=conditions)

            # Perform search
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit,
                query_filter=search_filter,
                score_threshold=min_score
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
        filters: Optional[Dict] = None,
        min_score: Optional[float] = None
    ) -> List[Dict]:
        """
        Search for content similar to the query text using its vector representation

        Args:
            query_text: The query text (for potential use in metadata)
            query_vector: The embedding vector of the query text
            limit: Maximum number of results to return
            filters: Optional filters to apply to the search
            min_score: Optional minimum score threshold for results

        Returns:
            List of similar content with scores
        """
        return self.search_similar_content(query_vector, limit, filters, min_score)

    def search_content_by_metadata(
        self,
        metadata_filters: Dict[str, any],
        limit: int = 10
    ) -> List[Dict]:
        """
        Search for content based on metadata filters

        Args:
            metadata_filters: Dictionary of metadata fields and values to filter by
            limit: Maximum number of results to return

        Returns:
            List of content matching the metadata filters
        """
        if not QDRANT_AVAILABLE:
            logger.warning("Qdrant not available, returning empty results")
            return []

        try:
            # Prepare filters
            conditions = []
            for key, value in metadata_filters.items():
                conditions.append(
                    FieldCondition(
                        key=f"metadata.{key}",
                        match=MatchValue(value=value)
                    )
                )

            search_filter = Filter(must=conditions) if conditions else None

            # Perform search with empty query vector (to get all matching documents)
            # We'll use a zero vector and disable scoring for metadata-only search
            results = self.client.scroll(
                collection_name=self.collection_name,
                scroll_filter=search_filter,
                limit=limit
            )

            # Format results
            formatted_results = []
            for record in results[0]:  # results is a tuple (records, next_page_offset)
                formatted_results.append({
                    "id": record.id,
                    "content_text": record.payload.get("content_text", ""),
                    "metadata": record.payload.get("metadata", {})
                })

            logger.info(f"Found {len(formatted_results)} content items by metadata")
            return formatted_results

        except Exception as e:
            logger.error(f"Error searching for content by metadata: {e}")
            return []

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

    def index_textbook_content(self, content_id: str, content_text: str, module: str, content_type: str) -> bool:
        """
        Index textbook content with proper metadata for RAG functionality

        Args:
            content_id: Unique ID for the content
            content_text: The actual text content to be indexed
            module: The module this content belongs to (e.g., 'ROS 2', 'Gazebo & Unity')
            content_type: The type of content (e.g., 'text', 'video', 'exercise')

        Returns:
            bool: True if successful, False otherwise
        """
        if not QDRANT_AVAILABLE:
            logger.warning("Qdrant not available, skipping content indexing")
            return False

        try:
            # This method would typically generate embeddings for the content
            # For now, we'll use a mock implementation since the actual embedding generation
            # would require the OpenAI API or another embedding service
            # In a real implementation, you would call an embedding service here
            from openai import OpenAI
            if settings.openai_api_key:
                client = OpenAI(api_key=settings.openai_api_key)
                response = client.embeddings.create(
                    input=content_text,
                    model="text-embedding-ada-002"
                )
                vector = response.data[0].embedding
            else:
                # Fallback to mock vector if OpenAI API is not available
                vector = [0.1] * 1536

            # Prepare metadata for the content
            metadata = {
                "module": module,
                "content_type": content_type,
                "indexed_at": str(self.get_current_timestamp())
            }

            # Add the content to the vector store
            success = self.add_content_to_vector_store(
                content_id=content_id,
                content_text=content_text,
                vector=vector,
                metadata=metadata
            )

            if success:
                logger.info(f"Successfully indexed content {content_id} for RAG")
            else:
                logger.error(f"Failed to index content {content_id} for RAG")

            return success

        except Exception as e:
            logger.error(f"Error indexing textbook content: {e}")
            return False

    def index_multiple_textbook_contents(self, contents: List[Dict[str, any]]) -> bool:
        """
        Index multiple textbook contents in a batch operation

        Args:
            contents: List of dictionaries containing content_id, content_text, module, and content_type

        Returns:
            bool: True if successful, False otherwise
        """
        if not QDRANT_AVAILABLE:
            logger.warning("Qdrant not available, skipping batch content indexing")
            return False

        try:
            if not contents:
                logger.warning("Empty content list provided for batch indexing")
                return True

            # Prepare content for batch addition
            content_list = []
            for content in contents:
                content_id = content["content_id"]
                content_text = content["content_text"]
                module = content["module"]
                content_type = content["content_type"]

                # Generate embedding for the content
                from openai import OpenAI
                if settings.openai_api_key:
                    client = OpenAI(api_key=settings.openai_api_key)
                    response = client.embeddings.create(
                        input=content_text,
                        model="text-embedding-ada-002"
                    )
                    vector = response.data[0].embedding
                else:
                    # Fallback to mock vector if OpenAI API is not available
                    vector = [0.1] * 1536

                # Prepare metadata
                metadata = {
                    "module": module,
                    "content_type": content_type,
                    "indexed_at": str(self.get_current_timestamp())
                }

                content_list.append({
                    "content_id": content_id,
                    "content_text": content_text,
                    "vector": vector,
                    "metadata": metadata
                })

            # Add all contents to the vector store in a batch operation
            success = self.batch_add_content_to_vector_store(content_list)

            if success:
                logger.info(f"Successfully indexed {len(contents)} contents for RAG")
            else:
                logger.error(f"Failed to index {len(contents)} contents for RAG")

            return success

        except Exception as e:
            logger.error(f"Error indexing multiple textbook contents: {e}")
            return False

    def get_current_timestamp(self):
        """
        Helper method to get the current timestamp
        """
        from datetime import datetime
        return datetime.utcnow()


# Singleton instance of RAG Service
rag_service = RAGService()