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
        def __init__(self, id=None, vector=None, payload=None):
            self.id = id
            self.vector = vector
            self.payload = payload
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

from sqlalchemy.orm import Session
from openai import OpenAI
import requests

from ..core.config import settings
from .qdrant_client import get_qdrant_client
from ..models.book_content import BookContent

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
        # Initialize OpenAI client for generating embeddings
        if settings.openai_api_key:
            self.openai_client = OpenAI(api_key=settings.openai_api_key)
            logger.info("RAGService initialized with OpenAI client for embeddings")
        else:
            self.openai_client = None
            logger.warning("OpenAI API key not found. Embedding functionality will use mock responses.")

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

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for the given text using Hugging Face API

        Args:
            text: Text to generate embedding for

        Returns:
            List of floats representing the embedding vector
        """
        if settings.huggingface_api_key:
            try:
                # Using Hugging Face Inference API with a sentence transformer model
                hf_api_url = "https://api-inference.huggingface.co/models/sentence-transformers/all-MiniLM-L6-v2"

                headers = {
                    "Authorization": f"Bearer {settings.huggingface_api_key}",
                    "Content-Type": "application/json"
                }

                payload = {
                    "inputs": text,
                    "options": {
                        "wait_for_model": True  # Wait for model to load if it's not loaded
                    }
                }

                response = requests.post(hf_api_url, headers=headers, json=payload)

                if response.status_code == 200:
                    embedding = response.json()
                    # Ensure the embedding is a flat list of floats
                    if isinstance(embedding, list) and len(embedding) > 0:
                        if isinstance(embedding[0], list):
                            # If it's a nested list, flatten it
                            return embedding[0]
                        return embedding
                    else:
                        logger.error(f"Hugging Face API returned unexpected format: {embedding}")
                        return [0.0] * 384  # Default size for all-MiniLM-L6-v2
                else:
                    logger.error(f"Hugging Face API error: {response.status_code} - {response.text}")
                    return [0.0] * 384  # Default size for all-MiniLM-L6-v2
            except Exception as e:
                logger.error(f"Error generating embedding with Hugging Face: {e}")
                return [0.0] * 384  # Default size for all-MiniLM-L6-v2
        elif self.openai_client:
            # Fallback to OpenAI if Hugging Face API key is not available
            logger.warning("Hugging Face API key not available, falling back to OpenAI")
            try:
                response = self.openai_client.embeddings.create(
                    input=text,
                    model="text-embedding-ada-002"
                )
                return response.data[0].embedding
            except Exception as e:
                logger.error(f"Error generating embedding for text: {e}")
                return [0.0] * 1536
        else:
            # Fallback to mock embedding if neither service is available
            logger.warning("No embedding service available, using mock embedding")
            return [0.0] * 384  # Default to Hugging Face model size

    def index_all_content(self, db: Session, batch_size: int = 100) -> bool:
        """
        Index all textbook content from PostgreSQL to Qdrant vector store

        Args:
            db: Database session
            batch_size: Number of records to process in each batch

        Returns:
            bool: True if successful, False otherwise
        """
        if not QDRANT_AVAILABLE:
            logger.error("Qdrant not available, cannot index content")
            return False

        try:
            # Get total count of content
            total_content = db.query(BookContent).count()
            logger.info(f"Starting to index {total_content} content items")

            # Process content in batches
            offset = 0
            indexed_count = 0

            while offset < total_content:
                # Get a batch of content
                content_batch = db.query(BookContent).offset(offset).limit(batch_size).all()

                # Prepare points for batch upsert
                points = []
                for content in content_batch:
                    # Generate embedding for the content
                    content_text = f"{content.title} {content.content}"
                    embedding = self.generate_embedding(content_text)

                    # Create metadata
                    metadata = {
                        "title": content.title,
                        "module": content.module,
                        "chapter_number": content.chapter_number,
                        "content_type": content.content_type,
                        "version": content.version,
                        "created_at": content.created_at.isoformat() if content.created_at else None,
                        "updated_at": content.updated_at.isoformat() if content.updated_at else None
                    }

                    # Create point structure
                    point = PointStruct(
                        id=content.id,
                        vector=embedding,
                        payload={
                            "content_id": content.id,
                            "content_text": content_text,
                            "metadata": metadata
                        }
                    )
                    points.append(point)

                # Upsert the batch to Qdrant
                if points:
                    self.client.upsert(
                        collection_name=self.collection_name,
                        points=points
                    )
                    indexed_count += len(points)
                    logger.info(f"Indexed batch: {offset} to {offset + len(points)}, total indexed: {indexed_count}")

                # Move to next batch
                offset += batch_size

            logger.info(f"Successfully indexed {indexed_count} content items to Qdrant")
            return True

        except Exception as e:
            logger.error(f"Error indexing content to Qdrant: {e}")
            return False

    def index_single_content(self, db: Session, content_id: str) -> bool:
        """
        Index a single content item from PostgreSQL to Qdrant vector store

        Args:
            db: Database session
            content_id: ID of the content to index

        Returns:
            bool: True if successful, False otherwise
        """
        if not QDRANT_AVAILABLE:
            logger.error("Qdrant not available, cannot index content")
            return False

        try:
            # Get the content from database
            content = db.query(BookContent).filter(BookContent.id == content_id).first()
            if not content:
                logger.error(f"Content with ID {content_id} not found in database")
                return False

            # Generate embedding for the content
            content_text = f"{content.title} {content.content}"
            embedding = self.generate_embedding(content_text)

            # Create metadata
            metadata = {
                "title": content.title,
                "module": content.module,
                "chapter_number": content.chapter_number,
                "content_type": content.content_type,
                "version": content.version,
                "created_at": content.created_at.isoformat() if content.created_at else None,
                "updated_at": content.updated_at.isoformat() if content.updated_at else None
            }

            # Create point structure
            point = PointStruct(
                id=content.id,
                vector=embedding,
                payload={
                    "content_id": content.id,
                    "content_text": content_text,
                    "metadata": metadata
                }
            )

            # Upsert to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=[point]
            )

            logger.info(f"Successfully indexed content {content_id} to Qdrant")
            return True

        except Exception as e:
            logger.error(f"Error indexing content {content_id} to Qdrant: {e}")
            return False

    def sync_content_changes(self, db: Session, content_id: str) -> bool:
        """
        Sync content changes between PostgreSQL and Qdrant vector store

        Args:
            db: Database session
            content_id: ID of the content to sync

        Returns:
            bool: True if successful, False otherwise
        """
        if not QDRANT_AVAILABLE:
            logger.error("Qdrant not available, cannot sync content")
            return False

        try:
            # Get the content from database
            content = db.query(BookContent).filter(BookContent.id == content_id).first()
            if not content:
                # Content might have been deleted, so delete from Qdrant too
                success = self.delete_content_by_id(content_id)
                if success:
                    logger.info(f"Deleted content {content_id} from Qdrant (content no longer exists in DB)")
                return success

            # Update the content in Qdrant
            success = self.index_single_content(db, content_id)
            if success:
                logger.info(f"Synced content {content_id} between PostgreSQL and Qdrant")
            return success

        except Exception as e:
            logger.error(f"Error syncing content {content_id}: {e}")
            return False

# Singleton instance of RAG Service
rag_service = RAGService()