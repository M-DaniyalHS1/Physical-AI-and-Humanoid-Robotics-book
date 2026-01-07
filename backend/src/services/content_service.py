"""
Content Service for AI Textbook Platform

This module provides content management functionality for the textbook platform,
including retrieving, storing, and managing textbook content.
"""
from typing import List, Optional, Dict, Any
from sqlalchemy.orm import Session
from sqlalchemy import and_, or_, func
import logging
import json

from ..models.book_content import BookContent
from ..models.content_metadata import ContentMetadata
from ..models.progress import Progress
from .rag_service import rag_service

logger = logging.getLogger(__name__)

class ContentService:
    """
    Service class for managing textbook content operations
    """

    def __init__(self):
        # Initialize the RAG service for enhanced content retrieval
        self.rag_service = rag_service

    def get_all_content(self, db: Session, skip: int = 0, limit: int = 100) -> List[Dict[str, Any]]:
        """
        Retrieve all textbook content with pagination

        Args:
            db: Database session
            skip: Number of records to skip for pagination
            limit: Maximum number of records to return

        Returns:
            List of content records as dictionaries
        """
        try:
            content_list = db.query(BookContent).offset(skip).limit(limit).all()
            return [content.to_dict() for content in content_list]
        except Exception as e:
            logger.error(f"Error retrieving all content: {e}")
            raise

    def get_content_by_id(self, db: Session, content_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve specific content by its ID

        Args:
            db: Database session
            content_id: The ID of the content to retrieve

        Returns:
            Content record as dictionary if found, None otherwise
        """
        try:
            content = db.query(BookContent).filter(BookContent.id == content_id).first()
            if content:
                return content.to_dict()
            return None
        except Exception as e:
            logger.error(f"Error retrieving content by ID {content_id}: {e}")
            raise

    def get_content_by_module(self, db: Session, module: str, skip: int = 0, limit: int = 100) -> List[Dict[str, Any]]:
        """
        Retrieve content filtered by module

        Args:
            db: Database session
            module: The module to filter by (e.g., 'ROS 2', 'Gazebo & Unity', etc.)
            skip: Number of records to skip for pagination
            limit: Maximum number of records to return

        Returns:
            List of content records as dictionaries
        """
        try:
            content_list = db.query(BookContent).filter(
                BookContent.module == module
            ).offset(skip).limit(limit).all()
            return [content.to_dict() for content in content_list]
        except Exception as e:
            logger.error(f"Error retrieving content by module {module}: {e}")
            raise

    def get_content_by_type(self, db: Session, content_type: str, skip: int = 0, limit: int = 100) -> List[Dict[str, Any]]:
        """
        Retrieve content filtered by content type

        Args:
            db: Database session
            content_type: The content type to filter by (e.g., 'text', 'video', etc.)
            skip: Number of records to skip for pagination
            limit: Maximum number of records to return

        Returns:
            List of content records as dictionaries
        """
        try:
            content_list = db.query(BookContent).filter(
                BookContent.content_type == content_type
            ).offset(skip).limit(limit).all()
            return [content.to_dict() for content in content_list]
        except Exception as e:
            logger.error(f"Error retrieving content by type {content_type}: {e}")
            raise

    def search_content(self, db: Session, query: str, skip: int = 0, limit: int = 100) -> List[Dict[str, Any]]:
        """
        Search for content by title or content text

        Args:
            db: Database session
            query: Search query string
            skip: Number of records to skip for pagination
            limit: Maximum number of records to return

        Returns:
            List of content records as dictionaries
        """
        try:
            content_list = db.query(BookContent).filter(
                or_(
                    BookContent.title.ilike(f"%{query}%"),
                    BookContent.content.ilike(f"%{query}%")
                )
            ).offset(skip).limit(limit).all()
            return [content.to_dict() for content in content_list]
        except Exception as e:
            logger.error(f"Error searching content with query '{query}': {e}")
            raise

    def semantic_search_content(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Perform semantic search on content using RAG functionality

        Args:
            query: Search query string for semantic search
            limit: Maximum number of results to return

        Returns:
            List of content records as dictionaries with similarity scores
        """
        try:
            # Generate embedding for the query
            from openai import OpenAI
            from ..core.config import settings

            if settings.openai_api_key:
                client = OpenAI(api_key=settings.openai_api_key)
                response = client.embeddings.create(
                    input=query,
                    model="text-embedding-ada-002"
                )
                query_vector = response.data[0].embedding
            else:
                # Fallback to mock vector if OpenAI API is not available
                query_vector = [0.1] * 1536

            # Use RAG service to search for similar content
            results = self.rag_service.search_content_by_text_similarity(
                query_text=query,
                query_vector=query_vector,
                limit=limit
            )

            # Format results to match expected content structure
            formatted_results = []
            for result in results:
                formatted_results.append({
                    "id": result["id"],
                    "content": result["content_text"],
                    "metadata": result["metadata"],
                    "similarity_score": result["score"]
                })

            logger.info(f"Semantic search found {len(formatted_results)} results for query: {query}")
            return formatted_results
        except Exception as e:
            logger.error(f"Error performing semantic search with query '{query}': {e}")
            # Return empty list if semantic search fails
            return []

    def get_content_metadata(self, db: Session, content_id: str, level: Optional[str] = None) -> Optional[Dict[str, Any]]:
        """
        Retrieve content metadata for a specific content ID and personalization level

        Args:
            db: Database session
            content_id: The ID of the content
            level: Personalization level (beginner, intermediate, advanced), or None for any level

        Returns:
            Content metadata as dictionary if found, None otherwise
        """
        try:
            query = db.query(ContentMetadata).filter(ContentMetadata.content_id == content_id)
            if level:
                query = query.filter(ContentMetadata.personalization_level == level)

            metadata = query.first()
            if metadata:
                return metadata.to_dict()
            return None
        except Exception as e:
            logger.error(f"Error retrieving content metadata for content_id {content_id}, level {level}: {e}")
            raise

    def get_all_content_metadata(self, db: Session, content_id: str) -> List[Dict[str, Any]]:
        """
        Retrieve all content metadata for a specific content ID (all personalization levels)

        Args:
            db: Database session
            content_id: The ID of the content

        Returns:
            List of content metadata records as dictionaries
        """
        try:
            metadata_list = db.query(ContentMetadata).filter(
                ContentMetadata.content_id == content_id
            ).all()
            return [meta.to_dict() for meta in metadata_list]
        except Exception as e:
            logger.error(f"Error retrieving all content metadata for content_id {content_id}: {e}")
            raise

    def get_user_progress(self, db: Session, user_id: str, content_id: Optional[str] = None) -> Optional[Dict[str, Any]]:
        """
        Retrieve user progress for a specific content or all content if content_id is None

        Args:
            db: Database session
            user_id: The ID of the user
            content_id: The ID of the content (optional)

        Returns:
            User progress as dictionary if found, None otherwise
        """
        try:
            query = db.query(Progress).filter(Progress.user_id == user_id)
            if content_id:
                query = query.filter(Progress.content_id == content_id)
                progress = query.first()
                if progress:
                    return progress.to_dict()
                return None
            else:
                progress_list = query.all()
                return [progress.to_dict() for progress in progress_list]
        except Exception as e:
            logger.error(f"Error retrieving user progress for user_id {user_id}, content_id {content_id}: {e}")
            raise

    def update_user_progress(self, db: Session, user_id: str, content_id: str,
                           progress_percentage: float, time_spent_seconds: Optional[int] = None,
                           completed_exercises: Optional[List[str]] = None,
                           bookmark_position: Optional[int] = None) -> bool:
        """
        Update user progress for specific content

        Args:
            db: Database session
            user_id: The ID of the user
            content_id: The ID of the content
            progress_percentage: Progress percentage (0-100)
            time_spent_seconds: Time spent in seconds (optional)
            completed_exercises: List of completed exercise IDs (optional)
            bookmark_position: Bookmark position in content (optional)

        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Try to find existing progress record
            progress = db.query(Progress).filter(
                and_(Progress.user_id == user_id, Progress.content_id == content_id)
            ).first()

            if progress:
                # Update existing progress
                progress.progress_percentage = progress_percentage
                if time_spent_seconds is not None:
                    progress.time_spent_seconds = time_spent_seconds
                if completed_exercises is not None:
                    progress.completed_exercises = json.dumps(completed_exercises)
                if bookmark_position is not None:
                    progress.bookmark_position = bookmark_position
            else:
                # Create new progress record
                progress = Progress(
                    id=f"progress_{user_id}_{content_id}",
                    user_id=user_id,
                    content_id=content_id,
                    progress_percentage=progress_percentage,
                    time_spent_seconds=time_spent_seconds or 0,
                    completed_exercises=json.dumps(completed_exercises) if completed_exercises else None,
                    bookmark_position=bookmark_position or 0
                )
                db.add(progress)

            db.commit()
            logger.info(f"Updated progress for user {user_id} on content {content_id}")
            return True
        except Exception as e:
            logger.error(f"Error updating user progress: {e}")
            db.rollback()
            return False

    def create_book_content(self, db: Session, content_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Create new book content

        Args:
            db: Database session
            content_data: Dictionary containing the content data

        Returns:
            Created content as dictionary if successful, None otherwise
        """
        try:
            content = BookContent(**content_data)
            db.add(content)
            db.commit()
            db.refresh(content)

            # Index the content in the RAG service for semantic search
            content_id = content.id
            content_text = content.content
            module = content.module
            content_type = content.content_type

            # Index in background to not block the response
            try:
                self.rag_service.index_textbook_content(content_id, content_text, module, content_type)
            except Exception as e:
                logger.error(f"Error indexing content {content_id} in RAG service: {e}")

            logger.info(f"Created new content with ID: {content.id}")
            return content.to_dict()
        except Exception as e:
            logger.error(f"Error creating book content: {e}")
            db.rollback()
            return None

    def update_book_content(self, db: Session, content_id: str, content_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Update existing book content

        Args:
            db: Database session
            content_id: The ID of the content to update
            content_data: Dictionary containing the fields to update

        Returns:
            Updated content as dictionary if successful, None otherwise
        """
        try:
            content = db.query(BookContent).filter(BookContent.id == content_id).first()
            if not content:
                return None

            # Store old values for RAG update
            old_content_text = content.content
            old_module = content.module
            old_content_type = content.content_type

            # Update only the fields provided in content_data
            for key, value in content_data.items():
                if hasattr(content, key):
                    setattr(content, key, value)

            db.commit()
            db.refresh(content)

            # Update the content in the RAG service if content changed
            if (content.content != old_content_text or
                content.module != old_module or
                content.content_type != old_content_type):
                try:
                    # For simplicity, we'll re-index the content
                    # In a real implementation, you might want to update the existing vector
                    self.rag_service.index_textbook_content(
                        content_id,
                        content.content,
                        content.module,
                        content.content_type
                    )
                except Exception as e:
                    logger.error(f"Error updating content {content_id} in RAG service: {e}")

            logger.info(f"Updated content with ID: {content.id}")
            return content.to_dict()
        except Exception as e:
            logger.error(f"Error updating book content: {e}")
            db.rollback()
            return None

    def delete_book_content(self, db: Session, content_id: str) -> bool:
        """
        Delete book content by ID

        Args:
            db: Database session
            content_id: The ID of the content to delete

        Returns:
            bool: True if successful, False otherwise
        """
        try:
            content = db.query(BookContent).filter(BookContent.id == content_id).first()
            if not content:
                return False

            # Remove the content from the RAG service
            try:
                self.rag_service.delete_content_by_id(content_id)
            except Exception as e:
                logger.error(f"Error removing content {content_id} from RAG service: {e}")
                # Continue with deletion even if RAG removal fails

            db.delete(content)
            db.commit()
            logger.info(f"Deleted content with ID: {content_id}")
            return True
        except Exception as e:
            logger.error(f"Error deleting book content: {e}")
            db.rollback()
            return False

    def batch_index_content(self, db: Session, content_ids: List[str]) -> bool:
        """
        Batch index multiple content items in the RAG service

        Args:
            db: Database session
            content_ids: List of content IDs to index

        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Fetch content from database
            contents = db.query(BookContent).filter(BookContent.id.in_(content_ids)).all()

            # Prepare content for batch indexing
            content_list = []
            for content in contents:
                content_list.append({
                    "content_id": content.id,
                    "content_text": content.content,
                    "module": content.module,
                    "content_type": content.content_type
                })

            # Index in RAG service
            success = self.rag_service.index_multiple_textbook_contents(content_list)

            if success:
                logger.info(f"Successfully batch indexed {len(content_list)} contents")
            else:
                logger.error(f"Failed to batch index {len(content_list)} contents")

            return success
        except Exception as e:
            logger.error(f"Error in batch indexing: {e}")
            return False


# Singleton instance of Content Service
content_service = ContentService()