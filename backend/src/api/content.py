from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List, Optional
from fastapi.security import HTTPBearer
import logging
import re

from ..models.database import get_db
from ..services.content_service import content_service
from ..core.config import settings

router = APIRouter(prefix="/content", tags=["Content"])
security = HTTPBearer()

# Configure logging for this module
logger = logging.getLogger(__name__)

# Define constants for validation
MAX_PAGINATION_LIMIT = 1000
VALID_PERSONALIZATION_LEVELS = ["beginner", "intermediate", "advanced"]
CONTENT_ID_REGEX = re.compile(r"^[a-zA-Z0-9_-]+$")  # Valid content ID format


@router.get("/", response_model=List[dict])
def get_all_content(
    skip: int = 0,
    limit: int = 100,
    module: Optional[str] = None,
    content_type: Optional[str] = None,
    db: Session = Depends(get_db)
):
    """
    Retrieve all textbook content with pagination and optional filtering.

    Args:
        skip (int): Number of records to skip for pagination
        limit (int): Maximum number of records to return
        module (str, optional): Module to filter by (e.g., 'ROS 2', 'Gazebo & Unity', etc.)
        content_type (str, optional): Content type to filter by (e.g., 'text', 'video', etc.)
        db (Session): Database session dependency

    Returns:
        List[dict]: List of content records
    """
    try:
        # Validate pagination parameters
        if skip < 0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Skip parameter must be non-negative"
            )
        
        if limit <= 0 or limit > MAX_PAGINATION_LIMIT:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Limit parameter must be between 1 and {MAX_PAGINATION_LIMIT}"
            )

        # Log the request parameters for debugging
        filter_params = {}
        if module:
            filter_params["module"] = module
        if content_type:
            filter_params["content_type"] = content_type

        logger.info(f"Retrieving content with skip={skip}, limit={limit}, filters={filter_params}")

        # Call service with appropriate parameters
        if module:
            content_list = content_service.get_content_by_module(db, module, skip, limit)
        elif content_type:
            content_list = content_service.get_content_by_type(db, content_type, skip, limit)
        else:
            content_list = content_service.get_all_content(db, skip=skip, limit=limit)

        logger.info(f"Retrieved {len(content_list)} content items")
        return content_list
    except HTTPException:
        # Re-raise HTTP exceptions as they are
        raise
    except Exception as e:
        logger.error(f"Error retrieving content: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while retrieving content"
        )


@router.get("/{content_id}", response_model=dict)
def get_content_by_id(
    content_id: str,
    db: Session = Depends(get_db)
):
    """
    Retrieve specific content by its ID.

    Args:
        content_id (str): The ID of the content to retrieve
        db (Session): Database session dependency

    Returns:
        dict: Content record
    """
    try:
        # Validate content_id format
        if not content_id or len(content_id) == 0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Content ID cannot be empty"
            )
        
        # Check if content_id follows expected pattern (optional: can be removed if not needed)
        if not CONTENT_ID_REGEX.match(content_id):
            logger.warning(f"Content ID {content_id} does not follow expected format")
            # We'll still allow it but potentially log for monitoring

        content = content_service.get_content_by_id(db, content_id)
        if content is None:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Content with ID {content_id} not found"
            )

        logger.info(f"Retrieved content: {content_id}")
        return content
    except HTTPException:
        # Re-raise HTTP exceptions as they are
        raise
    except Exception as e:
        logger.error(f"Error retrieving content by ID {content_id}: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while retrieving content"
        )


@router.get("/{content_id}/metadata", response_model=dict)
def get_content_metadata(
    content_id: str,
    level: Optional[str] = None,
    db: Session = Depends(get_db)
):
    """
    Retrieve content metadata for personalization.

    Args:
        content_id (str): The ID of the content
        level (str, optional): Personalization level (beginner, intermediate, advanced)
        db (Session): Database session dependency

    Returns:
        dict: Content metadata
    """
    try:
        # Validate content_id
        if not content_id or len(content_id) == 0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Content ID cannot be empty"
            )
        
        # Validate level if provided
        if level and level not in VALID_PERSONALIZATION_LEVELS:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Level must be one of: {', '.join(VALID_PERSONALIZATION_LEVELS)}"
            )

        metadata = content_service.get_content_metadata(db, content_id, level)
        if metadata is None:
            level_info = f" for level {level}" if level else ""
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Content metadata{level_info} for ID {content_id} not found"
            )

        logger.info(f"Retrieved metadata for content: {content_id}, level: {level}")
        return metadata
    except HTTPException:
        # Re-raise HTTP exceptions as they are
        raise
    except Exception as e:
        logger.error(f"Error retrieving content metadata for ID {content_id}: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while retrieving content metadata"
        )


@router.get("/{content_id}/progress", response_model=dict)
def get_user_progress(
    content_id: str,
    user_id: str,
    db: Session = Depends(get_db)
):
    """
    Retrieve user progress for specific content.

    Args:
        content_id (str): The ID of the content
        user_id (str): The ID of the user
        db (Session): Database session dependency

    Returns:
        dict: User progress record
    """
    try:
        # Validate input parameters
        if not content_id or len(content_id) == 0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Content ID cannot be empty"
            )
        
        if not user_id or len(user_id) == 0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="User ID cannot be empty"
            )
        
        # Validate content_id format
        if not CONTENT_ID_REGEX.match(content_id):
            logger.warning(f"Content ID {content_id} does not follow expected format")
        
        # Validate user_id format (in a real app, you might have specific rules)
        if len(user_id) < 3:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="User ID must be at least 3 characters long"
            )

        progress = content_service.get_user_progress(db, user_id, content_id)
        if progress is None:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Progress for user {user_id} on content {content_id} not found"
            )

        logger.info(f"Retrieved progress for user {user_id} on content {content_id}")
        return progress
    except HTTPException:
        # Re-raise HTTP exceptions as they are
        raise
    except Exception as e:
        logger.error(f"Error retrieving user progress for user {user_id} on content {content_id}: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while retrieving user progress"
        )


@router.post("/{content_id}/progress", response_model=bool)
def update_user_progress(
    content_id: str,
    user_id: str,
    progress_percentage: float,
    time_spent_seconds: Optional[int] = 0,
    completed_exercises: Optional[List[str]] = None,
    bookmark_position: Optional[int] = 0,
    db: Session = Depends(get_db)
):
    """
    Update user progress for specific content.

    Args:
        content_id (str): The ID of the content
        user_id (str): The ID of the user
        progress_percentage (float): Progress percentage (0-100)
        time_spent_seconds (int, optional): Time spent in seconds
        completed_exercises (List[str], optional): List of completed exercise IDs
        bookmark_position (int, optional): Bookmark position in content
        db (Session): Database session dependency

    Returns:
        bool: True if successful
    """
    try:
        # Validate required parameters
        if not content_id or len(content_id) == 0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Content ID cannot be empty"
            )
        
        if not user_id or len(user_id) == 0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="User ID cannot be empty"
            )
        
        # Validate content_id format
        if not CONTENT_ID_REGEX.match(content_id):
            logger.warning(f"Content ID {content_id} does not follow expected format")

        # Validate user_id format (in a real app, you might have specific rules)
        if len(user_id) < 3:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="User ID must be at least 3 characters long"
            )

        # Validate progress_percentage
        if not (0 <= progress_percentage <= 100):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Progress percentage must be between 0 and 100"
            )
        
        # Validate time_spent_seconds if provided
        if time_spent_seconds is not None and time_spent_seconds < 0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Time spent must be non-negative"
            )
        
        # Validate bookmark_position if provided
        if bookmark_position is not None and bookmark_position < 0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Bookmark position must be non-negative"
            )
        
        # Validate completed_exercises if provided
        if completed_exercises is not None:
            if not isinstance(completed_exercises, list):
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="Completed exercises must be a list of strings"
                )
            # Validate each exercise ID
            for exercise_id in completed_exercises:
                if not isinstance(exercise_id, str) or len(exercise_id) == 0:
                    raise HTTPException(
                        status_code=status.HTTP_400_BAD_REQUEST,
                        detail=f"Exercise ID '{exercise_id}' is invalid"
                    )

        success = content_service.update_user_progress(
            db, user_id, content_id, progress_percentage,
            time_spent_seconds, completed_exercises, bookmark_position
        )

        if success:
            logger.info(f"Updated progress for user {user_id} on content {content_id}: {progress_percentage}%")
            return success
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to update user progress"
            )
    except HTTPException:
        # Re-raise HTTP exceptions as they are
        raise
    except Exception as e:
        logger.error(f"Error updating user progress for user {user_id} on content {content_id}: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while updating user progress"
        )


@router.get("/{content_id}/search", response_model=List[dict])
def search_content(
    content_id: str,
    query: str,
    db: Session = Depends(get_db)
):
    """
    Search within content based on a query string.

    Args:
        content_id (str): The ID of the content (or '*' for all content)
        query (str): The search query string
        db (Session): Database session dependency

    Returns:
        List[dict]: List of content records matching the search
    """
    try:
        # Validate input parameters
        if not query or len(query.strip()) == 0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Search query cannot be empty"
            )
        
        if len(query) > 1000:  # Prevent overly long queries
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Search query is too long (max 1000 characters)"
            )
        
        # If content_id is '*', search across all content
        if content_id == "*":
            results = content_service.search_content(db, query, skip=0, limit=50)
        else:
            # For a specific content, we might want to implement section-based search
            # For now, we'll return the specific content if it matches the query
            content = content_service.get_content_by_id(db, content_id)
            if content is None:
                raise HTTPException(
                    status_code=status.HTTP_404_NOT_FOUND,
                    detail=f"Content with ID {content_id} not found"
                )
            
            # Check if the query matches the content (simplified search)
            search_results = []
            if query.lower() in content.get('title', '').lower() or query.lower() in content.get('content', '').lower():
                search_results = [content]
            
            results = search_results

        logger.info(f"Search for '{query}' in content {content_id} returned {len(results)} results")
        return results
    except HTTPException:
        # Re-raise HTTP exceptions as they are
        raise
    except Exception as e:
        logger.error(f"Error searching content with query '{query}': {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while searching content"
        )