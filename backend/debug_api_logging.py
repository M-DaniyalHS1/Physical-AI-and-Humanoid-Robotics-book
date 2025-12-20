"""
Enhanced logging for content API to help debug production issues
"""
import logging
from functools import wraps

def debug_api_endpoint(func):
    """Decorator to add enhanced logging to API endpoints"""
    @wraps(func)
    def wrapper(*args, **kwargs):
        logger = logging.getLogger(__name__)
        logger.info(f"API endpoint {func.__name__} called with args: {args}, kwargs: {kwargs}")
        
        try:
            result = func(*args, **kwargs)
            logger.info(f"API endpoint {func.__name__} completed successfully")
            return result
        except Exception as e:
            logger.error(f"API endpoint {func.__name__} failed with error: {e}", exc_info=True)
            raise
    
    return wrapper

# Example of how this would be used in the content API:
'''
@router.get("/", response_model=List[dict])
@debug_api_endpoint
def get_all_content(
    skip: int = 0,
    limit: int = 100,
    module: Optional[str] = None,
    content_type: Optional[str] = None,
    db: Session = Depends(get_db)
):
    # ... existing implementation
'''