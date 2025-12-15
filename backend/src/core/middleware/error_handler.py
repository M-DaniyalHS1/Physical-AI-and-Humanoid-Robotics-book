import logging
import time
from fastapi import Request
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException
from traceback import format_exc


# Set up logger
logger = logging.getLogger(__name__)


async def error_handler_middleware(request: Request, call_next):
    """
    Error handling middleware to catch exceptions and return consistent error responses
    """
    try:
        response = await call_next(request)
        return response
    except RequestValidationError as exc:
        logger.error(f"Validation error: {exc}")
        return JSONResponse(
            status_code=422,
            content={
                "detail": "Validation error",
                "errors": [
                    {
                        "loc": err["loc"],
                        "msg": err["msg"],
                        "type": err["type"]
                    } for err in exc.errors()
                ]
            }
        )
    except StarletteHTTPException as exc:
        logger.error(f"HTTP exception: {exc.detail}")
        return JSONResponse(
            status_code=exc.status_code,
            content={"detail": exc.detail}
        )
    except Exception as exc:
        logger.error(f"Unhandled exception: {str(exc)}\n{format_exc()}")
        return JSONResponse(
            status_code=500,
            content={
                "detail": "Internal server error",
                "message": "Something went wrong on our end. Please try again later."
            }
        )


def log_request_response(request: Request, response=None, error=None):
    """
    Helper function to log request and response details
    """
    log_dict = {
        "timestamp": time.time(),
        "client_host": request.client.host,
        "method": request.method,
        "path": request.url.path,
        "query_params": str(request.query_params),
        "headers": dict(request.headers),
    }
    
    if response:
        log_dict.update({
            "response_status": response.status_code,
            "response_body": getattr(response, "body", None)
        })
    
    if error:
        log_dict.update({
            "error": str(error),
            "traceback": format_exc()
        })
    
    logger.info(log_dict)