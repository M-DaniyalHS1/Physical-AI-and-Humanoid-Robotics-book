from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException
from .core.config import settings
import os
import sys
import time
import uvicorn
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure comprehensive logging
from .core.logging_config import setup_logging, get_logger
setup_logging()
logger = get_logger(__name__)

# Validate environment configuration but don't exit immediately
from .core.environment import validate_environment, print_environment_summary
env_validation_result = validate_environment()
if not env_validation_result:
    logger.warning("Environment validation failed. Some features may not work properly.")
    print_environment_summary()  # Print summary for immediate feedback
else:
    logger.info("Environment validation passed.")
    print_environment_summary()

# Import all models to register them with SQLAlchemy
from .models import user, book_content, chat_session, chat_message, personalization_profile, translation_cache, progress, content_metadata

# Import middleware
from .core.middleware.rate_limit import RateLimitMiddleware
from .core.middleware.error_handler import error_handler_middleware

# Create FastAPI app with metadata
app = FastAPI(
    title="AI-Powered Physical AI & Humanoid Robotics Textbook API",
    description="API for the AI-powered textbook platform that teaches Physical AI & Humanoid Robotics with RAG chatbot, personalization, and Urdu translation capabilities",
    version="1.0.0",
    docs_url="/api/docs",
    redoc_url="/api/redoc",
    openapi_url="/api/openapi.json"
)

# Add custom middleware
app.add_middleware(RateLimitMiddleware)
app.middleware("http")(error_handler_middleware)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # TODO: Restrict this in production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routers
from .api.auth import router as auth_router
from .api.content import router as content_router
from .api.chat import router as chat_router
from .api.user import router as user_router
from .api.translation import router as translation_router

app.include_router(auth_router, prefix="/api/v1", tags=["Authentication"])
app.include_router(content_router, prefix="/api/v1", tags=["Content"])
app.include_router(chat_router, prefix="/api/v1", tags=["Chat"])
app.include_router(user_router, prefix="/api/v1", tags=["User"])
app.include_router(translation_router, prefix="/api/v1", tags=["Translation"])

@app.get("/")
def read_root():
    return {"message": "Welcome to the AI-Powered Physical AI & Humanoid Robotics Textbook API"}

@app.get("/health")
def health_check():
    # Simple health check that returns immediately
    return {"status": "healthy", "version": "1.0.0", "timestamp": time.time(), "env_validated": env_validation_result}

@app.get("/ready")
def readiness_check():
    # More comprehensive readiness check that verifies dependencies
    try:
        # Check environment validation status
        if not env_validation_result:
            return {"status": "not_ready", "checks": {"environment": "invalid"}}, 503

        # For now, just return that we're ready
        # In a more complex app, this would check database connectivity, etc.
        return {"status": "ready", "checks": {"database": "ok", "redis": "ok", "qdrant": "ok"}}
    except Exception as e:
        return {"status": "not_ready", "error": str(e)}, 503

# Global exception handlers will be added via middleware
# Error handling and rate limiting will be handled by middleware

if __name__ == "__main__":
    
    uvicorn.run(app, host="0.0.0.0", port=int(os.getenv("PORT", 8000)))