from pydantic_settings import BaseSettings
from pydantic import field_validator, ValidationInfo
from typing import Optional
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class Settings(BaseSettings):
    # Application
    app_name: str = "AI-Powered Physical AI & Humanoid Robotics Textbook API"
    app_version: str = "1.0.0"
    environment: str = "development"  # development, staging, production
    debug: bool = False
    log_level: str = "INFO"

    # Database
    database_url: str = "sqlite:///./development.db"  # Default for development

    # JWT
    jwt_secret_key: str = "dev-secret-key-change-in-production"  # Default for development
    jwt_algorithm: str = "HS256"
    access_token_expire_minutes: int = 30

    # API Rate Limits - format should be "X/Y" where X is number and Y is time unit
    chat_rate_limit: str = "10/minute"
    translation_rate_limit: str = "50/minute"
    content_rate_limit: str = "100/minute"

    # External Service URLs
    qdrant_url: Optional[str] = None
    qdrant_api_key: Optional[str] = None
    openai_api_key: Optional[str] = None

    # Redis (for rate limiting and caching)
    redis_url: Optional[str] = None

    @field_validator('database_url')
    @classmethod
    def validate_database_url(cls, v):
        if not v or v == "":
            raise ValueError('DATABASE_URL must be set in environment variables')
        # Warn if using default development DB in non-development environment
        if v == "sqlite:///./development.db":
            import warnings
            warnings.warn("Using default development database. This should be changed for production use.")
        return v

    @field_validator('jwt_secret_key')
    @classmethod
    def validate_jwt_secret_key(cls, v):
        if not v or v == "":
            raise ValueError('JWT_SECRET_KEY must be set in environment variables')
        # Warn if using default development key in non-development environment
        if v == "dev-secret-key-change-in-production":
            import warnings
            warnings.warn("Using default development JWT secret. This should be changed for production use.")
        return v

    @field_validator('chat_rate_limit', 'translation_rate_limit', 'content_rate_limit')
    @classmethod
    def validate_rate_limit_format(cls, v):
        if '/' not in v:
            raise ValueError(f'Rate limit format must be "X/Y" (e.g., "10/minute"), got: {v}')
        count, period = v.split('/', 1)
        try:
            int(count)
        except ValueError:
            raise ValueError(f'Rate limit count must be an integer, got: {count}')
        if period not in ['second', 'minute', 'hour']:
            raise ValueError(f'Rate limit period must be "second", "minute", or "hour", got: {period}')
        return v

    @field_validator('debug')
    @classmethod
    def validate_debug(cls, v):
        if isinstance(v, str):
            return v.lower() in ['true', '1', 'yes', 'on']
        return bool(v)

    class Config:
        env_file = ".env"
        case_sensitive = True
        extra = "ignore"  # Ignore extra fields from .env that aren't defined as attributes

    def is_development(self) -> bool:
        return self.environment.lower() == "development"

    def is_production(self) -> bool:
        return self.environment.lower() == "production"

    def is_staging(self) -> bool:
        return self.environment.lower() == "staging"

settings = Settings()