try:
    from pydantic_settings import BaseSettings
except ImportError:
    from pydantic import BaseSettings
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class Settings(BaseSettings):
    # Database
    database_url: str = os.getenv("DATABASE_URL", "")

    # JWT
    jwt_secret_key: str = os.getenv("JWT_SECRET_KEY", "fallback_secret_key_for_development")
    jwt_algorithm: str = os.getenv("JWT_ALGORITHM", "HS256")
    access_token_expire_minutes: int = int(os.getenv("ACCESS_TOKEN_EXPIRE_MINUTES", "30"))

    # API Rate Limits
    chat_rate_limit: str = os.getenv("CHAT_RATE_LIMIT", "10/minute")
    translation_rate_limit: str = os.getenv("TRANSLATION_RATE_LIMIT", "50/minute")
    content_rate_limit: str = os.getenv("CONTENT_RATE_LIMIT", "100/minute")

    # External Service URLs
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    openai_api_key: str = os.getenv("OPENAI_API_KEY", "")

    class Config:
        env_file = ".env"

settings = Settings()