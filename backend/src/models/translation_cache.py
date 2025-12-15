from sqlalchemy import Column, String, DateTime, Text, Float
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
import uuid
from .database import Base

class TranslationCache(Base):
    __tablename__ = "translation_cache"

    id = Column(String, primary_key=True, index=True)
    content_id = Column(String, nullable=False)  # Foreign key reference
    target_language = Column(String, nullable=False)  # e.g., 'ur' for Urdu
    translated_content = Column(Text, nullable=False)
    translation_method = Column(String, default="ai")  # ai, human, ai_with_review
    quality_score = Column(Float, default=0.0)  # 0-1 rating
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, onupdate=func.now())