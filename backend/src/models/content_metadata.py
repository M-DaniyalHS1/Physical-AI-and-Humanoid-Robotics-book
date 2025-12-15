from sqlalchemy import Column, String, DateTime, Integer, Text, Float
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
import uuid
from .database import Base

class ContentMetadata(Base):
    __tablename__ = "content_metadata"

    id = Column(String, primary_key=True, index=True)
    content_id = Column(String, nullable=False)  # Foreign key reference
    personalization_level = Column(String, nullable=False)  # beginner/intermediate/advanced
    adjusted_content = Column(Text)
    adjusted_examples = Column(String)  # JSON string array
    difficulty_rating = Column(Integer, default=1)  # 1-5 rating
    estimated_time_minutes = Column(Integer, default=10)  # Estimated time to complete
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, onupdate=func.now())