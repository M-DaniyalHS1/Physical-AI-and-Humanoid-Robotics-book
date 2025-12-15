from sqlalchemy import Column, String, DateTime, Text
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
import uuid
from .database import Base

class PersonalizationProfile(Base):
    __tablename__ = "personalization_profiles"

    id = Column(String, primary_key=True, index=True)
    user_id = Column(String, nullable=False)  # Foreign key reference
    chapter_id = Column(String, nullable=False)  # Foreign key reference
    content_level = Column(String, nullable=False)  # beginner/intermediate/advanced
    example_preference = Column(String)  # theoretical/practical
    update_context = Column(String)
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, onupdate=func.now())