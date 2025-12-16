from sqlalchemy import Column, String, DateTime, Integer, Text, ForeignKey, CheckConstraint
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
from .database import Base
import json


class ContentMetadata(Base):
    __tablename__ = "content_metadata"

    # Add check constraint for personalization_level and difficulty_rating
    __table_args__ = (
        CheckConstraint(
            "personalization_level IN ('beginner', 'intermediate', 'advanced')",
            name="valid_personalization_level_check"
        ),
        CheckConstraint(
            "difficulty_rating >= 1 AND difficulty_rating <= 5",
            name="valid_difficulty_rating_check"
        ),
    )

    id = Column(String, primary_key=True, index=True)
    content_id = Column(String, nullable=False, index=True)  # Foreign key reference to BookContent
    personalization_level = Column(String, nullable=False)  # beginner/intermediate/advanced
    adjusted_content = Column(Text)
    adjusted_examples = Column(String)  # JSON string array
    difficulty_rating = Column(Integer)  # 1-5 rating, required per spec
    estimated_time_minutes = Column(Integer)  # Estimated time to complete, required per spec
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now())

    def __repr__(self):
        return f"<ContentMetadata(id={self.id}, content_id={self.content_id}, level={self.personalization_level})>"

    def to_dict(self):
        """Convert the model instance to a dictionary representation"""
        return {
            "id": self.id,
            "content_id": self.content_id,
            "personalization_level": self.personalization_level,
            "adjusted_content": self.adjusted_content,
            "adjusted_examples": json.loads(self.adjusted_examples) if self.adjusted_examples else [],
            "difficulty_rating": self.difficulty_rating,
            "estimated_time_minutes": self.estimated_time_minutes,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "updated_at": self.updated_at.isoformat() if self.updated_at else None
        }