from sqlalchemy import Column, String, DateTime, Float, Integer, CheckConstraint
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
import uuid
from .database import Base
import json


class Progress(Base):
    __tablename__ = "progress"

    # Add check constraints for validation based on the spec
    __table_args__ = (
        CheckConstraint(
            "progress_percentage >= 0 AND progress_percentage <= 100",
            name="valid_progress_percentage_check"
        ),
        CheckConstraint(
            "bookmark_position >= 0",
            name="valid_bookmark_position_check"
        ),
    )

    id = Column(String, primary_key=True, index=True)
    user_id = Column(String, nullable=False, index=True)  # Foreign key reference to User
    content_id = Column(String, nullable=False, index=True)  # Foreign key reference to BookContent
    progress_percentage = Column(Float, nullable=False)  # 0-100, required per spec
    time_spent_seconds = Column(Integer, default=0)
    last_accessed = Column(DateTime, default=func.now())
    completed_exercises = Column(String)  # JSON string array
    bookmark_position = Column(Integer, default=0)  # Character offset, non-negative per spec
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now())

    def __repr__(self):
        return f"<Progress(id={self.id}, user_id={self.user_id}, content_id={self.content_id}, percentage={self.progress_percentage})>"

    def to_dict(self):
        """Convert the model instance to a dictionary representation"""
        return {
            "id": self.id,
            "user_id": self.user_id,
            "content_id": self.content_id,
            "progress_percentage": self.progress_percentage,
            "time_spent_seconds": self.time_spent_seconds,
            "last_accessed": self.last_accessed.isoformat() if self.last_accessed else None,
            "completed_exercises": json.loads(self.completed_exercises) if self.completed_exercises else [],
            "bookmark_position": self.bookmark_position,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "updated_at": self.updated_at.isoformat() if self.updated_at else None
        }