from sqlalchemy import Column, String, DateTime, Float, Integer
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
import uuid
from .database import Base

class Progress(Base):
    __tablename__ = "progress"

    id = Column(String, primary_key=True, index=True)
    user_id = Column(String, nullable=False)  # Foreign key reference
    content_id = Column(String, nullable=False)  # Foreign key reference
    progress_percentage = Column(Float, default=0.0)  # 0-100
    time_spent_seconds = Column(Integer, default=0)
    last_accessed = Column(DateTime, default=func.now())
    completed_exercises = Column(String)  # JSON string array
    bookmark_position = Column(Integer, default=0)  # Character offset
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, onupdate=func.now())