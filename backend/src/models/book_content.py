from sqlalchemy import Column, String, DateTime, Integer, Text, CheckConstraint
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
import uuid
from .database import Base
import json


class BookContent(Base):
    __tablename__ = "book_content"

    # Add check constraint for module field to match API contract
    __table_args__ = (
        CheckConstraint(
            "module IN ('ROS 2', 'Gazebo & Unity', 'NVIDIA Isaac', 'VLA')",
            name="valid_module_check"
        ),
    )

    id = Column(String, primary_key=True, index=True)
    title = Column(String, nullable=False)
    module = Column(String, nullable=False)  # Must be one of: ROS 2, Gazebo & Unity, NVIDIA Isaac, VLA
    chapter_number = Column(Integer, nullable=False)
    content_type = Column(String, default="text")  # text, video, diagram, lab, etc.
    content = Column(Text, nullable=False)
    version = Column(String, default="1.0.0")
    vector_id = Column(String)  # Reference to vector in Qdrant database
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now())
    authors = Column(String)  # JSON string array
    learning_objectives = Column(String)  # JSON string array

    def __repr__(self):
        return f"<BookContent(id={self.id}, title='{self.title}', module='{self.module}')>"

    def to_dict(self):
        """Convert the model instance to a dictionary representation matching the API contract"""
        # Safely parse JSON fields, returning empty arrays if parsing fails
        def safe_json_parse(json_str, default=None):
            if default is None:
                default = []
            if not json_str:
                return default
            try:
                return json.loads(json_str)
            except (json.JSONDecodeError, TypeError):
                # Log the error for debugging but return default to prevent crashes
                import logging
                logger = logging.getLogger(__name__)
                logger.warning(f"Failed to parse JSON: {json_str}")
                return default

        return {
            "id": self.id,
            "title": self.title,
            "module": self.module,
            "chapter_number": self.chapter_number,
            "content_type": self.content_type,
            "content": self.content,
            "version": self.version,
            "vector_id": self.vector_id,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "updated_at": self.updated_at.isoformat() if self.updated_at else None,
            "authors": safe_json_parse(self.authors),
            "learning_objectives": safe_json_parse(self.learning_objectives)
        }