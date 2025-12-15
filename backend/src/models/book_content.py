from sqlalchemy import Column, String, DateTime, Integer, Text
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
import uuid
from .database import Base

class BookContent(Base):
    __tablename__ = "book_content"

    id = Column(String, primary_key=True, index=True)
    title = Column(String, nullable=False)
    module = Column(String, nullable=False)  # ROS 2, Gazebo & Unity, NVIDIA Isaac, VLA
    chapter_number = Column(Integer, nullable=False)
    content_type = Column(String, default="text")  # text, video, diagram, lab, etc.
    content = Column(Text, nullable=False)
    version = Column(String, default="1.0.0")
    vector_id = Column(String)  # Reference to vector in Qdrant database
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, onupdate=func.now())
    authors = Column(String)  # JSON string array
    learning_objectives = Column(String)  # JSON string array