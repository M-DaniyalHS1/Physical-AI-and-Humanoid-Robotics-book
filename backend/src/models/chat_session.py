from sqlalchemy import Column, String, DateTime, Boolean, Text, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
import uuid
from .database import Base

class ChatSession(Base):
    __tablename__ = "chat_sessions"

    id = Column(String, primary_key=True, index=True)
    user_id = Column(String, nullable=False)  # Foreign key reference
    session_start = Column(DateTime, default=func.now())
    session_end = Column(DateTime)
    selected_text = Column(Text)
    mode = Column(String, default="general")  # general or selected-text-only
    is_active = Column(Boolean, default=True)

    # Relationship to chat messages
    messages = relationship("ChatMessage", back_populates="chat_session", cascade="all, delete-orphan")