from sqlalchemy import Column, String, DateTime, Text, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
import uuid
from .database import Base

class ChatMessage(Base):
    __tablename__ = "chat_messages"

    id = Column(String, primary_key=True, index=True)
    session_id = Column(String, ForeignKey("chat_sessions.id"), nullable=False)  # Foreign key reference
    sender_type = Column(String, nullable=False)  # user or ai
    content = Column(Text, nullable=False)
    timestamp = Column(DateTime, default=func.now())
    context_used = Column(String)  # Context from book content used to generate response

    # Relationship to chat session
    chat_session = relationship("ChatSession", back_populates="messages")