from sqlalchemy import Column, String, DateTime, Integer, Text
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
import uuid
from .database import Base

class User(Base):
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String, unique=True, nullable=False, index=True)
    password_hash = Column(String, nullable=False)
    software_experience = Column(String, nullable=False)  # beginner/intermediate/advanced
    hardware_experience = Column(String, nullable=False)  # beginner/intermediate/advanced
    math_physics_level = Column(String, nullable=False)  # beginner/intermediate/advanced
    learning_goals = Column(Text)
    personalization_settings = Column(String)  # JSON string for personalization settings
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())
    last_login = Column(DateTime(timezone=True))

    def __repr__(self):
        return f"<User(id={self.id}, email={self.email})>"