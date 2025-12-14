from sqlalchemy import create_engine, Column, Integer, String, DateTime, Text, Boolean, Float
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from datetime import datetime
import os
from dotenv import load_dotenv

load_dotenv()

# Database URL from environment variable
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://username:password@localhost/dbname")

# Create engine
engine = create_engine(DATABASE_URL)

# Create session
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Base class for models
Base = declarative_base()

class User(Base):
    __tablename__ = "users"

    id = Column(String, primary_key=True, index=True)
    email = Column(String, unique=True, index=True, nullable=False)
    password_hash = Column(String, nullable=False)
    software_experience = Column(String, nullable=False)  # beginner/intermediate/advanced
    hardware_experience = Column(String, nullable=False)  # beginner/intermediate/advanced
    math_physics_level = Column(String)  # beginner/intermediate/advanced
    learning_goals = Column(String)
    personalization_settings = Column(String)  # JSON string
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    last_login = Column(DateTime)

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
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    authors = Column(String)  # JSON string array
    learning_objectives = Column(String)  # JSON string array

class ChatSession(Base):
    __tablename__ = "chat_sessions"

    id = Column(String, primary_key=True, index=True)
    user_id = Column(String, nullable=False)  # Foreign key reference
    session_start = Column(DateTime, default=datetime.utcnow)
    session_end = Column(DateTime)
    selected_text = Column(Text)
    mode = Column(String, default="general")  # general or selected-text-only
    is_active = Column(Boolean, default=True)

class ChatMessage(Base):
    __tablename__ = "chat_messages"

    id = Column(String, primary_key=True, index=True)
    session_id = Column(String, nullable=False)  # Foreign key reference
    sender_type = Column(String, nullable=False)  # user or ai
    content = Column(Text, nullable=False)
    timestamp = Column(DateTime, default=datetime.utcnow)
    context_used = Column(String)  # Context from book content used to generate response

class PersonalizationProfile(Base):
    __tablename__ = "personalization_profiles"

    id = Column(String, primary_key=True, index=True)
    user_id = Column(String, nullable=False)  # Foreign key reference
    chapter_id = Column(String, nullable=False)  # Foreign key reference
    content_level = Column(String, nullable=False)  # beginner/intermediate/advanced
    example_preference = Column(String)  # theoretical/practical
    update_context = Column(String)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

class TranslationCache(Base):
    __tablename__ = "translation_cache"

    id = Column(String, primary_key=True, index=True)
    content_id = Column(String, nullable=False)  # Foreign key reference
    target_language = Column(String, nullable=False)  # e.g., 'ur' for Urdu
    translated_content = Column(Text, nullable=False)
    translation_method = Column(String, default="ai")  # ai, human, ai_with_review
    quality_score = Column(Float, default=0.0)  # 0-1 rating
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

class Progress(Base):
    __tablename__ = "progress"

    id = Column(String, primary_key=True, index=True)
    user_id = Column(String, nullable=False)  # Foreign key reference
    content_id = Column(String, nullable=False)  # Foreign key reference
    progress_percentage = Column(Float, default=0.0)  # 0-100
    time_spent_seconds = Column(Integer, default=0)
    last_accessed = Column(DateTime, default=datetime.utcnow)
    completed_exercises = Column(String)  # JSON string array
    bookmark_position = Column(Integer, default=0)  # Character offset
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

class ContentMetadata(Base):
    __tablename__ = "content_metadata"

    id = Column(String, primary_key=True, index=True)
    content_id = Column(String, nullable=False)  # Foreign key reference
    personalization_level = Column(String, nullable=False)  # beginner/intermediate/advanced
    adjusted_content = Column(Text)
    adjusted_examples = Column(String)  # JSON string array
    difficulty_rating = Column(Integer, default=1)  # 1-5 rating
    estimated_time_minutes = Column(Integer, default=10)  # Estimated time to complete

# Dependency to get database session
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()