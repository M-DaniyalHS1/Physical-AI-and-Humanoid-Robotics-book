from sqlalchemy import create_engine, String
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from datetime import datetime
import os
from dotenv import load_dotenv
import uuid

load_dotenv()

# Import database URL - this will be different in tests
DATABASE_URL = os.getenv("DATABASE_URL", "sqlite:///./test.db")

# Create engine - for PostgreSQL use UUID, for SQLite use String
def create_db_engine(database_url=None):
    url = database_url or DATABASE_URL
    if url.startswith("postgresql"):
        from sqlalchemy.dialects.postgresql import UUID
        return create_engine(url)
    else:
        # For SQLite, we'll use String for IDs
        return create_engine(url, connect_args={"check_same_thread": False})

# Create engine only when needed
def get_engine():
    if not hasattr(get_engine, '_engine'):
        get_engine._engine = create_db_engine()
    return get_engine._engine

# Create session (engine will be initialized when needed)
def get_SessionLocal():
    if not hasattr(get_SessionLocal, '_SessionLocal'):
        engine = get_engine()
        get_SessionLocal._SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
    return get_SessionLocal._SessionLocal()

# Base class for models
Base = declarative_base()

# Check if using PostgreSQL
def get_id_column_type(database_url=None):
    url = database_url or DATABASE_URL
    if url.startswith("postgresql"):
        from sqlalchemy.dialects.postgresql import UUID
        return UUID(as_uuid=True), uuid.uuid4
    else:
        # For SQLite, use String
        return String, str(uuid.uuid4())

# Dependency to get database session
def get_db():
    db = get_SessionLocal()
    try:
        yield db
    finally:
        db.close()