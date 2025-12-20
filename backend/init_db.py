"""
Database initialization script for the AI Textbook Platform
This script ensures that the database schema is up-to-date
"""
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from alembic.config import Config
from alembic import command
from alembic.script import ScriptDirectory
from alembic.runtime.environment import EnvironmentContext
from alembic.runtime.migration import MigrationContext
import os
from src.models.database import DATABASE_URL, Base

def run_migrations_online():
    """Run migrations in 'online' mode."""
    # Use the same database URL as the application
    connectable = create_engine(DATABASE_URL)

    with connectable.connect() as connection:
        # Check current revision
        context = MigrationContext.configure(connection)
        current_rev = context.get_current_revision()
        
        # Run the migrations
        alembic_cfg = Config("alembic.ini")
        alembic_cfg.set_main_option("sqlalchemy.url", DATABASE_URL)
        command.upgrade(alembic_cfg, "head")

def init_db():
    """Initialize the database with all tables and run migrations"""
    try:
        # First run alembic migrations to ensure schema is up-to-date
        print("Running database migrations...")
        run_migrations_online()
        print("Migrations completed successfully")
        
        # Then create tables as a fallback (for cases where alembic isn't used)
        engine = create_engine(DATABASE_URL)
        Base.metadata.create_all(bind=engine)
        print("Database tables created/verified successfully")
        
        return True
    except Exception as e:
        print(f"Error initializing database: {e}")
        return False

if __name__ == "__main__":
    print("Initializing database...")
    success = init_db()
    if success:
        print("Database initialization completed successfully")
    else:
        print("Database initialization failed")