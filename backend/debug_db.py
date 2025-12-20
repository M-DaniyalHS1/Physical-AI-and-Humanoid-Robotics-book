"""
Debug script to test database connectivity and identify potential issues
"""
import os
import sys
from sqlalchemy import create_engine, text
from sqlalchemy.orm import sessionmaker
from src.models.database import get_engine
from src.models.book_content import BookContent

def test_db_connection():
    print("Testing database connection...")
    
    try:
        # Get the engine
        engine = get_engine()
        print(f"Engine created successfully with URL: {engine.url}")
        
        # Test the connection
        with engine.connect() as connection:
            print("PASS: Database connection successful")

            # Test a simple query
            result = connection.execute(text("SELECT 1"))
            print(f"PASS: Simple query successful: {result.fetchone()}")

        # Test creating a session
        SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
        db = SessionLocal()
        try:
            # Test querying the book_content table
            content_count = db.query(BookContent).count()
            print(f"PASS: Query to book_content table successful, found {content_count} records")
        finally:
            db.close()

        return True
    except Exception as e:
        print(f"FAIL: Database connection failed: {e}")
        return False

if __name__ == "__main__":
    success = test_db_connection()
    if success:
        print("\nDatabase connectivity test passed!")
    else:
        print("\nDatabase connectivity test failed!")
        sys.exit(1)