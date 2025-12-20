"""
Debug script to simulate the exact API call that's failing in production
"""
from src.services.content_service import content_service
from src.models.database import get_SessionLocal
from contextlib import contextmanager
import traceback
import sys

@contextmanager
def get_test_db():
    db = get_SessionLocal()
    try:
        yield db
    finally:
        db.close()

def debug_content_retrieval():
    print("Debugging content retrieval process...")
    
    try:
        print("1. Creating database session...")
        with get_test_db() as db:
            print("   PASS: Database session created successfully")
            
            print("2. Calling content service to get all content...")
            content_list = content_service.get_all_content(db, skip=0, limit=100)
            print(f"   PASS: Content service call successful, retrieved {len(content_list)} items")

            print("3. Checking content items...")
            for i, content in enumerate(content_list[:2]):  # Just check first 2 items
                print(f"   Content {i+1}: ID={content.get('id')}, Title={content.get('title')}")

                # Check if JSON parsing works on each item
                try:
                    authors = content.get('authors', [])
                    learning_objectives = content.get('learning_objectives', [])
                    print(f"     - Authors: {authors} (type: {type(authors)})")
                    print(f"     - Learning Objectives: {learning_objectives} (type: {type(learning_objectives)})")
                except Exception as e:
                    print(f"     - ERROR in JSON parsing: {e}")
                    traceback.print_exc()

            print("\n4. Testing filtering by module...")
            ros_content = content_service.get_content_by_module(db, "ROS 2", skip=0, limit=100)
            print(f"   PASS: Filtered content by 'ROS 2' module, retrieved {len(ros_content)} items")

            print("\n5. Testing content by ID...")
            if content_list:
                sample_id = content_list[0]['id']
                content_by_id = content_service.get_content_by_id(db, sample_id)
                if content_by_id:
                    print(f"   PASS: Retrieved content by ID '{sample_id}' successfully")
                else:
                    print(f"   PASS: Content with ID '{sample_id}' not found (expected for non-existent IDs)")

            return True

    except Exception as e:
        print(f"   FAIL: Error occurred: {e}")
        print("   Full traceback:")
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = debug_content_retrieval()
    if success:
        print("\nContent retrieval debugging completed successfully!")
    else:
        print("\nContent retrieval debugging failed!")
        sys.exit(1)