"""
Direct API endpoint test to simulate the exact issue in production
"""
from src.api.content import get_all_content
from src.models.database import get_SessionLocal
from contextlib import contextmanager
import sys
import traceback

@contextmanager
def get_test_db():
    db = get_SessionLocal()
    try:
        yield db
    finally:
        db.close()

def test_direct_api_endpoint():
    print("Testing the direct API endpoint function...")
    
    try:
        print("1. Calling get_all_content API function directly...")
        
        # This simulates the exact call made by the API endpoint
        with get_test_db() as db:
            result = get_all_content(skip=0, limit=100, module=None, content_type=None, db=db)
            
        print(f"2. Function executed successfully, returned {len(result)} items")
        
        # Print first item to verify content
        if result:
            first_item = result[0]
            print(f"3. First item: ID={first_item.get('id')}, Title={first_item.get('title')}")
            
        print("4. All tests passed!")
        return True
        
    except Exception as e:
        print(f"FAIL: Error occurred: {e}")
        print("Full traceback:")
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_direct_api_endpoint()
    if success:
        print("\nDirect API endpoint test completed successfully!")
    else:
        print("\nDirect API endpoint test failed!")
        sys.exit(1)