#!/usr/bin/env python3
"""
Test script for authentication implementation
"""
import sys
import os

# Set a valid database URL to bypass the .env issue
os.environ['DATABASE_URL'] = 'sqlite:///./test.db'

# Add the backend directory and backend/src directory to the path so we can import modules
backend_path = os.path.join(os.path.dirname(__file__), 'backend')
sys.path.insert(0, backend_path)
sys.path.insert(0, os.path.join(backend_path, 'src'))

def test_auth_functions():
    from api.auth import get_password_hash, verify_password
    
    print("Testing password hashing and verification...")
    
    # Test 1: Basic hashing and verification
    password = "test_password_123"
    hashed = get_password_hash(password)
    is_valid = verify_password(password, hashed)
    
    if is_valid:
        print("+ Password hashing and verification working")
    else:
        print("- Password verification failed")
        return False
        
    # Test 2: Wrong password should fail
    is_wrong_valid = verify_password("wrong_password", hashed)
    
    if not is_wrong_valid:
        print("+ Password verification correctly rejects wrong passwords")
    else:
        print("- Password verification incorrectly accepts wrong passwords")
        return False
    
    return True

def test_jwt_functions():
    import jwt
    from core.config import settings
    
    print("\nTesting JWT token creation and verification...")
    
    # Create a simple token
    data = {"sub": "test@example.com", "exp": 9999999999}  # Far future expiration
    token = jwt.encode(data, settings.jwt_secret_key, algorithm=settings.jwt_algorithm)
    
    # Verify the token
    try:
        decoded = jwt.decode(token, settings.jwt_secret_key, algorithms=[settings.jwt_algorithm])
        
        if decoded["sub"] == "test@example.com":
            print("+ JWT token creation and verification working")
            return True
        else:
            print("- JWT token verification failed: wrong subject")
            return False
    except jwt.ExpiredSignatureError:
        print("- JWT token already expired (unlikely with far future expiration)")
        return False
    except Exception as e:
        print(f"- JWT token verification failed with error: {e}")
        return False

if __name__ == "__main__":
    print("Running authentication functionality tests...\n")
    
    try:
        auth_ok = test_auth_functions()
        jwt_ok = test_jwt_functions()
        
        if auth_ok and jwt_ok:
            print("\n+ All authentication functionality tests passed!")
        else:
            print("\n- Some authentication functionality tests failed")
    except Exception as e:
        print(f"\n- Error during testing: {e}")
        import traceback
        traceback.print_exc()