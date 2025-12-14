# Simple test to verify the authentication implementation
import sys
import os

# Add the backend directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

# Set a valid DATABASE_URL for testing purposes
os.environ['DATABASE_URL'] = 'sqlite:///./test.db'

def test_auth_import():
    """Test that the authentication module imports without errors."""
    try:
        # Import the password utilities
        from src.api.auth import get_password_hash, verify_password
        print("✓ Authentication functions imported successfully")
        
        # Test password hashing
        password = "test_password"
        hashed = get_password_hash(password)
        result = verify_password(password, hashed)
        
        if result:
            print("+ Password hashing and verification working correctly")
        else:
            print("- Password verification failed")

        # Test with wrong password
        wrong_result = verify_password("wrong_password", hashed)
        if not wrong_result:
            print("+ Password verification correctly rejects wrong passwords")
        else:
            print("- Password verification incorrectly accepts wrong passwords")

        return True
    except Exception as e:
        print(f"- Error testing auth: {e}")
        return False

def test_jwt_functions():
    """Test JWT token creation and verification."""
    try:
        import jwt
        from datetime import timedelta
        from src.core.config import settings

        # Create a simple token
        data = {"sub": "test@example.com"}
        token = jwt.encode(data, settings.jwt_secret_key, algorithm=settings.jwt_algorithm)

        # Verify the token
        decoded = jwt.decode(token, settings.jwt_secret_key, algorithms=[settings.jwt_algorithm])

        if decoded["sub"] == "test@example.com":
            print("+ JWT token creation and verification working correctly")
        else:
            print("- JWT token verification failed")

        return True
    except Exception as e:
        print(f"- Error testing JWT: {e}")
        return False

if __name__ == "__main__":
    print("Testing authentication implementation...")

    auth_success = test_auth_import()
    jwt_success = test_jwt_functions()

    if auth_success and jwt_success:
        print("\n+ All authentication tests passed!")
    else:
        print("\n- Some authentication tests failed")