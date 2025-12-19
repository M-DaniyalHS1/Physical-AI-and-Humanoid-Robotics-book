"""
Test the signup endpoint directly
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from src.main import app
from fastapi.testclient import TestClient

def test_signup_direct():
    client = TestClient(app)
    
    # Try with query parameters (as the endpoint currently expects)
    params = {
        "email": "test@example.com",
        "password": "test_password",
        "software_experience": "beginner",
        "hardware_experience": "intermediate",
        "math_physics_level": "advanced",
        "learning_goals": "Learn robotics"
    }
    
    # FastAPI test client will convert params to query parameters
    response = client.post("/api/v1/auth/signup", params=params)
    print(f"Response status: {response.status_code}")
    print(f"Response body: {response.text}")
    
    if response.status_code != 201:
        print("Signup failed with query parameters")
    else:
        print("Signup succeeded with query parameters")

if __name__ == "__main__":
    test_signup_direct()