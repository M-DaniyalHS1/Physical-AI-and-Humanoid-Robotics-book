import pytest
from fastapi.testclient import TestClient
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
import os
from datetime import timedelta

# Override the DATABASE_URL environment variable before importing our modules
os.environ["DATABASE_URL"] = "sqlite:///./test_auth.db"

from src.main import app
from src.models.database import Base, get_db, create_db_engine, get_id_column_type
from src.models.user import User
from src.api.auth import get_password_hash, verify_password
from src.core.config import settings

# Create test engine and session
test_engine = create_db_engine("sqlite:///./test_auth.db")
TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=test_engine)

# Create tables in the test database
Base.metadata.create_all(bind=test_engine)

def override_get_db():
    try:
        db = TestingSessionLocal()
        yield db
    finally:
        db.close()

app.dependency_overrides[get_db] = override_get_db

client = TestClient(app)

def test_password_hashing():
    """Test that password hashing and verification work correctly."""
    password = "test_password"
    hashed = get_password_hash(password)
    assert verify_password(password, hashed) == True
    assert verify_password("wrong_password", hashed) == False

def test_signup():
    """Test user signup functionality."""
    # For query parameters, we append them to the URL
    params = (
        "?email=test@example.com"
        "&password=test_password"
        "&software_experience=beginner"
        "&hardware_experience=intermediate"
        "&math_physics_level=advanced"
        "&learning_goals=Learn robotics"
    )
    response = client.post("/api/v1/auth/signup" + params)
    assert response.status_code == 201
    data = response.json()
    assert data["email"] == "test@example.com"
    assert "id" in data

def test_signup_duplicate_email():
    """Test that signup fails with duplicate email."""
    # First signup should succeed
    first_params = (
        "?email=duplicate@example.com"
        "&password=test_password"
        "&software_experience=beginner"
        "&hardware_experience=intermediate"
        "&math_physics_level=advanced"
        "&learning_goals=Learn robotics"
    )
    first_response = client.post("/api/v1/auth/signup" + first_params)
    assert first_response.status_code == 201

    # Second signup with same email should fail
    second_params = (
        "?email=duplicate@example.com"
        "&password=different_password"
        "&software_experience=beginner"
        "&hardware_experience=intermediate"
        "&math_physics_level=advanced"
        "&learning_goals=Learn robotics"
    )
    second_response = client.post("/api/v1/auth/signup" + second_params)
    assert second_response.status_code == 400

def test_login_success():
    """Test successful login."""
    # First, create a user
    signup_params = (
        "?email=login_test@example.com"
        "&password=test_password"
        "&software_experience=beginner"
        "&hardware_experience=intermediate"
        "&math_physics_level=advanced"
        "&learning_goals=Learn robotics"
    )
    signup_response = client.post("/api/v1/auth/signup" + signup_params)
    assert signup_response.status_code == 201

    # Then, try to log in
    response = client.post(
        "/api/v1/auth/login",
        data={
            "username": "login_test@example.com",
            "password": "test_password"
        }
    )
    assert response.status_code == 200
    data = response.json()
    assert "access_token" in data
    assert data["token_type"] == "bearer"

def test_login_failure():
    """Test login failure with wrong credentials."""
    response = client.post(
        "/api/v1/auth/login",
        data={
            "username": "nonexistent@example.com",
            "password": "wrong_password"
        }
    )
    assert response.status_code == 401

def test_get_current_user():
    """Test getting current user with valid token."""
    # Create and login user to get token
    signup_params = (
        "?email=current_user_test@example.com"
        "&password=test_password"
        "&software_experience=beginner"
        "&hardware_experience=intermediate"
        "&math_physics_level=advanced"
        "&learning_goals=Learn robotics"
    )
    signup_response = client.post("/api/v1/auth/signup" + signup_params)
    assert signup_response.status_code == 201

    login_response = client.post(
        "/api/v1/auth/login",
        data={
            "username": "current_user_test@example.com",
            "password": "test_password"
        }
    )
    assert login_response.status_code == 200
    token_data = login_response.json()
    token = token_data["access_token"]

    # Make a request to get current user
    response = client.get(
        "/api/v1/auth/me",
        headers={"Authorization": f"Bearer {token}"}
    )
    assert response.status_code == 200
    user_data = response.json()
    assert user_data["email"] == "current_user_test@example.com"

def test_get_current_user_invalid_token():
    """Test getting current user with invalid token."""
    response = client.get(
        "/api/v1/auth/me",
        headers={"Authorization": "Bearer invalid_token"}
    )
    assert response.status_code == 401