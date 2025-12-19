"""
Simple test to verify the health check endpoints work correctly after our fixes
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from backend.src.main import app
from fastapi.testclient import TestClient

def test_health_check():
    client = TestClient(app)
    response = client.get("/health")
    assert response.status_code == 200
    data = response.json()
    assert "status" in data
    assert data["status"] == "healthy"
    assert "version" in data
    assert "timestamp" in data
    assert "env_validated" in data
    print("Health check endpoint works correctly")

def test_readiness_check():
    client = TestClient(app)
    response = client.get("/ready")
    assert response.status_code in [200, 503]  # Might be 503 if environment is invalid
    print("Readiness check endpoint is accessible")

    if response.status_code == 200:
        data = response.json()
        assert "status" in data
        assert data["status"] == "ready"
        print("Application is ready")
    elif response.status_code == 503:
        data = response.json()
        assert "status" in data
        assert data["status"] == "not_ready"
        print("Application is not ready as expected (due to environment validation)")

if __name__ == "__main__":
    print("Testing health check endpoints after fixes...")
    test_health_check()
    test_readiness_check()
    print("All health check tests passed!")