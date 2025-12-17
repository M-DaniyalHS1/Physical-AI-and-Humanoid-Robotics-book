"""
Test script to verify content API implementation
"""
from fastapi.testclient import TestClient
from src.main import app

client = TestClient(app)

print("Testing Content API endpoints...")

# Test the health endpoint
response = client.get('/health')
print(f"Health endpoint: Status {response.status_code}, Response: {response.json()}")

# Test the content endpoint
response = client.get('/api/v1/content/')
print(f"Content endpoint: Status {response.status_code}")

# Check if it's the placeholder response or real data
if response.status_code == 200:
    data = response.json()
    print(f"Content endpoint response type: {type(data)}")
    if isinstance(data, list):
        print(f"Response is a list with {len(data)} items")
    else:
        print(f"Response: {data}")
else:
    print(f"Content endpoint error: {response.json()}")

# Test the content by ID endpoint with an arbitrary ID
response = client.get('/api/v1/content/test-id')
print(f"Content by ID endpoint: Status {response.status_code}")
if response.status_code == 404:
    print("Correctly returns 404 for non-existent content")
elif response.status_code == 200:
    data = response.json()
    print(f"Content by ID response: {data}")
else:
    print(f"Content by ID error: {response.json()}")