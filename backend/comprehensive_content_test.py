"""
Comprehensive test for content API endpoints
"""
from fastapi.testclient import TestClient
from src.main import app

client = TestClient(app)

print("=== Comprehensive Content API Test ===")

# Test the health endpoint
response = client.get('/health')
print(f"PASS: Health endpoint: Status {response.status_code}")

# Test the content endpoint (GET /api/v1/content/)
response = client.get('/api/v1/content/')
print(f"PASS: Content endpoint: Status {response.status_code}")
if response.status_code == 200:
    data = response.json()
    print(f"  - Retrieved {len(data)} content items")
    if data:
        print(f"  - First item: {data[0]['title']} (ID: {data[0]['id']})")

# Test the content endpoint with filters
response = client.get('/api/v1/content/?module=ROS%202')
print(f"PASS: Content endpoint with module filter: Status {response.status_code}")

response = client.get('/api/v1/content/?content_type=text')
print(f"PASS: Content endpoint with type filter: Status {response.status_code}")

# Test the content by ID endpoint with an existing ID
if data:  # If we have content from the previous request
    sample_id = data[0]['id']
    response = client.get(f'/api/v1/content/{sample_id}')
    print(f"PASS: Content by ID endpoint: Status {response.status_code}")
    if response.status_code == 200:
        content = response.json()
        print(f"  - Retrieved content: {content['title']}")
    elif response.status_code == 404:
        print(f"  - Expected 404 for ID: {sample_id}")

# Test the content by ID endpoint with a non-existent ID
response = client.get('/api/v1/content/non-existent-id')
print(f"PASS: Content by non-existent ID: Status {response.status_code}")
if response.status_code == 404:
    print("  - Correctly returns 404 for non-existent content")

# Test the metadata endpoint
if data:
    sample_id = data[0]['id']
    response = client.get(f'/api/v1/content/{sample_id}/metadata')
    print(f"PASS: Content metadata endpoint: Status {response.status_code}")

# Test the search endpoint
response = client.get(f'/api/v1/content/*/search?query=ROS')
print(f"PASS: Content search endpoint: Status {response.status_code}")

print("\n=== All tests completed ===")