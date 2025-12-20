"""
Test script to check for potential JSON parsing issues in the content model
"""
import json
from src.models.book_content import BookContent

# Create a mock book content instance to test the to_dict method
mock_content = BookContent(
    id="test-id",
    title="Test Title",
    module="ROS 2",
    chapter_number=1,
    content="Test content",
    authors='["Author 1", "Author 2"]',  # Valid JSON
    learning_objectives='["Objective 1", "Objective 2"]'  # Valid JSON
)

print("Testing to_dict with valid JSON:")
try:
    result = mock_content.to_dict()
    print(f"Success: {result['title']}")
    print(f"Authors: {result['authors']}")
    print(f"Learning Objectives: {result['learning_objectives']}")
except Exception as e:
    print(f"Error with valid JSON: {e}")

# Test with invalid JSON to see if it causes an exception
mock_content_invalid = BookContent(
    id="test-id-2",
    title="Test Title 2",
    module="ROS 2",
    chapter_number=1,
    content="Test content",
    authors='["Author 1", "Author 2"',  # Invalid JSON - missing closing bracket
    learning_objectives='["Objective 1", "Objective 2"]'  # Valid JSON
)

print("\nTesting to_dict with invalid JSON:")
try:
    result = mock_content_invalid.to_dict()
    print(f"Success: {result['title']}")
except Exception as e:
    print(f"Error with invalid JSON (expected): {e}")

# Test with None values
mock_content_none = BookContent(
    id="test-id-3",
    title="Test Title 3",
    module="ROS 2",
    chapter_number=1,
    content="Test content",
    authors=None,
    learning_objectives=None
)

print("\nTesting to_dict with None values:")
try:
    result = mock_content_none.to_dict()
    print(f"Success: {result['title']}")
    print(f"Authors: {result['authors']}")
    print(f"Learning Objectives: {result['learning_objectives']}")
except Exception as e:
    print(f"Error with None values: {e}")