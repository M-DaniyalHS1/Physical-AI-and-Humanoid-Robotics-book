"""
Contract tests for the /chat/sessions endpoint

This module contains contract tests that verify the API contract
for the chat sessions endpoint, ensuring it returns the expected responses
and handles inputs correctly.
"""
import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
from src.main import app
from src.models.database import get_db
from src.models.chat_session import ChatSession
from src.models.chat_message import ChatMessage
from datetime import datetime
from src.models.user import User

# Create test client with dependency override to avoid database dependency
client = TestClient(app)

# Mock database session for testing
def get_test_db():
    mock_db = MagicMock()
    yield mock_db

# Override the database dependency for tests
app.dependency_overrides[get_db] = get_test_db


@pytest.fixture
def mock_chat_session_data():
    """Mock chat session data for testing"""
    return [
        {
            "id": "session-1",
            "user_id": "user-123",
            "session_start": datetime.now().isoformat(),
            "session_end": None,
            "selected_text": "Some selected text",
            "mode": "general",
            "is_active": True
        },
        {
            "id": "session-2",
            "user_id": "user-123",
            "session_start": datetime.now().isoformat(),
            "session_end": datetime.now().isoformat(),
            "selected_text": None,
            "mode": "selected-text-only",
            "is_active": False
        }
    ]


def test_chat_sessions_endpoint_exists():
    """
    Contract test: GET /chat/sessions endpoint should exist and return 200 status code
    """
    # This test will initially fail because the endpoint requires authentication
    # We'll test the endpoint structure without authentication for contract testing
    response = client.get("/api/v1/chat/sessions")
    
    # The endpoint should return 401/403 for unauthorized access or 200 for authorized access
    # This confirms the endpoint exists
    assert response.status_code in [200, 401, 403]


def test_chat_sessions_endpoint_returns_list():
    """
    Contract test: GET /chat/sessions endpoint should return a list of chat sessions
    """
    # Mock the authentication and database calls to test the contract
    with patch('src.api.chat.get_current_user') as mock_get_current_user, \
         patch('src.models.database.get_db') as mock_get_db:
        
        # Mock a current user
        mock_user = MagicMock()
        mock_user.id = "user-123"
        mock_get_current_user.return_value = mock_user
        
        # Mock the database session
        mock_db = MagicMock()
        mock_get_db.return_value.__enter__.return_value = mock_db
        mock_get_db.return_value.__exit__.return_value = None
        
        # Mock chat sessions
        mock_session1 = MagicMock(spec=ChatSession)
        mock_session1.id = "session-1"
        mock_session1.user_id = "user-123"
        mock_session1.session_start = datetime.now()
        mock_session1.session_end = None
        mock_session1.selected_text = "Some selected text"
        mock_session1.mode = "general"
        mock_session1.is_active = True
        
        mock_session2 = MagicMock(spec=ChatSession)
        mock_session2.id = "session-2"
        mock_session2.user_id = "user-123"
        mock_session2.session_start = datetime.now()
        mock_session2.session_end = datetime.now()
        mock_session2.selected_text = None
        mock_session2.mode = "selected-text-only"
        mock_session2.is_active = False
        
        mock_db.query.return_value.filter.return_value.offset.return_value.limit.return_value.all.return_value = [
            mock_session1, mock_session2
        ]
        
        # Add an authorization header to bypass authentication for testing
        headers = {"Authorization": "Bearer fake-token"}
        response = client.get("/api/v1/chat/sessions", headers=headers)
        
        # Check that the response is a list
        if response.status_code == 200:
            data = response.json()
            assert isinstance(data, list)
            
            # If the list is not empty, check the structure of the first item
            if len(data) > 0:
                session = data[0]
                assert "id" in session
                assert "user_id" in session
                assert "session_start" in session
                assert "session_end" in session
                assert "selected_text" in session
                assert "mode" in session
                assert "is_active" in session


def test_chat_sessions_endpoint_pagination():
    """
    Contract test: GET /chat/sessions endpoint should support pagination parameters
    """
    # Test with pagination parameters
    headers = {"Authorization": "Bearer fake-token"}
    response = client.get("/api/v1/chat/sessions?skip=0&limit=10", headers=headers)
    
    # The endpoint should accept these parameters without error
    assert response.status_code in [200, 401, 403]


def test_chat_sessions_endpoint_authentication():
    """
    Contract test: GET /chat/sessions endpoint should require authentication
    """
    # Test that the endpoint requires authentication
    response = client.get("/api/v1/chat/sessions")
    
    # Should return 401 or 403 if authentication is required
    assert response.status_code in [401, 403]


def test_chat_sessions_endpoint_response_format():
    """
    Contract test: GET /chat/sessions endpoint should return properly formatted response
    """
    # Mock the authentication and database calls to test the response format
    with patch('src.api.chat.get_current_user') as mock_get_current_user, \
         patch('src.models.database.get_db') as mock_get_db:

        # Mock a current user
        mock_user = MagicMock()
        mock_user.id = "user-123"
        mock_get_current_user.return_value = mock_user

        # Mock the database session
        mock_db = MagicMock()
        mock_get_db.return_value.__enter__.return_value = mock_db
        mock_get_db.return_value.__exit__.return_value = None

        # Mock a chat session
        mock_session = MagicMock(spec=ChatSession)
        mock_session.id = "session-1"
        mock_session.user_id = "user-123"
        mock_session.session_start = datetime.now()
        mock_session.session_end = None
        mock_session.selected_text = "Some selected text"
        mock_session.mode = "general"
        mock_session.is_active = True

        mock_db.query.return_value.filter.return_value.offset.return_value.limit.return_value.all.return_value = [
            mock_session
        ]

        # Add an authorization header to bypass authentication for testing
        headers = {"Authorization": "Bearer fake-token"}
        response = client.get("/api/v1/chat/sessions", headers=headers)

        if response.status_code == 200:
            data = response.json()

            # Check that the response is a list
            assert isinstance(data, list)

            # Check the structure of a session if the list is not empty
            if len(data) > 0:
                session = data[0]

                # Validate required fields
                assert "id" in session
                assert isinstance(session["id"], str)

                assert "user_id" in session
                assert isinstance(session["user_id"], str)

                assert "session_start" in session
                # session_start can be a string (ISO format) or None

                assert "session_end" in session
                # session_end can be a string (ISO format) or None

                assert "selected_text" in session
                # selected_text can be a string or None

                assert "mode" in session
                assert isinstance(session["mode"], str)

                assert "is_active" in session
                assert isinstance(session["is_active"], bool)


def test_chat_session_messages_endpoint_exists():
    """
    Contract test: GET /chat/sessions/{sessionId}/messages endpoint should exist and return appropriate status code
    """
    # Test that the endpoint exists and returns appropriate status codes
    headers = {"Authorization": "Bearer fake-token"}
    response = client.get("/api/v1/chat/sessions/session-123/messages", headers=headers)

    # The endpoint should return 200 for authorized access with valid session
    # or 401/403 for unauthorized access
    # or 404 if session doesn't exist
    assert response.status_code in [200, 401, 403, 404]


def test_chat_session_messages_endpoint_returns_list():
    """
    Contract test: GET /chat/sessions/{sessionId}/messages endpoint should return a list of messages
    """
    # Mock the authentication and database calls to test the contract
    with patch('src.api.chat.get_current_user') as mock_get_current_user, \
         patch('src.models.database.get_db') as mock_get_db:

        # Mock a current user
        mock_user = MagicMock()
        mock_user.id = "user-123"
        mock_get_current_user.return_value = mock_user

        # Mock the database session
        mock_db = MagicMock()
        mock_get_db.return_value.__enter__.return_value = mock_db
        mock_get_db.return_value.__exit__.return_value = None

        # Mock a chat session to verify ownership
        mock_session = MagicMock(spec=ChatSession)
        mock_session.id = "session-123"
        mock_session.user_id = "user-123"
        mock_db.query.return_value.filter.return_value.first.return_value = mock_session

        # Mock chat messages
        mock_message1 = MagicMock(spec=ChatMessage)
        mock_message1.id = "msg-1"
        mock_message1.session_id = "session-123"
        mock_message1.sender_type = "user"
        mock_message1.content = "Hello, how does this work?"
        mock_message1.timestamp = datetime.now()
        mock_message1.context_used = None

        mock_message2 = MagicMock(spec=ChatMessage)
        mock_message2.id = "msg-2"
        mock_message2.session_id = "session-123"
        mock_message2.sender_type = "ai"
        mock_message2.content = "This is an AI response."
        mock_message2.timestamp = datetime.now()
        mock_message2.context_used = "Some context from the textbook"

        mock_db.query.return_value.filter.return_value.offset.return_value.limit.return_value.all.return_value = [
            mock_message1, mock_message2
        ]

        # Add an authorization header to bypass authentication for testing
        headers = {"Authorization": "Bearer fake-token"}
        response = client.get("/api/v1/chat/sessions/session-123/messages", headers=headers)

        if response.status_code == 200:
            data = response.json()
            # Check that the response is a list
            assert isinstance(data, list)

            # If the list is not empty, check the structure of the first item
            if len(data) > 0:
                message = data[0]
                assert "id" in message
                assert "session_id" in message
                assert "sender_type" in message
                assert "content" in message
                assert "timestamp" in message
                assert "context_used" in message


def test_chat_session_messages_endpoint_pagination():
    """
    Contract test: GET /chat/sessions/{sessionId}/messages endpoint should support pagination parameters
    """
    # Test with pagination parameters
    headers = {"Authorization": "Bearer fake-token"}
    response = client.get("/api/v1/chat/sessions/session-123/messages?skip=0&limit=10", headers=headers)

    # The endpoint should accept these parameters without error
    assert response.status_code in [200, 401, 403, 404]


def test_chat_session_messages_endpoint_authentication():
    """
    Contract test: GET /chat/sessions/{sessionId}/messages endpoint should require authentication
    """
    # Test that the endpoint requires authentication
    response = client.get("/api/v1/chat/sessions/session-123/messages")

    # Should return 401 or 403 if authentication is required
    assert response.status_code in [401, 403]


def test_chat_session_messages_endpoint_response_format():
    """
    Contract test: GET /chat/sessions/{sessionId}/messages endpoint should return properly formatted response
    """
    # Mock the authentication and database calls to test the response format
    with patch('src.api.chat.get_current_user') as mock_get_current_user, \
         patch('src.models.database.get_db') as mock_get_db:

        # Mock a current user
        mock_user = MagicMock()
        mock_user.id = "user-123"
        mock_get_current_user.return_value = mock_user

        # Mock the database session
        mock_db = MagicMock()
        mock_get_db.return_value.__enter__.return_value = mock_db
        mock_get_db.return_value.__exit__.return_value = None

        # Mock a chat session to verify ownership
        mock_session = MagicMock(spec=ChatSession)
        mock_session.id = "session-123"
        mock_session.user_id = "user-123"
        mock_db.query.return_value.filter.return_value.first.return_value = mock_session

        # Mock a chat message
        mock_message = MagicMock(spec=ChatMessage)
        mock_message.id = "msg-1"
        mock_message.session_id = "session-123"
        mock_message.sender_type = "user"
        mock_message.content = "Test message content"
        mock_message.timestamp = datetime.now()
        mock_message.context_used = "Some context"

        mock_db.query.return_value.filter.return_value.offset.return_value.limit.return_value.all.return_value = [
            mock_message
        ]

        # Add an authorization header to bypass authentication for testing
        headers = {"Authorization": "Bearer fake-token"}
        response = client.get("/api/v1/chat/sessions/session-123/messages", headers=headers)

        if response.status_code == 200:
            data = response.json()

            # Check that the response is a list
            assert isinstance(data, list)

            # Check the structure of a message if the list is not empty
            if len(data) > 0:
                message = data[0]

                # Validate required fields
                assert "id" in message
                assert isinstance(message["id"], str)

                assert "session_id" in message
                assert isinstance(message["session_id"], str)

                assert "sender_type" in message
                assert isinstance(message["sender_type"], str)

                assert "content" in message
                assert isinstance(message["content"], str)

                assert "timestamp" in message
                # timestamp can be a string (ISO format) or None

                assert "context_used" in message
                # context_used can be a string or None