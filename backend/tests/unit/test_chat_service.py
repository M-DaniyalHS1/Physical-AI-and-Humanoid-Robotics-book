"""
Unit tests for the ChatService

This module contains unit tests for the ChatService class,
testing individual methods in isolation.
"""
import pytest
from unittest.mock import MagicMock, patch
from sqlalchemy.orm import Session
from datetime import datetime

from src.services.chat_service import ChatService
from src.models.chat_session import ChatSession
from src.models.chat_message import ChatMessage


class TestChatService:
    """Unit tests for ChatService methods"""

    def setup_method(self):
        """Set up test fixtures before each test method"""
        self.chat_service = ChatService()
        self.mock_db = MagicMock(spec=Session)

    def test_create_chat_session_success(self):
        """Test successful creation of a chat session"""
        user_id = "test-user-123"
        selected_text = "Some selected text"
        mode = "general"

        session = self.chat_service.create_chat_session(user_id, selected_text, mode)

        assert session.user_id == user_id
        assert session.selected_text == selected_text
        assert session.mode == mode
        assert session.is_active is True
        assert session.id is not None

    def test_create_chat_session_validation_user_id(self):
        """Test validation of user_id in create_chat_session"""
        # Test with None user_id
        with pytest.raises(ValueError, match="user_id must be a non-empty string"):
            self.chat_service.create_chat_session(None, "text", "general")

        # Test with empty string user_id
        with pytest.raises(ValueError, match="user_id must be a non-empty string"):
            self.chat_service.create_chat_session("", "text", "general")

        # Test with whitespace-only user_id
        with pytest.raises(ValueError, match="user_id must be a non-empty string"):
            self.chat_service.create_chat_session("   ", "text", "general")

    def test_create_chat_session_validation_mode(self):
        """Test validation of mode in create_chat_session"""
        # Test with invalid mode
        with pytest.raises(ValueError, match="mode must be one of"):
            self.chat_service.create_chat_session("user-123", "text", "invalid-mode")

    def test_create_chat_session_selected_text_required(self):
        """Test that selected_text is required in selected-text-only mode"""
        # Test with None selected_text in selected-text-only mode
        with pytest.raises(ValueError, match="selected_text must be a non-empty string"):
            self.chat_service.create_chat_session("user-123", None, "selected-text-only")

        # Test with empty string selected_text in selected-text-only mode
        with pytest.raises(ValueError, match="selected_text must be a non-empty string"):
            self.chat_service.create_chat_session("user-123", "", "selected-text-only")

        # Test with whitespace-only selected_text in selected-text-only mode
        with pytest.raises(ValueError, match="selected_text must be a non-empty string"):
            self.chat_service.create_chat_session("user-123", "   ", "selected-text-only")

    def test_create_chat_session_selected_text_optional(self):
        """Test that selected_text is optional in general mode"""
        # This should not raise an exception
        session = self.chat_service.create_chat_session("user-123", None, "general")
        assert session.user_id == "user-123"
        assert session.selected_text is None
        assert session.mode == "general"

    def test_add_message_to_session(self):
        """Test adding a message to a session"""
        session_id = "session-123"
        sender_type = "user"
        content = "Hello, this is a test message"

        # Mock the database operations
        mock_message = MagicMock(spec=ChatMessage)
        mock_message.id = "msg-123"
        mock_message.session_id = session_id
        mock_message.sender_type = sender_type
        mock_message.content = content
        mock_message.timestamp = datetime.now()
        mock_message.context_used = None

        # Patch uuid.uuid4 to return a predictable value
        with patch('src.services.chat_service.uuid.uuid4', return_value='msg-123'):
            result = self.chat_service.add_message_to_session(self.mock_db, session_id, sender_type, content)

        # Verify the database operations were called
        self.mock_db.add.assert_called_once()
        self.mock_db.commit.assert_called_once()
        self.mock_db.refresh.assert_called_once()

        # Verify the returned message
        assert result.id == "msg-123"
        assert result.session_id == session_id
        assert result.sender_type == sender_type
        assert result.content == content

    def test_get_session_messages_validation(self):
        """Test validation in get_session_messages"""
        # Test with invalid session_id
        with pytest.raises(ValueError, match="session_id must be a non-empty string"):
            self.chat_service.get_session_messages(self.mock_db, "", 0, 10)

        # Test with invalid skip
        with pytest.raises(ValueError, match="skip must be a non-negative integer"):
            self.chat_service.get_session_messages(self.mock_db, "session-123", -1, 10)

        # Test with invalid limit (negative)
        with pytest.raises(ValueError, match="limit must be a positive integer not exceeding 1000"):
            self.chat_service.get_session_messages(self.mock_db, "session-123", 0, -5)

        # Test with invalid limit (exceeds max)
        with pytest.raises(ValueError, match="limit must be a positive integer not exceeding 1000"):
            self.chat_service.get_session_messages(self.mock_db, "session-123", 0, 1001)

    def test_get_user_sessions_validation(self):
        """Test validation in get_user_sessions"""
        # Test with invalid user_id
        with pytest.raises(ValueError, match="user_id must be a non-empty string"):
            self.chat_service.get_user_sessions(self.mock_db, "", 0, 10)

        # Test with invalid skip
        with pytest.raises(ValueError, match="skip must be a non-negative integer"):
            self.chat_service.get_user_sessions(self.mock_db, "user-123", -1, 10)

        # Test with invalid limit (negative)
        with pytest.raises(ValueError, match="limit must be a positive integer not exceeding 1000"):
            self.chat_service.get_user_sessions(self.mock_db, "user-123", 0, -5)

        # Test with invalid limit (exceeds max)
        with pytest.raises(ValueError, match="limit must be a positive integer not exceeding 1000"):
            self.chat_service.get_user_sessions(self.mock_db, "user-123", 0, 1001)

    def test_process_user_message_validation(self):
        """Test validation in process_user_message"""
        # Test with invalid session_id
        with pytest.raises(ValueError, match="session_id must be a non-empty string"):
            self.chat_service.process_user_message(self.mock_db, "", "Hello")

        # Test with invalid user_message (empty)
        with pytest.raises(ValueError, match="user_message must be a non-empty string"):
            self.chat_service.process_user_message(self.mock_db, "session-123", "")

        # Test with invalid user_message (too long)
        long_message = "A" * 10001  # 10001 characters
        with pytest.raises(ValueError, match="user_message is too long"):
            self.chat_service.process_user_message(self.mock_db, "session-123", long_message)

    @patch('src.services.chat_service.ChatService.add_message_to_session')
    @patch('src.models.database.get_db')
    def test_process_user_message_session_not_found(self, mock_get_db, mock_add_message):
        """Test process_user_message when session is not found"""
        # Mock the database query to return None (session not found)
        self.mock_db.query.return_value.filter.return_value.first.return_value = None

        with pytest.raises(ValueError, match="Session test-session not found"):
            self.chat_service.process_user_message(self.mock_db, "test-session", "Hello")

    @patch('src.services.rag_service.rag_service.search_content_by_text_similarity')
    @patch('src.services.chat_service.ChatService.add_message_to_session')
    def test_process_user_message_with_rag_content(self, mock_add_message, mock_search):
        """Test process_user_message with RAG content"""
        # Mock the database query to return a session
        mock_session = MagicMock(spec=ChatSession)
        mock_session.id = "session-123"
        mock_session.user_id = "user-123"
        mock_session.selected_text = None
        mock_session.mode = "general"
        mock_session.is_active = True
        self.mock_db.query.return_value.filter.return_value.first.return_value = mock_session

        # Mock RAG search results
        mock_rag_result = [
            {
                "id": "content-1",
                "content_text": "This is relevant textbook content.",
                "metadata": {"topic": "ROS 2"},
                "score": 0.95
            }
        ]
        mock_search.return_value = mock_rag_result

        # Mock the add_message_to_session to return a mock message
        mock_message = MagicMock(spec=ChatMessage)
        mock_message.id = "msg-123"
        mock_message.session_id = "session-123"
        mock_message.sender_type = "ai"
        mock_message.content = "AI response based on context"
        mock_message.timestamp = datetime.now()
        mock_message.context_used = str(mock_rag_result[:1])
        mock_add_message.return_value = mock_message

        # Mock the OpenAI client to avoid actual API calls
        with patch.object(self.chat_service, 'openai_client') as mock_openai:
            mock_response = MagicMock()
            mock_response.choices = [MagicMock()]
            mock_response.choices[0].message.content = "AI response based on context"
            mock_openai.chat.completions.create.return_value = mock_response
            mock_embedding_response = MagicMock()
            mock_embedding_response.data = [MagicMock()]
            mock_embedding_response.data[0].embedding = [0.1] * 1536
            mock_openai.embeddings.create.return_value = mock_embedding_response

            result = self.chat_service.process_user_message(self.mock_db, "session-123", "Tell me about ROS 2")

        # Verify the result
        assert result == "AI response based on context"

    @patch('src.services.rag_service.rag_service.search_content_by_text_similarity')
    @patch('src.services.chat_service.ChatService.add_message_to_session')
    def test_process_user_message_selected_text_mode(self, mock_add_message, mock_search):
        """Test process_user_message in selected-text-only mode"""
        # Mock the database query to return a session in selected-text-only mode
        mock_session = MagicMock(spec=ChatSession)
        mock_session.id = "session-123"
        mock_session.user_id = "user-123"
        mock_session.selected_text = "The control algorithms in humanoid robotics require precise timing."
        mock_session.mode = "selected-text-only"
        mock_session.is_active = True
        self.mock_db.query.return_value.filter.return_value.first.return_value = mock_session

        # Mock RAG search results
        mock_rag_result = [
            {
                "id": "content-1",
                "content_text": "Control algorithms in humanoid robotics must account for dynamic movements.",
                "metadata": {"topic": "Control Algorithms"},
                "score": 0.92
            }
        ]
        mock_search.return_value = mock_rag_result

        # Mock the add_message_to_session to return a mock message
        mock_message = MagicMock(spec=ChatMessage)
        mock_message.id = "msg-123"
        mock_message.session_id = "session-123"
        mock_message.sender_type = "ai"
        mock_message.content = "AI response focused on selected text"
        mock_message.timestamp = datetime.now()
        mock_message.context_used = str(mock_rag_result[:1])
        mock_add_message.return_value = mock_message

        # Mock the OpenAI client to avoid actual API calls
        with patch.object(self.chat_service, 'openai_client') as mock_openai:
            mock_response = MagicMock()
            mock_response.choices = [MagicMock()]
            mock_response.choices[0].message.content = "AI response focused on selected text"
            mock_openai.chat.completions.create.return_value = mock_response
            mock_embedding_response = MagicMock()
            mock_embedding_response.data = [MagicMock()]
            mock_embedding_response.data[0].embedding = [0.1] * 1536
            mock_openai.embeddings.create.return_value = mock_embedding_response

            result = self.chat_service.process_user_message(self.mock_db, "session-123", "Explain more about control algorithms")

        # Verify the result
        assert result == "AI response focused on selected text"

    def test_close_session_success(self):
        """Test successful closing of a session"""
        session_id = "session-123"

        # Mock the session query
        mock_session = MagicMock(spec=ChatSession)
        mock_session.id = session_id
        mock_session.is_active = True
        self.mock_db.query.return_value.filter.return_value.first.return_value = mock_session

        result = self.chat_service.close_session(self.mock_db, session_id)

        # Verify the session was updated
        assert mock_session.is_active is False
        assert mock_session.session_end is not None
        self.mock_db.commit.assert_called_once()
        assert result is True

    def test_close_session_not_found(self):
        """Test closing a session that doesn't exist"""
        # Mock the session query to return None
        self.mock_db.query.return_value.filter.return_value.first.return_value = None

        result = self.chat_service.close_session(self.mock_db, "non-existent-session")

        assert result is False