"""
Unit tests for the selected-text-only mode in ChatService

This module contains unit tests specifically for the selected-text-only mode
functionality in the ChatService class.
"""
import pytest
import sys
import os
from unittest.mock import MagicMock, patch
from sqlalchemy.orm import Session
from datetime import datetime

# Add the backend/src directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

# Import using absolute paths to avoid relative import issues
from src.services.chat_service import ChatService
from src.models.chat_session import ChatSession
from src.models.chat_message import ChatMessage


class TestSelectedTextMode:
    """Unit tests for selected-text-only mode in ChatService"""

    def setup_method(self):
        """Set up test fixtures before each test method"""
        self.chat_service = ChatService()
        self.mock_db = MagicMock(spec=Session)

    def test_process_user_message_selected_text_mode_with_related_content(self):
        """Test process_user_message in selected-text-only mode with related content"""
        # Mock the database query to return a session in selected-text-only mode
        mock_session = MagicMock(spec=ChatSession)
        mock_session.id = "session-123"
        mock_session.user_id = "user-123"
        mock_session.selected_text = "The control algorithms in humanoid robotics require precise timing."
        mock_session.mode = "selected-text-only"
        mock_session.is_active = True
        self.mock_db.query.return_value.filter.return_value.first.return_value = mock_session

        # Mock RAG search results with content related to the selected text
        mock_rag_result = [
            {
                "id": "content-1",
                "content_text": "Control algorithms in humanoid robotics must account for dynamic movements and precise timing.",
                "metadata": {"topic": "Control Algorithms"},
                "score": 0.92
            },
            {
                "id": "content-2", 
                "content_text": "Feedback mechanisms are crucial for humanoid robotics control systems.",
                "metadata": {"topic": "Feedback Systems"},
                "score": 0.85
            }
        ]

        # Mock the RAG service search method
        with patch.object(self.chat_service.rag_service, 'search_content_by_text_similarity', return_value=mock_rag_result):
            # Mock the add_message_to_session to return a mock message
            mock_message = MagicMock(spec=ChatMessage)
            mock_message.id = "msg-123"
            mock_message.session_id = "session-123"
            mock_message.sender_type = "ai"
            mock_message.content = "AI response focused on selected text"
            mock_message.timestamp = datetime.now()
            mock_message.context_used = str(mock_rag_result[:1])
            
            with patch.object(self.chat_service, 'add_message_to_session', return_value=mock_message):
                # Mock the OpenAI client to avoid actual API calls
                with patch.object(self.chat_service, 'openai_client') as mock_openai:
                    mock_response = MagicMock()
                    mock_response.choices = [MagicMock()]
                    mock_response.choices[0].message.content = "AI response focused on selected text about control algorithms and precise timing."
                    mock_openai.chat.completions.create.return_value = mock_response
                    mock_embedding_response = MagicMock()
                    mock_embedding_response.data = [MagicMock()]
                    mock_embedding_response.data[0].embedding = [0.1] * 1536
                    mock_openai.embeddings.create.return_value = mock_embedding_response

                    result = self.chat_service.process_user_message(self.mock_db, "session-123", "Explain more about control algorithms")

                    # Verify the result
                    assert "control algorithms" in result.lower()
                    assert "timing" in result.lower()

    def test_process_user_message_selected_text_mode_no_related_content(self):
        """Test process_user_message in selected-text-only mode with no related content"""
        # Mock the database query to return a session in selected-text-only mode
        mock_session = MagicMock(spec=ChatSession)
        mock_session.id = "session-456"
        mock_session.user_id = "user-456"
        mock_session.selected_text = "The PID controller is essential for motor control in robotics."
        mock_session.mode = "selected-text-only"
        mock_session.is_active = True
        self.mock_db.query.return_value.filter.return_value.first.return_value = mock_session

        # Mock RAG search results with content not related to the selected text
        mock_rag_result = [
            {
                "id": "content-1",
                "content_text": "The history of computer science dates back to the early 1900s.",
                "metadata": {"topic": "Computer History"},
                "score": 0.88
            },
            {
                "id": "content-2", 
                "content_text": "Modern web frameworks provide efficient ways to build applications.",
                "metadata": {"topic": "Web Development"},
                "score": 0.75
            }
        ]

        # Mock the RAG service search method
        with patch.object(self.chat_service.rag_service, 'search_content_by_text_similarity', return_value=mock_rag_result):
            # Mock the add_message_to_session to return a mock message
            mock_message = MagicMock(spec=ChatMessage)
            mock_message.id = "msg-456"
            mock_message.session_id = "session-456"
            mock_message.sender_type = "ai"
            mock_message.content = "AI response based only on selected text"
            mock_message.timestamp = datetime.now()
            mock_message.context_used = str(mock_rag_result[:1])
            
            with patch.object(self.chat_service, 'add_message_to_session', return_value=mock_message):
                # Mock the OpenAI client to avoid actual API calls
                with patch.object(self.chat_service, 'openai_client') as mock_openai:
                    mock_response = MagicMock()
                    mock_response.choices = [MagicMock()]
                    mock_response.choices[0].message.content = "Based only on the selected text about PID controllers: The PID controller is essential for motor control in robotics."
                    mock_openai.chat.completions.create.return_value = mock_response
                    mock_embedding_response = MagicMock()
                    mock_embedding_response.data = [MagicMock()]
                    mock_embedding_response.data[0].embedding = [0.1] * 1536
                    mock_openai.embeddings.create.return_value = mock_embedding_response

                    result = self.chat_service.process_user_message(self.mock_db, "session-456", "What is a PID controller?")

                    # Verify the result is based on the selected text
                    assert "pid controller" in result.lower()
                    assert "motor control" in result.lower()

    def test_process_user_message_general_mode_unchanged(self):
        """Test that general mode still works as before"""
        # Mock the database query to return a session in general mode
        mock_session = MagicMock(spec=ChatSession)
        mock_session.id = "session-789"
        mock_session.user_id = "user-789"
        mock_session.selected_text = None
        mock_session.mode = "general"
        mock_session.is_active = True
        self.mock_db.query.return_value.filter.return_value.first.return_value = mock_session

        # Mock RAG search results
        mock_rag_result = [
            {
                "id": "content-1",
                "content_text": "ROS 2 provides a flexible framework for robotics applications.",
                "metadata": {"topic": "ROS 2"},
                "score": 0.95
            }
        ]

        # Mock the RAG service search method
        with patch.object(self.chat_service.rag_service, 'search_content_by_text_similarity', return_value=mock_rag_result):
            # Mock the add_message_to_session to return a mock message
            mock_message = MagicMock(spec=ChatMessage)
            mock_message.id = "msg-789"
            mock_message.session_id = "session-789"
            mock_message.sender_type = "ai"
            mock_message.content = "AI response with general information"
            mock_message.timestamp = datetime.now()
            mock_message.context_used = str(mock_rag_result[:1])
            
            with patch.object(self.chat_service, 'add_message_to_session', return_value=mock_message):
                # Mock the OpenAI client to avoid actual API calls
                with patch.object(self.chat_service, 'openai_client') as mock_openai:
                    mock_response = MagicMock()
                    mock_response.choices = [MagicMock()]
                    mock_response.choices[0].message.content = "ROS 2 provides a flexible framework for robotics applications."
                    mock_openai.chat.completions.create.return_value = mock_response
                    mock_embedding_response = MagicMock()
                    mock_embedding_response.data = [MagicMock()]
                    mock_embedding_response.data[0].embedding = [0.1] * 1536
                    mock_openai.embeddings.create.return_value = mock_embedding_response

                    result = self.chat_service.process_user_message(self.mock_db, "session-789", "What is ROS 2?")

                    # Verify the result includes the RAG content
                    assert "ros 2" in result.lower()
                    assert "framework" in result.lower()