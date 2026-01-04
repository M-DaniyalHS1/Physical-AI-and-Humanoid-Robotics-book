"""
Integration tests for the selected-text-only mode functionality

This module tests the selected-text-only answering mode in the chat service,
verifying that the AI focuses exclusively on the selected text when in this mode.
"""
import pytest
from unittest.mock import patch, MagicMock
from sqlalchemy.orm import Session
from datetime import datetime
from src.services.chat_service import chat_service
from src.models.chat_session import ChatSession
from src.models.chat_message import ChatMessage
from src.models.user import User


def test_process_user_message_selected_text_only_mode():
    """
    Integration test: Verify that the selected-text-only mode works correctly
    Tests that when a session is in 'selected-text-only' mode, the AI focuses 
    exclusively on the selected text provided during session creation
    """
    # Mock the database session
    mock_db = MagicMock(spec=Session)

    # Create a mock session in selected-text-only mode with selected text
    mock_session = MagicMock(spec=ChatSession)
    mock_session.id = "test-selected-session-123"
    mock_session.user_id = "test-user-123"
    mock_session.session_start = datetime.now()
    mock_session.session_end = None
    mock_session.selected_text = "The control algorithms in humanoid robotics require precise timing and feedback mechanisms."
    mock_session.mode = "selected-text-only"
    mock_session.is_active = True

    # Configure the database query to return the mock session
    mock_db.query.return_value.filter.return_value.first.return_value = mock_session

    # Mock the RAG service search to return some related content
    with patch('src.services.rag_service.rag_service.search_content_by_text_similarity') as mock_search:
        mock_search.return_value = [
            {
                "id": "content-1",
                "content_text": "Control algorithms in humanoid robotics must account for the dynamic nature of bipedal locomotion. Feedback mechanisms are crucial for maintaining balance and executing complex movements.",
                "metadata": {"topic": "Control Algorithms", "module": "Humanoid Control"},
                "score": 0.92
            }
        ]

        # Mock the OpenAI client
        with patch.object(chat_service, 'openai_client') as mock_openai:
            mock_response = MagicMock()
            mock_response.choices = [MagicMock()]
            mock_response.choices[0].message.content = "Based on the selected text, the feedback mechanisms are crucial for maintaining balance in humanoid robotics."
            mock_openai.chat.completions.create.return_value = mock_response
            mock_embedding_response = MagicMock()
            mock_embedding_response.data = [MagicMock()]
            mock_embedding_response.data[0].embedding = [0.1] * 1536
            mock_openai.embeddings.create.return_value = mock_embedding_response

            # Mock the message addition
            with patch.object(chat_service, 'add_message_to_session') as mock_add_message:
                mock_user_message = MagicMock(spec=ChatMessage)
                mock_user_message.id = "user-msg-1"
                mock_user_message.session_id = "test-selected-session-123"
                mock_user_message.sender_type = "user"
                mock_user_message.content = "Can you explain more about the feedback mechanisms?"
                mock_user_message.timestamp = datetime.now()
                mock_user_message.context_used = None
                mock_add_message.return_value = mock_user_message

                # Call the selected-text-only processing method
                response = chat_service.process_user_message_selected_text_only(
                    mock_db,
                    "test-selected-session-123",
                    "Can you explain more about the feedback mechanisms?"
                )

                # Verify the response
                assert "feedback mechanisms" in response.lower()
                assert "selected text" in response.lower() or "control algorithms" in response.lower()

                # Verify that the RAG search was called with the combined query
                mock_search.assert_called_once()
                # Check that the search method was called with the expected parameters
                call_args = mock_search.call_args
                # The search_content_by_text_similarity method has named parameters
                assert "query_text" in call_args.kwargs
                assert "Can you explain more about the feedback mechanisms?" in call_args.kwargs["query_text"]


def test_process_user_message_selected_text_only_validation():
    """
    Integration test: Verify validation for selected-text-only mode
    Tests that the method properly validates that the session is in the correct mode
    and has selected text before processing
    """
    # Mock the database session
    mock_db = MagicMock(spec=Session)

    # Create a mock session in GENERAL mode (not selected-text-only)
    mock_session = MagicMock(spec=ChatSession)
    mock_session.id = "test-general-session-456"
    mock_session.user_id = "test-user-456"
    mock_session.session_start = datetime.now()
    mock_session.session_end = None
    mock_session.selected_text = "Some selected text"
    mock_session.mode = "general"  # This is NOT selected-text-only mode
    mock_session.is_active = True

    # Configure the database query to return the mock session
    mock_db.query.return_value.filter.return_value.first.return_value = mock_session

    # Try to call the selected-text-only processing method on a general mode session
    with pytest.raises(ValueError, match="is not in selected-text-only mode"):
        chat_service.process_user_message_selected_text_only(
            mock_db,
            "test-general-session-456",
            "Any question here"
        )

    # Now test with a selected-text-only session but without selected text
    mock_session_no_text = MagicMock(spec=ChatSession)
    mock_session_no_text.id = "test-no-text-session-789"
    mock_session_no_text.user_id = "test-user-789"
    mock_session_no_text.session_start = datetime.now()
    mock_session_no_text.session_end = None
    mock_session_no_text.selected_text = None  # No selected text
    mock_session_no_text.mode = "selected-text-only"  # But in the right mode
    mock_session_no_text.is_active = True

    # Configure the database query to return the mock session without text
    mock_db.query.return_value.filter.return_value.first.return_value = mock_session_no_text

    # Try to call the selected-text-only processing method without selected text
    with pytest.raises(ValueError, match="does not have selected text for selected-text-only mode"):
        chat_service.process_user_message_selected_text_only(
            mock_db,
            "test-no-text-session-789",
            "Any question here"
        )


def test_process_user_message_selected_text_only_fallback():
    """
    Integration test: Verify fallback behavior when OpenAI is not available
    Tests that the method works correctly even when the OpenAI API is not configured
    """
    # Temporarily set openai_client to None to simulate unavailability
    original_client = chat_service.openai_client
    chat_service.openai_client = None

    try:
        # Mock the database session
        mock_db = MagicMock(spec=Session)

        # Create a mock session in selected-text-only mode with selected text
        mock_session = MagicMock(spec=ChatSession)
        mock_session.id = "test-fallback-session-101"
        mock_session.user_id = "test-user-101"
        mock_session.session_start = datetime.now()
        mock_session.session_end = None
        mock_session.selected_text = "The PID controller is essential for motor control in robotics."
        mock_session.mode = "selected-text-only"
        mock_session.is_active = True

        # Configure the database query to return the mock session
        mock_db.query.return_value.filter.return_value.first.return_value = mock_session

        # Mock the RAG service search to return some related content
        with patch('src.services.rag_service.rag_service.search_content_by_text_similarity') as mock_search:
            mock_search.return_value = [
                {
                    "id": "content-2",
                    "content_text": "PID controllers adjust the motor output based on the difference between the desired and actual positions.",
                    "metadata": {"topic": "PID Control", "module": "Motor Control"},
                    "score": 0.88
                }
            ]

            # Mock the message addition
            with patch.object(chat_service, 'add_message_to_session') as mock_add_message:
                mock_user_message = MagicMock(spec=ChatMessage)
                mock_user_message.id = "user-msg-2"
                mock_user_message.session_id = "test-fallback-session-101"
                mock_user_message.sender_type = "user"
                mock_user_message.content = "How does PID control work?"
                mock_user_message.timestamp = datetime.now()
                mock_user_message.context_used = None
                mock_add_message.return_value = mock_user_message

                # Call the selected-text-only processing method
                response = chat_service.process_user_message_selected_text_only(
                    mock_db,
                    "test-fallback-session-101",
                    "How does PID control work?"
                )

                # Verify the response is a fallback response
                assert "[This is a mock response as OpenAI API is not configured]" in response
                assert "PID controller" in response  # Should contain the selected text

    finally:
        # Restore the original client
        chat_service.openai_client = original_client


def test_create_session_with_selected_text_mode():
    """
    Integration test: Verify that sessions can be created with selected-text-only mode
    Tests that the create_chat_session method properly handles the selected-text-only mode
    """
    # Test creating a session in selected-text-only mode
    session = chat_service.create_chat_session(
        user_id="test-user-202",
        selected_text="The Kalman filter is used for sensor fusion in robotics.",
        mode="selected-text-only"
    )

    # Verify the session properties
    assert session.mode == "selected-text-only"
    assert session.selected_text == "The Kalman filter is used for sensor fusion in robotics."
    assert session.user_id == "test-user-202"
    assert session.is_active is True
    assert session.id is not None

    # Test that creating a session in selected-text-only mode without selected text raises an error
    with pytest.raises(ValueError, match="selected_text must be a non-empty string when mode is 'selected-text-only'"):
        chat_service.create_chat_session(
            user_id="test-user-203",
            selected_text=None,
            mode="selected-text-only"
        )

    with pytest.raises(ValueError, match="selected_text must be a non-empty string when mode is 'selected-text-only'"):
        chat_service.create_chat_session(
            user_id="test-user-204",
            selected_text="",
            mode="selected-text-only"
        )

    with pytest.raises(ValueError, match="selected_text must be a non-empty string when mode is 'selected-text-only'"):
        chat_service.create_chat_session(
            user_id="test-user-205",
            selected_text="   ",  # whitespace-only
            mode="selected-text-only"
        )


def test_process_user_message_uses_selected_text_mode_logic():
    """
    Integration test: Verify that the general process_user_message method
    properly handles selected-text-only mode sessions
    """
    # Mock the database session
    mock_db = MagicMock(spec=Session)

    # Create a mock session in selected-text-only mode with selected text
    mock_session = MagicMock(spec=ChatSession)
    mock_session.id = "test-general-method-303"
    mock_session.user_id = "test-user-303"
    mock_session.session_start = datetime.now()
    mock_session.session_end = None
    mock_session.selected_text = "The inverse kinematics problem involves calculating joint angles from end-effector position."
    mock_session.mode = "selected-text-only"
    mock_session.is_active = True

    # Configure the database query to return the mock session
    mock_db.query.return_value.filter.return_value.first.return_value = mock_session

    # Mock the RAG service search
    with patch('src.services.rag_service.rag_service.search_content_by_text_similarity') as mock_search:
        mock_search.return_value = [
            {
                "id": "content-3",
                "content_text": "Inverse kinematics is essential for robotic arm control. It determines the joint parameters needed to position the end-effector at a desired location.",
                "metadata": {"topic": "Inverse Kinematics", "module": "Kinematics"},
                "score": 0.94
            }
        ]

        # Mock the OpenAI client
        with patch.object(chat_service, 'openai_client') as mock_openai:
            mock_response = MagicMock()
            mock_response.choices = [MagicMock()]
            mock_response.choices[0].message.content = "Based on the selected text, inverse kinematics involves calculating joint angles from end-effector position."
            mock_openai.chat.completions.create.return_value = mock_response
            mock_embedding_response = MagicMock()
            mock_embedding_response.data = [MagicMock()]
            mock_embedding_response.data[0].embedding = [0.1] * 1536
            mock_openai.embeddings.create.return_value = mock_embedding_response

            # Mock the message addition
            with patch.object(chat_service, 'add_message_to_session') as mock_add_message:
                mock_user_message = MagicMock(spec=ChatMessage)
                mock_user_message.id = "user-msg-3"
                mock_user_message.session_id = "test-general-method-303"
                mock_user_message.sender_type = "user"
                mock_user_message.content = "What is inverse kinematics?"
                mock_user_message.timestamp = datetime.now()
                mock_user_message.context_used = None
                mock_add_message.return_value = mock_user_message

                # Call the general processing method (this should handle selected-text-only mode)
                response = chat_service.process_user_message(
                    mock_db,
                    "test-general-method-303",
                    "What is inverse kinematics?"
                )

                # Verify the response
                assert "inverse kinematics" in response.lower()
                
                # Verify that the search was called with a query that includes the selected text
                mock_search.assert_called_once()
                # Check that the search method was called with the expected parameters
                call_args = mock_search.call_args
                # The search_content_by_text_similarity method has named parameters
                assert "query_text" in call_args.kwargs
                assert "Regarding the selected text" in call_args.kwargs["query_text"]  # query_text should include selected text