"""
Chat Service for AI Textbook Platform

This module provides chat functionality using OpenAI Agent SDK and Chainkit,
with integration to RAG service for textbook content retrieval.
"""
from typing import List, Dict, Optional
from datetime import datetime
import logging
import uuid
from sqlalchemy.orm import Session

from openai import OpenAI
from ..models.chat_session import ChatSession
from ..models.chat_message import ChatMessage
from .rag_service import rag_service
from ..core.config import settings

logger = logging.getLogger(__name__)

class ChatService:
    """
    Service class for chat functionality using OpenAI Agent SDK and Chainkit,
    with integration to RAG service for textbook content retrieval.
    """

    def __init__(self):
        self.rag_service = rag_service
        # Initialize OpenAI client
        if settings.openai_api_key:
            self.openai_client = OpenAI(api_key=settings.openai_api_key)
            logger.info("ChatService initialized with OpenAI client")
        else:
            self.openai_client = None
            logger.warning("OpenAI API key not found. Chat functionality will use mock responses.")

    def create_chat_session(self, user_id: str, selected_text: Optional[str] = None, mode: str = "general") -> ChatSession:
        """
        Create a new chat session for a user

        Args:
            user_id: The ID of the user creating the session
            selected_text: Optional text that the user has selected to focus on
            mode: The mode of the chat session (general or selected-text-only)

        Returns:
            ChatSession: The created chat session
        """
        # Validate inputs
        if not user_id or not isinstance(user_id, str) or len(user_id.strip()) == 0:
            raise ValueError("user_id must be a non-empty string")

        valid_modes = ["general", "selected-text-only"]
        if mode not in valid_modes:
            raise ValueError(f"mode must be one of {valid_modes}, got '{mode}'")

        if mode == "selected-text-only" and (not selected_text or not isinstance(selected_text, str) or len(selected_text.strip()) == 0):
            raise ValueError("selected_text must be a non-empty string when mode is 'selected-text-only'")

        session_id = str(uuid.uuid4())
        session = ChatSession(
            id=session_id,
            user_id=user_id,
            session_start=datetime.now(),
            selected_text=selected_text,
            mode=mode,
            is_active=True
        )
        return session

    def add_message_to_session(self, db: Session, session_id: str, sender_type: str, content: str, context_used: Optional[str] = None) -> ChatMessage:
        """
        Add a message to a chat session

        Args:
            db: Database session
            session_id: The ID of the chat session
            sender_type: The type of sender (user or ai)
            content: The content of the message
            context_used: Optional context from textbook content used to generate response

        Returns:
            ChatMessage: The created chat message
        """
        message_id = str(uuid.uuid4())
        message = ChatMessage(
            id=message_id,
            session_id=session_id,
            sender_type=sender_type,
            content=content,
            context_used=context_used
        )

        # Add to database
        db.add(message)
        db.commit()
        db.refresh(message)

        return message

    def get_session_messages(self, db: Session, session_id: str, skip: int = 0, limit: int = 100) -> List[ChatMessage]:
        """
        Get messages for a specific chat session

        Args:
            db: Database session
            session_id: The ID of the chat session
            skip: Number of messages to skip (for pagination)
            limit: Maximum number of messages to return

        Returns:
            List[ChatMessage]: List of messages in the session
        """
        # Validate inputs
        if not session_id or not isinstance(session_id, str) or len(session_id.strip()) == 0:
            raise ValueError("session_id must be a non-empty string")

        if not isinstance(skip, int) or skip < 0:
            raise ValueError("skip must be a non-negative integer")

        if not isinstance(limit, int) or limit <= 0 or limit > 1000:
            raise ValueError("limit must be a positive integer not exceeding 1000")

        messages = db.query(ChatMessage).filter(
            ChatMessage.session_id == session_id
        ).offset(skip).limit(limit).all()
        return messages

    def process_user_message(self, db: Session, session_id: str, user_message: str) -> str:
        """
        Process a user message and generate an AI response using RAG

        Args:
            db: Database session
            session_id: The ID of the chat session
            user_message: The message from the user

        Returns:
            str: The AI-generated response
        """
        # Validate inputs
        if not session_id or not isinstance(session_id, str) or len(session_id.strip()) == 0:
            raise ValueError("session_id must be a non-empty string")

        if not user_message or not isinstance(user_message, str) or len(user_message.strip()) == 0:
            raise ValueError("user_message must be a non-empty string")

        if len(user_message.strip()) > 10000:  # Limit message length
            raise ValueError("user_message is too long (max 10000 characters)")

        # First, get the session to check its mode and selected text
        session = db.query(ChatSession).filter(ChatSession.id == session_id).first()
        if not session:
            raise ValueError(f"Session {session_id} not found")

        # Add user message to session
        self.add_message_to_session(db, session_id, "user", user_message)

        # Prepare the query for RAG based on session mode
        query_text = user_message
        if session.mode == "selected-text-only" and session.selected_text:
            # In selected-text-only mode, focus the query on the selected text
            query_text = f"Regarding the selected text: '{session.selected_text}', {user_message.lower()}"

        # Generate embedding for the query using OpenAI
        try:
            if self.openai_client:
                response = self.openai_client.embeddings.create(
                    input=query_text,
                    model="text-embedding-ada-002"
                )
                query_vector = response.data[0].embedding
            else:
                # Fallback to mock vector if OpenAI client is not available
                query_vector = [0.1] * 1536
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            # Fallback to mock vector if embedding fails
            query_vector = [0.1] * 1536

        # Use the RAG service to find relevant textbook content
        relevant_content = self.rag_service.search_content_by_text_similarity(
            query_text=query_text,
            query_vector=query_vector,
            limit=3
        )

        # Prepare context for the AI model
        context_str = ""
        if session.mode == "selected-text-only" and session.selected_text:
            # In selected-text-only mode, prioritize the selected text over other content
            # If the selected text is similar to content in the database, use it; otherwise, use the selected text as context
            context_str = f"Selected text: {session.selected_text}"

            # If we have relevant content that matches the selected text, we can add it too
            if relevant_content:
                # Check if any of the retrieved content is related to the selected text
                # For now, we'll add the selected text as primary context and include relevant content if it's related
                related_content = []
                for item in relevant_content:
                    content_text = item["content_text"]
                    # Simple check if the content is related to the selected text
                    if (session.selected_text.lower() in content_text.lower() or
                        any(word in content_text.lower() for word in session.selected_text.lower().split()[:5])):  # Check first 5 words
                        related_content.append(content_text)

                if related_content:
                    context_str += "\n\nAdditional related content from textbook:\n" + "\n\n".join(related_content)
                else:
                    # If no related content found, just use the selected text
                    context_str = f"Selected text: {session.selected_text}\n\nIMPORTANT: Answer only based on this selected text."
        else:
            # General mode - use all relevant content as before
            if relevant_content:
                context_snippets = [item["content_text"] for item in relevant_content]
                context_str = "\n\n".join(context_snippets)
            else:
                context_str = "No specific textbook content found. Please provide general information about Physical AI & Humanoid Robotics."

        # Prepare the prompt for the AI model
        system_prompt = f"""You are an AI assistant for a Physical AI & Humanoid Robotics textbook.
        Use the following context from the textbook to answer the user's question.
        If the context doesn't contain relevant information, provide general information about Physical AI & Humanoid Robotics.
        Context: {context_str}"""

        # If in selected-text-only mode, emphasize using only the selected text
        if session.mode == "selected-text-only" and session.selected_text:
            system_prompt += f"\n\nIMPORTANT: Answer the user's question based ONLY on the selected text provided above. " \
                            "Do not use any other knowledge or information beyond what is in the selected text and related content."

        try:
            if self.openai_client:
                # Use OpenAI to generate a response based on the context
                response = self.openai_client.chat.completions.create(
                    model="gpt-3.5-turbo",
                    messages=[
                        {"role": "system", "content": system_prompt},
                        {"role": "user", "content": user_message}
                    ],
                    temperature=0.7,
                    max_tokens=500
                )

                ai_response = response.choices[0].message.content
            else:
                # Fallback response when OpenAI client is not available
                ai_response = f"Based on the textbook content: {context_str[:200]}... [This is a mock response as OpenAI API is not configured]"
        except Exception as e:
            logger.error(f"Error generating AI response: {e}")
            ai_response = "I'm sorry, I encountered an error processing your request. Please try again later."

        # Add AI response to session
        self.add_message_to_session(db, session_id, "ai", ai_response, context_used=str(relevant_content[:1]) if relevant_content else None)

        return ai_response

    def process_user_message_selected_text_only(self, db: Session, session_id: str, user_message: str) -> str:
        """
        Process a user message in selected-text-only mode, focusing exclusively on the selected text

        Args:
            db: Database session
            session_id: The ID of the chat session
            user_message: The message from the user

        Returns:
            str: The AI-generated response based only on the selected text
        """
        # Validate inputs
        if not session_id or not isinstance(session_id, str) or len(session_id.strip()) == 0:
            raise ValueError("session_id must be a non-empty string")

        if not user_message or not isinstance(user_message, str) or len(user_message.strip()) == 0:
            raise ValueError("user_message must be a non-empty string")

        if len(user_message.strip()) > 10000:  # Limit message length
            raise ValueError("user_message is too long (max 10000 characters)")

        # First, get the session to check its mode and selected text
        session = db.query(ChatSession).filter(ChatSession.id == session_id).first()
        if not session:
            raise ValueError(f"Session {session_id} not found")

        # Verify that the session is in selected-text-only mode
        if session.mode != "selected-text-only":
            raise ValueError(f"Session {session_id} is not in selected-text-only mode")

        # Verify that selected text exists
        if not session.selected_text:
            raise ValueError(f"Session {session_id} does not have selected text for selected-text-only mode")

        # Add user message to session
        self.add_message_to_session(db, session_id, "user", user_message)

        # For selected-text-only mode, we'll use the selected text as the primary context
        # Generate embedding for the selected text combined with the user query
        combined_query = f"{session.selected_text} {user_message}"

        try:
            if self.openai_client:
                response = self.openai_client.embeddings.create(
                    input=combined_query,
                    model="text-embedding-ada-002"
                )
                query_vector = response.data[0].embedding
            else:
                # Fallback to mock vector if OpenAI client is not available
                query_vector = [0.1] * 1536
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            # Fallback to mock vector if embedding fails
            query_vector = [0.1] * 1536

        # Use the RAG service to find content similar to the selected text
        # In selected-text-only mode, we want to find content that's most similar to the selected text
        relevant_content = self.rag_service.search_content_by_text_similarity(
            query_text=combined_query,
            query_vector=query_vector,
            limit=5  # Get more results to ensure we have relevant content
        )

        # For selected-text-only mode, we'll create a context that prioritizes the selected text
        # and any closely related content
        context_str = f"Selected Text: {session.selected_text}\n\n"

        if relevant_content:
            # Filter to only include content that is highly related to the selected text
            # We'll use a higher threshold to ensure relevance
            filtered_content = [item for item in relevant_content if item["score"] > 0.7]

            if filtered_content:
                context_str += "Related Content:\n"
                for item in filtered_content:
                    context_str += f"- {item['content_text']}\n\n"
            else:
                # If no highly relevant content found, just use the selected text
                context_str = f"Selected Text: {session.selected_text}\n\n"
                logger.info(f"No highly relevant content found for selected-text-only mode in session {session_id}")
        else:
            context_str = f"Selected Text: {session.selected_text}\n\n"
            logger.warning(f"No content found for selected-text-only mode in session {session_id}")

        # Prepare the prompt for the AI model with strict instructions for selected-text-only mode
        system_prompt = f"""You are an AI assistant for a Physical AI & Humanoid Robotics textbook.
        You are in SELECTED-TEXT-ONLY mode.
        Answer the user's question based ONLY on the following selected text and closely related concepts.
        Do not provide any information outside of the provided context.
        If the selected text does not contain enough information to answer the question, acknowledge this limitation.
        Selected Text and Related Content:
        {context_str}"""

        try:
            if self.openai_client:
                # Use OpenAI to generate a response based only on the selected text
                response = self.openai_client.chat.completions.create(
                    model="gpt-3.5-turbo",
                    messages=[
                        {"role": "system", "content": system_prompt},
                        {"role": "user", "content": user_message}
                    ],
                    temperature=0.3,  # Lower temperature for more consistent, focused responses
                    max_tokens=500
                )

                ai_response = response.choices[0].message.content
            else:
                # Fallback response when OpenAI client is not available
                ai_response = f"Based on the selected text: '{session.selected_text[:200]}...' [This is a mock response as OpenAI API is not configured]"
        except Exception as e:
            logger.error(f"Error generating AI response in selected-text-only mode: {e}")
            ai_response = "I'm sorry, I encountered an error processing your request. Please try again later."

        # Add AI response to session
        self.add_message_to_session(db, session_id, "ai", ai_response, context_used=f"Selected Text: {session.selected_text}")

        return ai_response

    def close_session(self, db: Session, session_id: str) -> bool:
        """
        Close a chat session

        Args:
            db: Database session
            session_id: The ID of the chat session to close

        Returns:
            bool: True if successful, False otherwise
        """
        session = db.query(ChatSession).filter(ChatSession.id == session_id).first()
        if not session:
            return False

        session.is_active = False
        session.session_end = datetime.now()
        db.commit()
        return True

    def get_user_sessions(self, db: Session, user_id: str, skip: int = 0, limit: int = 100) -> List[ChatSession]:
        """
        Get all chat sessions for a specific user

        Args:
            db: Database session
            user_id: The ID of the user
            skip: Number of sessions to skip (for pagination)
            limit: Maximum number of sessions to return

        Returns:
            List[ChatSession]: List of chat sessions for the user
        """
        # Validate inputs
        if not user_id or not isinstance(user_id, str) or len(user_id.strip()) == 0:
            raise ValueError("user_id must be a non-empty string")

        if not isinstance(skip, int) or skip < 0:
            raise ValueError("skip must be a non-negative integer")

        if not isinstance(limit, int) or limit <= 0 or limit > 1000:
            raise ValueError("limit must be a positive integer not exceeding 1000")

        sessions = db.query(ChatSession).filter(
            ChatSession.user_id == user_id
        ).offset(skip).limit(limit).all()
        return sessions


# Singleton instance of Chat Service
chat_service = ChatService()