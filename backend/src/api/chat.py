from fastapi import APIRouter, Depends, HTTPException, Query
from typing import List, Optional
from pydantic import BaseModel, Field
from sqlalchemy.orm import Session
from src.models.chat_session import ChatSession
from src.models.chat_message import ChatMessage
from src.models.database import get_db
from src.api.auth import get_current_user
from src.models.user import User
from src.services.chat_service import chat_service

router = APIRouter(prefix="/chat", tags=["Chat"])

# Placeholder endpoint
@router.get("/")
def get_chat():
    return {"message": "Chat API placeholder"}

class CreateSessionRequest(BaseModel):
    selected_text: Optional[str] = Field(None, description="Optional text that the user has selected to focus on")
    mode: str = Field("general", description="The mode of the chat session (general or selected-text-only)")


@router.post("/sessions", response_model=dict)
def create_chat_session(
    request: CreateSessionRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Create a new chat session for the current user.
    """
    try:
        session = chat_service.create_chat_session(
            user_id=current_user.id,
            selected_text=request.selected_text,
            mode=request.mode
        )

        # Add the session to the database
        db.add(session)
        db.commit()
        db.refresh(session)

        return {
            "id": session.id,
            "user_id": session.user_id,
            "session_start": session.session_start.isoformat() if session.session_start else None,
            "selected_text": session.selected_text,
            "mode": session.mode,
            "is_active": session.is_active
        }
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/sessions", response_model=List[dict])
def get_chat_sessions(
    current_user: User = Depends(get_current_user),
    skip: int = Query(0, ge=0),
    limit: int = Query(100, ge=0, le=100),
    db: Session = Depends(get_db)
):
    """
    Get a list of chat sessions for the current user.
    Supports pagination via skip and limit parameters.
    """
    try:
        sessions = chat_service.get_user_sessions(db, current_user.id, skip, limit)

        # Convert to response format
        response_sessions = []
        for session in sessions:
            response_sessions.append({
                "id": session.id,
                "user_id": session.user_id,
                "session_start": session.session_start.isoformat() if session.session_start else None,
                "session_end": session.session_end.isoformat() if session.session_end else None,
                "selected_text": session.selected_text,
                "mode": session.mode,
                "is_active": session.is_active
            })

        return response_sessions
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


class SendMessageRequest(BaseModel):
    message: str = Field(..., min_length=1, max_length=10000, description="The message content from the user")


@router.post("/sessions/{session_id}/messages", response_model=dict)
def send_message_to_session(
    session_id: str,
    request: SendMessageRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Send a message to a specific chat session and get an AI response.
    Verifies that the session belongs to the current user.
    """
    # Verify that the session belongs to the current user
    session = db.query(ChatSession).filter(
        ChatSession.id == session_id,
        ChatSession.user_id == current_user.id
    ).first()

    if not session:
        raise HTTPException(status_code=404, detail="Chat session not found")

    try:
        # Process the user message using the chat service
        ai_response = chat_service.process_user_message(db, session_id, request.message)

        # Get the latest messages (user message and AI response)
        messages = chat_service.get_session_messages(db, session_id, skip=0, limit=2)

        # Find the AI response message to return
        ai_message = None
        for msg in messages:
            if msg.sender_type == "ai":
                ai_message = msg
                break

        if ai_message:
            return {
                "session_id": session_id,
                "message_id": ai_message.id,
                "sender_type": ai_message.sender_type,
                "content": ai_message.content,
                "timestamp": ai_message.timestamp.isoformat() if ai_message.timestamp else None,
                "context_used": ai_message.context_used
            }
        else:
            # If we couldn't find the AI message, return the response content
            return {
                "session_id": session_id,
                "content": ai_response,
                "sender_type": "ai"
            }
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/sessions/{session_id}/messages", response_model=List[dict])
def get_chat_session_messages(
    session_id: str,
    current_user: User = Depends(get_current_user),
    skip: int = Query(0, ge=0),
    limit: int = Query(100, ge=0, le=100),
    db: Session = Depends(get_db)
):
    """
    Get messages for a specific chat session.
    Verifies that the session belongs to the current user.
    Supports pagination via skip and limit parameters.
    """
    # Verify that the session belongs to the current user
    session = db.query(ChatSession).filter(
        ChatSession.id == session_id,
        ChatSession.user_id == current_user.id
    ).first()

    if not session:
        raise HTTPException(status_code=404, detail="Chat session not found")

    try:
        # Use the chat service to get messages
        messages = chat_service.get_session_messages(db, session_id, skip, limit)

        # Convert to response format
        response_messages = []
        for message in messages:
            response_messages.append({
                "id": message.id,
                "session_id": message.session_id,
                "sender_type": message.sender_type,
                "content": message.content,
                "timestamp": message.timestamp.isoformat() if message.timestamp else None,
                "context_used": message.context_used
            })

        return response_messages
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.patch("/sessions/{session_id}/close", response_model=dict)
def close_chat_session(
    session_id: str,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Close a specific chat session.
    Verifies that the session belongs to the current user.
    """
    # Verify that the session belongs to the current user
    session = db.query(ChatSession).filter(
        ChatSession.id == session_id,
        ChatSession.user_id == current_user.id
    ).first()

    if not session:
        raise HTTPException(status_code=404, detail="Chat session not found")

    try:
        success = chat_service.close_session(db, session_id)

        if success:
            return {
                "session_id": session_id,
                "message": "Session closed successfully",
                "is_active": False
            }
        else:
            raise HTTPException(status_code=500, detail="Failed to close session")
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))