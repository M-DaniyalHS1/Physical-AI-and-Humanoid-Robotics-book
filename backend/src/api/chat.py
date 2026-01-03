from fastapi import APIRouter, Depends, HTTPException, Query
from typing import List, Optional
from sqlalchemy.orm import Session
from src.models.chat_session import ChatSession
from src.models.chat_message import ChatMessage
from src.models.database import get_db
from src.api.auth import get_current_user
from src.models.user import User

router = APIRouter(prefix="/chat", tags=["Chat"])

# Placeholder endpoint
@router.get("/")
def get_chat():
    return {"message": "Chat API placeholder"}

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
    # Query chat sessions for the current user
    sessions = db.query(ChatSession).filter(
        ChatSession.user_id == current_user.id
    ).offset(skip).limit(limit).all()

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

    # Query messages for the session
    messages = db.query(ChatMessage).filter(
        ChatMessage.session_id == session_id
    ).offset(skip).limit(limit).all()

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