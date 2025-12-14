from fastapi import APIRouter

router = APIRouter(prefix="/chat", tags=["Chat"])

# Placeholder endpoint
@router.get("/")
def get_chat():
    return {"message": "Chat API placeholder"}