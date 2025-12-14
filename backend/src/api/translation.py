from fastapi import APIRouter

router = APIRouter(prefix="/translation", tags=["Translation"])

# Placeholder endpoint
@router.get("/")
def get_translation():
    return {"message": "Translation API placeholder"}