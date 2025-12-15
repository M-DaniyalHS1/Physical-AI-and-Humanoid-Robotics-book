from fastapi import APIRouter

router = APIRouter(prefix="/content", tags=["Content"])

# Placeholder endpoint
@router.get("/")
def get_content():
    return {"message": "Content API placeholder"}