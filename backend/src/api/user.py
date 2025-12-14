from fastapi import APIRouter

router = APIRouter(prefix="/user", tags=["User"])

# Placeholder endpoint
@router.get("/")
def get_user():
    return {"message": "User API placeholder"}