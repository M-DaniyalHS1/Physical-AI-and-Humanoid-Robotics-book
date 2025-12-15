from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from sqlalchemy.orm import Session
from typing import Optional
from datetime import datetime, timedelta
import jwt
from passlib.context import CryptContext
import uuid

from ..models.database import get_db
from ..models.user import User
from ..core.config import settings

router = APIRouter(prefix="/auth", tags=["Authentication"])

# Password hashing
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# OAuth2 scheme for login
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="auth/login")

def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a plain password against its hash."""
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password: str) -> str:
    """Generate a hash for a password."""
    return pwd_context.hash(password)

def authenticate_user(db: Session, email: str, password: str) -> Optional[User]:
    """Authenticate user by email and password."""
    user = db.query(User).filter(User.email == email).first()
    if not user or not verify_password(password, user.password_hash):
        return None
    return user

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    """Create a JWT access token."""
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)  # Default 15 minutes
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, settings.jwt_secret_key, algorithm=settings.jwt_algorithm)
    return encoded_jwt

def get_current_user(token: str = Depends(oauth2_scheme), db: Session = Depends(get_db)) -> User:
    """Get the current authenticated user from the token."""
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    try:
        payload = jwt.decode(token, settings.jwt_secret_key, algorithms=[settings.jwt_algorithm])
        email: str = payload.get("sub")
        if email is None:
            raise credentials_exception
    except jwt.PyJWTError:
        raise credentials_exception
    user = db.query(User).filter(User.email == email).first()
    if user is None:
        raise credentials_exception
    return user

@router.post("/signup", status_code=status.HTTP_201_CREATED)
async def signup(
    email: str,
    password: str,
    software_experience: str,
    hardware_experience: str,
    math_physics_level: str,
    learning_goals: str = "",
    db: Session = Depends(get_db)
):
    """Register a new user with their background information."""
    # Check if user already exists
    existing_user = db.query(User).filter(User.email == email).first()
    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="User with this email already exists"
        )

    # Hash the password
    hashed_password = get_password_hash(password)

    # Create new user
    user = User(
        email=email,
        password_hash=hashed_password,
        software_experience=software_experience,
        hardware_experience=hardware_experience,
        math_physics_level=math_physics_level,
        learning_goals=learning_goals
    )
    
    db.add(user)
    db.commit()
    db.refresh(user)

    # Return user without password hash
    return {
        "id": user.id,
        "email": user.email,
        "software_experience": user.software_experience,
        "hardware_experience": user.hardware_experience,
        "math_physics_level": user.math_physics_level,
        "learning_goals": user.learning_goals,
        "created_at": user.created_at,
        "updated_at": user.updated_at,
        "last_login": user.last_login
    }

@router.post("/login")
async def login(form_data: OAuth2PasswordRequestForm = Depends(), db: Session = Depends(get_db)):
    """Authenticate user and return session token."""
    user = authenticate_user(db, form_data.username, form_data.password)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )
    
    # Update last login
    user.last_login = datetime.utcnow()
    db.commit()
    
    # Create access token
    access_token_expires = timedelta(minutes=settings.access_token_expire_minutes)
    access_token = create_access_token(
        data={"sub": user.email}, expires_delta=access_token_expires
    )
    
    return {
        "access_token": access_token,
        "token_type": "bearer",
        "user": {
            "id": user.id,
            "email": user.email,
            "software_experience": user.software_experience,
            "hardware_experience": user.hardware_experience,
            "math_physics_level": user.math_physics_level,
            "learning_goals": user.learning_goals,
            "created_at": user.created_at,
            "updated_at": user.updated_at,
            "last_login": user.last_login
        }
    }

@router.get("/me")
async def read_users_me(current_user: User = Depends(get_current_user)):
    """Get current authenticated user."""
    return current_user