"""
API module initialization - aggregates all API routers
"""
from . import auth
from . import content
from . import chat
from . import user
from . import translation

__all__ = ["auth", "content", "chat", "user", "translation"]