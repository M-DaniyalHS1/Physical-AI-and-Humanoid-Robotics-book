import time
from collections import defaultdict
from fastapi import Request, Response
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import ClientDisconnect
from typing import Optional
import logging

from ..config import settings

logger = logging.getLogger(__name__)

class RateLimitMiddleware(BaseHTTPMiddleware):
    def __init__(self, app):
        super().__init__(app)
        self.store = defaultdict(list)  # In-memory store (fallback option)

        # Initialize Redis if available
        self.redis_client: Optional[object] = None
        try:
            import redis
            if settings.redis_url:  # assuming redis_url is configured in settings
                self.redis_client = redis.from_url(settings.redis_url)
                logger.info("Redis client initialized for rate limiting")
            else:
                logger.info("No Redis URL provided, using in-memory rate limiting")
        except ImportError:
            logger.warning("Redis library not installed, using in-memory rate limiting")
        except Exception as e:
            logger.warning(f"Could not connect to Redis for rate limiting: {e}, using in-memory store")

    async def dispatch(self, request: Request, call_next):
        # Get the client IP address
        client_ip = request.client.host

        # Get the matched path to determine rate limits by endpoint
        path = request.url.path

        # Determine rate limits based on the path
        # According to the config, we have different limits for different features:
        # - chat_rate_limit: 10 requests/minute
        # - translation_rate_limit: 50 requests/minute
        # - content_rate_limit: 100 requests/minute

        if "/chat" in path:
            max_requests, window_seconds = RateLimitMiddleware.parse_rate_limit(settings.chat_rate_limit)
        elif "/translate" in path:
            max_requests, window_seconds = RateLimitMiddleware.parse_rate_limit(settings.translation_rate_limit)
        elif "/content" in path:
            max_requests, window_seconds = RateLimitMiddleware.parse_rate_limit(settings.content_rate_limit)
        else:
            # Default rate limit for other endpoints
            max_requests, window_seconds = 100, 60  # 100 requests per minute

        current_time = time.time()

        # Use Redis if available, otherwise use in-memory store
        if self.redis_client:
            result = await self._check_rate_limit_redis(client_ip, max_requests, window_seconds, current_time)
        else:
            result = self._check_rate_limit_memory(client_ip, max_requests, window_seconds, current_time)

        # Check if the client has exceeded the rate limit
        if not result:
            return JSONResponse(
                status_code=429,
                content={
                    "detail": f"Rate limit exceeded. Maximum {max_requests} requests per {window_seconds} seconds."
                }
            )

        # Continue processing the request
        try:
            response = await call_next(request)
            return response
        except ClientDisconnect:
            # Handle client disconnection gracefully
            pass

    def _check_rate_limit_memory(self, client_ip: str, max_requests: int, window_seconds: int, current_time: float) -> bool:
        """Check rate limit using in-memory store"""
        # Clean old entries older than the window period
        self.store[client_ip] = [
            timestamp for timestamp in self.store[client_ip]
            if current_time - timestamp < window_seconds
        ]

        # Check if the client has exceeded the rate limit
        if len(self.store[client_ip]) >= max_requests:
            return False

        # Add the current request timestamp
        self.store[client_ip].append(current_time)
        return True

    async def _check_rate_limit_redis(self, client_ip: str, max_requests: int, window_seconds: int, current_time: float) -> bool:
        """Check rate limit using Redis (async implementation would require aioredis)"""
        # For now, we'll use the synchronous redis client with a simple implementation
        # In production, consider using aioredis for async operations
        try:
            from redis import Redis
            redis_client: Redis = self.redis_client

            # Create a key specific to this client and window
            key = f"rate_limit:{client_ip}:{int(current_time // window_seconds)}"

            # Get current count
            current_count = redis_client.get(key)
            if current_count is None:
                # Set count to 1 with expiration
                redis_client.setex(key, window_seconds, 1)
                return True
            else:
                count = int(current_count)
                if count >= max_requests:
                    return False
                else:
                    # Increment the count
                    redis_client.incr(key)
                    return True
        except Exception as e:
            logger.error(f"Error with Redis rate limiting: {e}, falling back to memory store")
            # Fallback to memory store
            return self._check_rate_limit_memory(client_ip, max_requests, window_seconds, current_time)

    @staticmethod
    def parse_rate_limit(rate_limit_str: str):
        """
        Parse rate limit string like "10/minute" into (max_requests, window_seconds)
        """
        try:
            number, period = rate_limit_str.split('/')
            max_requests = int(number)
        except ValueError:
            # If format is not X/Y, default to 100 requests per minute
            return 100, 60

        if 'second' in period:
            window_seconds = 1
        elif 'minute' in period:
            window_seconds = 60
        elif 'hour' in period:
            window_seconds = 3600
        else:
            # Default to minute
            window_seconds = 60

        return max_requests, window_seconds