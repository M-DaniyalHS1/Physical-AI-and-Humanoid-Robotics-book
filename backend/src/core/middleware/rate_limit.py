import time
from collections import defaultdict
from fastapi import Request, Response
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import ClientDisconnect
from ..config import settings


class RateLimitMiddleware(BaseHTTPMiddleware):
    def __init__(self, app):
        super().__init__(app)
        self.store = defaultdict(list)  # In-memory store (would use Redis in production)
        
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
            max_requests, window_seconds = self.parse_rate_limit(settings.chat_rate_limit)
        elif "/translate" in path:
            max_requests, window_seconds = self.parse_rate_limit(settings.translation_rate_limit)
        elif "/content" in path:
            max_requests, window_seconds = self.parse_rate_limit(settings.content_rate_limit)
        else:
            # Default rate limit for other endpoints
            max_requests, window_seconds = 100, 60  # 100 requests per minute

        current_time = time.time()

        # Clean old entries older than the window period
        self.store[client_ip] = [
            timestamp for timestamp in self.store[client_ip]
            if current_time - timestamp < window_seconds
        ]

        # Check if the client has exceeded the rate limit
        if len(self.store[client_ip]) >= max_requests:
            return JSONResponse(
                status_code=429,
                content={
                    "detail": f"Rate limit exceeded. Maximum {max_requests} requests per {window_seconds} seconds."
                }
            )

        # Add the current request timestamp
        self.store[client_ip].append(current_time)

        # Continue processing the request
        try:
            response = await call_next(request)
            return response
        except ClientDisconnect:
            # Handle client disconnection gracefully
            pass

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