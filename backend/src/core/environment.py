"""
Environment Configuration Management

This module provides utilities for managing different environment configurations
(Development, Staging, Production) and validating required environment variables.
"""
from typing import Dict, Any
import sys
from .config import settings


def validate_environment() -> bool:
    """
    Validate that all required environment variables are set correctly.
    Returns True if validation passes, False otherwise.
    """
    validation_errors = []
    
    # Check required variables
    if not settings.database_url:
        validation_errors.append("DATABASE_URL is required")
    
    if not settings.jwt_secret_key:
        validation_errors.append("JWT_SECRET_KEY is required")
    
    # Check rate limit formats (already validated by pydantic but we can double check)
    rate_limits = [
        settings.chat_rate_limit,
        settings.translation_rate_limit,
        settings.content_rate_limit,
    ]
    
    for rate_limit in rate_limits:
        if '/' not in rate_limit:
            validation_errors.append(f"Invalid rate limit format: {rate_limit}")
    
    # Log validation errors if any
    if validation_errors:
        for error in validation_errors:
            print(f"Configuration Error: {error}", file=sys.stderr)
        return False
    
    return True


def get_current_environment_info() -> Dict[str, Any]:
    """
    Get information about the current environment configuration.
    """
    return {
        "environment": settings.environment,
        "app_name": settings.app_name,
        "app_version": settings.app_version,
        "debug": settings.debug,
        "database_url_set": bool(settings.database_url),
        "qdrant_configured": bool(settings.qdrant_url and settings.qdrant_api_key),
        "openai_configured": bool(settings.openai_api_key),
        "log_level": settings.log_level,
        "is_development": settings.is_development(),
        "is_production": settings.is_production(),
        "is_staging": settings.is_staging(),
    }


def check_service_readiness() -> Dict[str, bool]:
    """
    Check if all required services are properly configured.
    """
    return {
        "database": bool(settings.database_url),
        "jwt": bool(settings.jwt_secret_key),
        "qdrant": bool(settings.qdrant_url and settings.qdrant_api_key),
        "openai": bool(settings.openai_api_key),
        "huggingface": bool(settings.huggingface_api_key),
        "google_gemini": bool(settings.google_gemini_api_key),
    }


def print_environment_summary():
    """
    Print a summary of the current environment configuration.
    """
    env_info = get_current_environment_info()
    service_readiness = check_service_readiness()
    
    print(f"Application: {env_info['app_name']} v{env_info['app_version']}")
    print(f"Environment: {env_info['environment']}")
    print(f"Debug Mode: {env_info['debug']}")
    print(f"Log Level: {env_info['log_level']}")
    
    print("\nService Configuration:")
    for service, configured in service_readiness.items():
        status_symbol = "+" if configured else "-"
        print(f"  {status_symbol} {service.capitalize()}: {'Configured' if configured else 'Not configured'}")

    # Validate the environment
    is_valid = validate_environment()
    print(f"\nConfiguration Validation: {'Passed' if is_valid else 'Failed'}")

    return is_valid