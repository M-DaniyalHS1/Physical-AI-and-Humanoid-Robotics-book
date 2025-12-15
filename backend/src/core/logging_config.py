import logging
import logging.config
import os
from .config import settings


def setup_logging():
    """
    Set up comprehensive logging configuration for the application.
    This includes different log levels, formatters, and handlers for
    different environments (development vs production).
    """

    # Determine log level based on environment
    log_level = os.getenv("LOG_LEVEL", "INFO").upper()

    # Define the logging configuration
    logging_config = {
        "version": 1,
        "disable_existing_loggers": False,
        "formatters": {
            "standard": {
                "format": "%(asctime)s [%(levelname)s] %(name)s: %(message)s",
                "datefmt": "%Y-%m-%d %H:%M:%S"
            },
            "detailed": {
                "format": "%(asctime)s [%(levelname)s] %(name)s:%(lineno)d: %(message)s",
                "datefmt": "%Y-%m-%d %H:%M:%S"
            },
            "json": {
                "format": "%(asctime)s %(name)s %(levelname)s %(message)s"
            }
        },
        "handlers": {
            "console": {
                "level": log_level,
                "class": "logging.StreamHandler",
                "formatter": "standard",
                "stream": "ext://sys.stdout"
            },
            "file": {
                "level": "INFO",
                "class": "logging.handlers.RotatingFileHandler",
                "formatter": "detailed",
                "filename": "logs/app.log",
                "maxBytes": 10485760,  # 10MB
                "backupCount": 5,
                "encoding": "utf8"
            },
            "error_file": {
                "level": "ERROR",
                "class": "logging.handlers.RotatingFileHandler",
                "formatter": "detailed",
                "filename": "logs/error.log",
                "maxBytes": 10485760,  # 10MB
                "backupCount": 5,
                "encoding": "utf8"
            }
        },
        "loggers": {
            "": {  # root logger
                "handlers": ["console", "file", "error_file"],
                "level": log_level,
                "propagate": False
            },
            "uvicorn": {
                "handlers": ["console", "file"],
                "level": log_level,
                "propagate": False
            },
            "uvicorn.error": {
                "handlers": ["console", "error_file"],
                "level": "ERROR",
                "propagate": False
            },
            "uvicorn.access": {
                "handlers": ["console", "file"],
                "level": log_level,
                "propagate": False
            }
        }
    }

    # Create logs directory if it doesn't exist
    os.makedirs("logs", exist_ok=True)

    # Apply the logging configuration
    logging.config.dictConfig(logging_config)

    # Get the root logger and return it
    logger = logging.getLogger(__name__)
    logger.info("Logging configuration has been set up successfully")

    return logger


def get_logger(name: str) -> logging.Logger:
    """
    Get a configured logger instance by name
    """
    return logging.getLogger(name)


# Initialize logging when this module is imported
logger = setup_logging()