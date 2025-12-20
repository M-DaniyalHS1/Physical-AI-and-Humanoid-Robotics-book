#!/bin/bash
set -e

echo "Running database migrations..."
python -m alembic upgrade head
echo "Migrations completed, starting application..."

# Use the PORT environment variable provided by Railway, defaulting to 8000 if not set
gunicorn --worker-class uvicorn.workers.UvicornWorker --workers 2 --bind 0.0.0.0:${PORT:-8000} --timeout 120 --keep-alive 5 src.main:app