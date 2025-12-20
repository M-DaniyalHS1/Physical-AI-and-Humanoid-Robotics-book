# Use an official Python runtime as a parent image
FROM python:3.11-slim

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1

# Set work directory
WORKDIR /app

# Install system dependencies
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        postgresql-client \
        build-essential \
        netcat-openbsd \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy project
COPY backend/src/ ./src/
COPY backend/alembic/ ./alembic/
COPY backend/alembic.ini ./

# Create a non-root user
RUN adduser --disabled-password --gecos '' appuser
RUN chown -R appuser:appuser /app
USER appuser

# Expose port
EXPOSE 8000

# Set PYTHONPATH to ensure modules can be found
ENV PYTHONPATH=/app

# Copy the startup script
COPY backend/start.sh /app/start.sh
RUN chmod +x /app/start.sh

# Run the application
# Use the PORT environment variable provided by Railway, defaulting to 8000 if not set
CMD ["/app/start.sh"]