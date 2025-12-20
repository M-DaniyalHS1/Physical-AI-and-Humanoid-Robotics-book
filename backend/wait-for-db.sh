#!/bin/bash
# Startup script to ensure database is ready before starting the application

set -e  # Exit on any error

echo "Starting database initialization..."

# Wait for the database to be ready
echo "Checking database connection..."
while ! pg_isready -h $DB_HOST -p $DB_PORT -U $DB_USER; do
  echo "Waiting for database connection..."
  sleep 2
done

echo "Database is ready, running migrations..."

# Run database migrations using alembic
python -m alembic upgrade head

echo "Migrations completed, starting application..."

# Start the main application
exec "$@"