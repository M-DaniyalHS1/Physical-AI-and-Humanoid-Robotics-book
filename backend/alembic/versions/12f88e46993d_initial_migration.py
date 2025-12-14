"""Initial migration

Revision ID: 12f88e46993d
Revises: 
Create Date: 2025-12-14 14:30:00.000000

"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers, used by Alembic.
revision = '12f88e46993d'
down_revision = None
branch_labels = None
depends_on = None


def upgrade() -> None
    # Create users table
    op.create_table(
        'users',
        sa.Column('id', sa.String(), nullable=False),
        sa.Column('email', sa.String(), nullable=False),
        sa.Column('password_hash', sa.String(), nullable=False),
        sa.Column('software_experience', sa.String(), nullable=False),
        sa.Column('hardware_experience', sa.String(), nullable=False),
        sa.Column('math_physics_level', sa.String(), nullable=True),
        sa.Column('learning_goals', sa.String(), nullable=True),
        sa.Column('personalization_settings', sa.String(), nullable=True),
        sa.Column('created_at', sa.DateTime(), nullable=True),
        sa.Column('updated_at', sa.DateTime(), nullable=True),
        sa.Column('last_login', sa.DateTime(), nullable=True),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('email')
    )

    # Create book_content table
    op.create_table(
        'book_content',
        sa.Column('id', sa.String(), nullable=False),
        sa.Column('title', sa.String(), nullable=False),
        sa.Column('module', sa.String(), nullable=False),
        sa.Column('chapter_number', sa.Integer(), nullable=False),
        sa.Column('content_type', sa.String(), nullable=True),
        sa.Column('content', sa.Text(), nullable=False),
        sa.Column('version', sa.String(), nullable=True),
        sa.Column('vector_id', sa.String(), nullable=True),
        sa.Column('created_at', sa.DateTime(), nullable=True),
        sa.Column('updated_at', sa.DateTime(), nullable=True),
        sa.Column('authors', sa.String(), nullable=True),
        sa.Column('learning_objectives', sa.String(), nullable=True),
        sa.PrimaryKeyConstraint('id')
    )

    # Create chat_sessions table
    op.create_table(
        'chat_sessions',
        sa.Column('id', sa.String(), nullable=False),
        sa.Column('user_id', sa.String(), nullable=False),
        sa.Column('session_start', sa.DateTime(), nullable=True),
        sa.Column('session_end', sa.DateTime(), nullable=True),
        sa.Column('selected_text', sa.Text(), nullable=True),
        sa.Column('mode', sa.String(), nullable=True),
        sa.Column('is_active', sa.Boolean(), nullable=True),
        sa.PrimaryKeyConstraint('id')
    )

    # Create chat_messages table
    op.create_table(
        'chat_messages',
        sa.Column('id', sa.String(), nullable=False),
        sa.Column('session_id', sa.String(), nullable=False),
        sa.Column('sender_type', sa.String(), nullable=False),
        sa.Column('content', sa.Text(), nullable=False),
        sa.Column('timestamp', sa.DateTime(), nullable=True),
        sa.Column('context_used', sa.String(), nullable=True),
        sa.PrimaryKeyConstraint('id')
    )

    # Create personalization_profiles table
    op.create_table(
        'personalization_profiles',
        sa.Column('id', sa.String(), nullable=False),
        sa.Column('user_id', sa.String(), nullable=False),
        sa.Column('chapter_id', sa.String(), nullable=False),
        sa.Column('content_level', sa.String(), nullable=False),
        sa.Column('example_preference', sa.String(), nullable=True),
        sa.Column('update_context', sa.String(), nullable=True),
        sa.Column('created_at', sa.DateTime(), nullable=True),
        sa.Column('updated_at', sa.DateTime(), nullable=True),
        sa.PrimaryKeyConstraint('id')
    )

    # Create translation_cache table
    op.create_table(
        'translation_cache',
        sa.Column('id', sa.String(), nullable=False),
        sa.Column('content_id', sa.String(), nullable=False),
        sa.Column('target_language', sa.String(), nullable=False),
        sa.Column('translated_content', sa.Text(), nullable=False),
        sa.Column('translation_method', sa.String(), nullable=True),
        sa.Column('quality_score', sa.Float(), nullable=True),
        sa.Column('created_at', sa.DateTime(), nullable=True),
        sa.Column('updated_at', sa.DateTime(), nullable=True),
        sa.PrimaryKeyConstraint('id')
    )

    # Create progress table
    op.create_table(
        'progress',
        sa.Column('id', sa.String(), nullable=False),
        sa.Column('user_id', sa.String(), nullable=False),
        sa.Column('content_id', sa.String(), nullable=False),
        sa.Column('progress_percentage', sa.Float(), nullable=True),
        sa.Column('time_spent_seconds', sa.Integer(), nullable=True),
        sa.Column('last_accessed', sa.DateTime(), nullable=True),
        sa.Column('completed_exercises', sa.String(), nullable=True),
        sa.Column('bookmark_position', sa.Integer(), nullable=True),
        sa.Column('created_at', sa.DateTime(), nullable=True),
        sa.Column('updated_at', sa.DateTime(), nullable=True),
        sa.PrimaryKeyConstraint('id')
    )

    # Create content_metadata table
    op.create_table(
        'content_metadata',
        sa.Column('id', sa.String(), nullable=False),
        sa.Column('content_id', sa.String(), nullable=False),
        sa.Column('personalization_level', sa.String(), nullable=False),
        sa.Column('adjusted_content', sa.Text(), nullable=True),
        sa.Column('adjusted_examples', sa.String(), nullable=True),
        sa.Column('difficulty_rating', sa.Integer(), nullable=True),
        sa.Column('estimated_time_minutes', sa.Integer(), nullable=True),
        sa.PrimaryKeyConstraint('id')
    )


def downgrade() -> None
    op.drop_table('content_metadata')
    op.drop_table('progress')
    op.drop_table('translation_cache')
    op.drop_table('personalization_profiles')
    op.drop_table('chat_messages')
    op.drop_table('chat_sessions')
    op.drop_table('book_content')
    op.drop_table('users')