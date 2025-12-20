"""Add created_at and updated_at columns to content_metadata table

Revision ID: f4a2b18870c5
Revises: 12f88e46993d
Create Date: 2025-12-20 18:23:35.678234

"""
from typing import Sequence, Union
import sqlalchemy as sa
from alembic import op


# revision identifiers, used by Alembic.
revision: str = 'f4a2b18870c5'
down_revision: Union[str, None] = '12f88e46993d'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    # Add created_at and updated_at columns to content_metadata table
    op.add_column('content_metadata', sa.Column('created_at', sa.DateTime(), nullable=True))
    op.add_column('content_metadata', sa.Column('updated_at', sa.DateTime(), nullable=True))


def downgrade() -> None:
    # Remove created_at and updated_at columns from content_metadata table
    op.drop_column('content_metadata', 'created_at')
    op.drop_column('content_metadata', 'updated_at')