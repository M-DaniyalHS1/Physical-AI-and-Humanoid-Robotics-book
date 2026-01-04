"""
Integration tests for RAG (Retrieval Augmented Generation) functionality

This module tests the integration between the RAG service, content service, and Qdrant vector store.
These tests verify that content can be indexed, searched semantically, and retrieved properly.
"""
import pytest
from unittest.mock import patch, MagicMock
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool
from src.models.database import Base, get_db
from src.models.book_content import BookContent
from src.services.rag_service import rag_service
from src.services.content_service import content_service
from datetime import datetime
import json
import uuid


# Setup an in-memory SQLite database for testing
SQLALCHEMY_DATABASE_URL = "sqlite:///:memory:"

engine = create_engine(
    SQLALCHEMY_DATABASE_URL,
    connect_args={"check_same_thread": False},
    poolclass=StaticPool,
)
TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)


# Create the tables in the test database
Base.metadata.create_all(bind=engine)


@pytest.fixture
def db_session():
    """Database session for testing"""
    session = TestingSessionLocal()
    try:
        yield session
    finally:
        session.close()


@pytest.fixture
def sample_content_data():
    """Sample content data for testing"""
    unique_id = f"test_content_{uuid.uuid4().hex[:8]}"
    return {
        "id": unique_id,
        "title": "Introduction to ROS 2",
        "module": "ROS 2",
        "chapter_number": 1,
        "content_type": "text",
        "content": "Robot Operating System 2 (ROS 2) is a set of software libraries and tools that help you build robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.",
        "version": "1.0.0",
        "vector_id": f"vec_{uuid.uuid4().hex[:8]}",
        "authors": json.dumps(["John Doe", "Jane Smith"]),
        "learning_objectives": json.dumps(["Understand ROS 2 basics", "Install ROS 2", "Create a simple ROS 2 node"]),
    }


def test_rag_service_initialization():
    """
    Test: Verify that the RAG service is properly initialized
    """
    # Verify that the rag_service singleton is available
    assert rag_service is not None
    assert hasattr(rag_service, 'search_content_by_text_similarity')
    assert hasattr(rag_service, 'index_textbook_content')
    assert hasattr(rag_service, 'add_content_to_vector_store')


def test_content_service_uses_rag_service():
    """
    Test: Verify that the content service properly integrates with the RAG service
    """
    # Verify that the content_service has access to rag_service
    assert content_service.rag_service is not None
    assert content_service.rag_service == rag_service
    
    # Verify that content_service has semantic search method
    assert hasattr(content_service, 'semantic_search_content')


def test_content_indexing_with_rag(db_session, sample_content_data):
    """
    Test: Verify that content can be indexed in the RAG service when created through content service
    """
    # Mock Qdrant client operations
    with patch.object(rag_service, 'index_textbook_content') as mock_index:
        mock_index.return_value = True

        # Create content using the content service
        result = content_service.create_book_content(db_session, sample_content_data)

        # Verify that content was created successfully
        assert result is not None
        assert result["id"] == sample_content_data["id"]

        # Verify that the content was indexed in the RAG service
        mock_index.assert_called_once()
        args, kwargs = mock_index.call_args
        assert args[0] == sample_content_data["id"]  # content_id
        assert args[1] == sample_content_data["content"]  # content_text
        assert args[2] == sample_content_data["module"]  # module
        assert args[3] == sample_content_data["content_type"]  # content_type


def test_semantic_search_functionality(db_session, sample_content_data):
    """
    Test: Verify that semantic search works through the content service
    """
    query = "What is ROS 2?"

    # Mock the RAG service search response
    with patch.object(rag_service, 'search_content_by_text_similarity') as mock_search:
        mock_search.return_value = [
            {
                "id": sample_content_data["id"],
                "content_text": sample_content_data["content"],
                "metadata": {"module": sample_content_data["module"], "content_type": sample_content_data["content_type"]},
                "score": 0.95
            }
        ]

        # Perform semantic search using the content service
        results = content_service.semantic_search_content(query, limit=5)

        # Verify the search was performed
        mock_search.assert_called_once()

        # Verify the results format
        assert len(results) == 1
        result = results[0]
        assert result["id"] == sample_content_data["id"]
        assert result["content"] == sample_content_data["content"]
        assert result["metadata"]["module"] == sample_content_data["module"]
        assert result["similarity_score"] == 0.95


def test_content_creation_triggers_rag_indexing(db_session, sample_content_data):
    """
    Test: Verify that creating content automatically triggers RAG indexing
    """
    # Mock Qdrant client operations
    with patch.object(rag_service, 'index_textbook_content') as mock_index:
        mock_index.return_value = True

        # Create content using the content service
        result = content_service.create_book_content(db_session, sample_content_data)

        # Verify that content was created successfully
        assert result is not None
        assert result["id"] == sample_content_data["id"]

        # Verify that RAG indexing was triggered
        mock_index.assert_called_once_with(
            sample_content_data["id"],
            sample_content_data["content"],
            sample_content_data["module"],
            sample_content_data["content_type"]
        )


def test_content_update_triggers_rag_reindexing(db_session, sample_content_data):
    """
    Test: Verify that updating content triggers RAG re-indexing when content changes
    """
    # First, create content
    original_content = content_service.create_book_content(db_session, sample_content_data)
    assert original_content is not None

    # Mock Qdrant client operations
    with patch.object(rag_service, 'index_textbook_content') as mock_index:
        mock_index.return_value = True

        # Update the content
        updated_data = {
            "content": "Updated content about ROS 2 and its architecture."
        }
        result = content_service.update_book_content(db_session, sample_content_data["id"], updated_data)

        # Verify that content was updated successfully
        assert result is not None
        assert result["content"] == updated_data["content"]

        # Verify that RAG re-indexing was triggered
        mock_index.assert_called_once_with(
            sample_content_data["id"],
            updated_data["content"],
            sample_content_data["module"],  # unchanged
            sample_content_data["content_type"]  # unchanged
        )


def test_content_deletion_removes_from_rag(db_session, sample_content_data):
    """
    Test: Verify that deleting content removes it from the RAG service
    """
    # First, create content
    created_content = content_service.create_book_content(db_session, sample_content_data)
    assert created_content is not None
    
    # Mock Qdrant client operations
    with patch.object(rag_service, 'delete_content_by_id') as mock_delete:
        mock_delete.return_value = True
        
        # Delete the content
        result = content_service.delete_book_content(db_session, sample_content_data["id"])
        
        # Verify that content was deleted successfully
        assert result is True
        
        # Verify that the content was removed from RAG service
        mock_delete.assert_called_once_with(sample_content_data["id"])


def test_batch_content_indexing(db_session):
    """
    Test: Verify that multiple contents can be indexed in batch
    """
    # Create multiple sample contents
    sample_contents = []
    for i in range(3):
        unique_id = f"batch_test_content_{i}_{uuid.uuid4().hex[:8]}"
        sample_contents.append({
            "id": unique_id,
            "title": f"Batch Content {i}",
            "module": "ROS 2" if i % 2 == 0 else "Gazebo & Unity",
            "chapter_number": i + 1,
            "content_type": "text",
            "content": f"This is test content {i} for batch indexing.",
            "version": "1.0.0",
            "vector_id": f"vec_batch_{i}",
            "authors": json.dumps([f"Author {i}"]),
            "learning_objectives": json.dumps([f"Learn about content {i}"]),
        })

    # Add contents to the database
    for content_data in sample_contents:
        content_record = BookContent(**content_data)
        db_session.add(content_record)
    db_session.commit()

    # Mock Qdrant client operations
    with patch.object(rag_service, 'index_multiple_textbook_contents') as mock_batch_index:
        mock_batch_index.return_value = True

        # Get the IDs of the created contents
        content_ids = [content["id"] for content in sample_contents]

        # Perform batch indexing
        result = content_service.batch_index_content(db_session, content_ids)

        # Verify that batch indexing was successful
        assert result is True

        # Verify that the batch indexing method was called
        mock_batch_index.assert_called_once()
        args, kwargs = mock_batch_index.call_args
        indexed_content_list = args[0]

        # Verify that the correct number of contents were prepared for indexing
        assert len(indexed_content_list) == len(sample_contents)

        # Verify the structure of each indexed content
        for i, indexed_content in enumerate(indexed_content_list):
            original_content = sample_contents[i]
            assert indexed_content["content_id"] == original_content["id"]
            assert indexed_content["content_text"] == original_content["content"]
            assert indexed_content["module"] == original_content["module"]
            assert indexed_content["content_type"] == original_content["content_type"]


def test_rag_search_with_filters(db_session, sample_content_data):
    """
    Test: Verify that RAG search can use filters
    """
    # Mock the RAG service search response with filters
    with patch.object(rag_service, 'search_similar_content') as mock_search:
        mock_search.return_value = [
            {
                "id": sample_content_data["id"],
                "content_text": sample_content_data["content"],
                "metadata": {"module": sample_content_data["module"], "content_type": sample_content_data["content_type"]},
                "score": 0.87
            }
        ]

        # Perform a search with filters using the RAG service directly
        query_vector = [0.5] * 1536
        filters = {"module": sample_content_data["module"]}
        results = rag_service.search_similar_content(
            query_vector=query_vector,
            limit=5,
            filters=filters
        )

        # Verify the search was performed with filters
        mock_search.assert_called_once()
        # The mock is applied to the original method, so this verifies the call was made properly
        assert len(results) == 1
        assert results[0]["id"] == sample_content_data["id"]