"""
Unit tests for the RAG Service

This module contains unit tests for the RAG (Retrieval Augmented Generation) service,
testing the new functionality added for indexing and syncing content between
PostgreSQL and Qdrant vector store.
"""
import pytest
from unittest.mock import MagicMock, patch, call
from sqlalchemy.orm import Session
from src.services.rag_service import rag_service, RAGService
from src.models.book_content import BookContent
from datetime import datetime


class TestRAGService:
    """Test class for RAG Service methods"""

    def test_generate_embedding_with_openai_client(self):
        """Test generating embedding with OpenAI client available"""
        # Create a RAGService instance with mocked OpenAI client
        rag_service = RAGService()
        rag_service.openai_client = MagicMock()
        
        # Mock the embeddings response
        mock_response = MagicMock()
        mock_response.data = [MagicMock()]
        mock_response.data[0].embedding = [0.1, 0.2, 0.3]
        rag_service.openai_client.embeddings.create.return_value = mock_response
        
        # Test the method
        text = "Test content for embedding"
        result = rag_service.generate_embedding(text)
        
        # Verify the result
        assert result == [0.1, 0.2, 0.3]
        rag_service.openai_client.embeddings.create.assert_called_once_with(
            input=text,
            model="text-embedding-ada-002"
        )

    def test_generate_embedding_with_openai_client_failure(self):
        """Test generating embedding when OpenAI API call fails"""
        # Create a RAGService instance with mocked OpenAI client
        rag_service = RAGService()
        rag_service.openai_client = MagicMock()
        
        # Mock the embeddings to raise an exception
        rag_service.openai_client.embeddings.create.side_effect = Exception("API Error")
        
        # Test the method
        text = "Test content for embedding"
        result = rag_service.generate_embedding(text)
        
        # Verify the result is a mock embedding
        assert len(result) == 1536  # Default embedding size
        assert all(isinstance(x, float) for x in result)
        rag_service.openai_client.embeddings.create.assert_called_once_with(
            input=text,
            model="text-embedding-ada-002"
        )

    def test_generate_embedding_without_openai_client(self):
        """Test generating embedding when OpenAI client is not available"""
        # Create a RAGService instance without OpenAI client
        rag_service = RAGService()
        rag_service.openai_client = None
        
        # Test the method
        text = "Test content for embedding"
        result = rag_service.generate_embedding(text)
        
        # Verify the result is a mock embedding
        assert len(result) == 1536  # Default embedding size
        assert all(isinstance(x, float) for x in result)

    @patch('src.services.rag_service.BookContent')
    @patch('src.services.rag_service.QDRANT_AVAILABLE', True)
    def test_index_all_content_success(self, mock_book_content):
        """Test indexing all content from PostgreSQL to Qdrant"""
        # Create a RAGService instance with mocked Qdrant client
        rag_service = RAGService()
        rag_service.client = MagicMock()
        rag_service.client.upsert = MagicMock()
        
        # Mock database session
        mock_db = MagicMock(spec=Session)
        
        # Create mock content objects
        mock_content1 = MagicMock(spec=BookContent)
        mock_content1.id = "content-1"
        mock_content1.title = "Test Title 1"
        mock_content1.content = "Test content 1"
        mock_content1.module = "ROS 2"
        mock_content1.chapter_number = 1
        mock_content1.content_type = "text"
        mock_content1.version = "1.0.0"
        mock_content1.created_at = datetime.now()
        mock_content1.updated_at = datetime.now()
        
        mock_content2 = MagicMock(spec=BookContent)
        mock_content2.id = "content-2"
        mock_content2.title = "Test Title 2"
        mock_content2.content = "Test content 2"
        mock_content2.module = "Gazebo & Unity"
        mock_content2.chapter_number = 2
        mock_content2.content_type = "video"
        mock_content2.version = "1.0.0"
        mock_content2.created_at = datetime.now()
        mock_content2.updated_at = datetime.now()
        
        # Mock the database query
        mock_db.query.return_value.count.return_value = 2
        mock_db.query.return_value.offset.return_value.limit.return_value.all.return_value = [
            mock_content1, mock_content2
        ]
        
        # Mock the generate_embedding method
        with patch.object(rag_service, 'generate_embedding', return_value=[0.1, 0.2, 0.3]):
            # Test the method
            result = rag_service.index_all_content(mock_db, batch_size=100)
            
            # Verify the result
            assert result is True
            # Verify upsert was called
            assert rag_service.client.upsert.call_count == 1
            
            # Check the call arguments
            call_args = rag_service.client.upsert.call_args
            assert call_args[1]['collection_name'] == rag_service.collection_name
            points = call_args[1]['points']
            assert len(points) == 2

    @patch('src.services.rag_service.BookContent')
    @patch('src.services.rag_service.QDRANT_AVAILABLE', False)
    def test_index_all_content_qdrant_not_available(self, mock_book_content):
        """Test indexing all content when Qdrant is not available"""
        # Create a RAGService instance
        rag_service = RAGService()
        
        # Mock database session
        mock_db = MagicMock(spec=Session)
        
        # Test the method
        result = rag_service.index_all_content(mock_db)
        
        # Verify the result
        assert result is False

    @patch('src.services.rag_service.BookContent')
    @patch('src.services.rag_service.QDRANT_AVAILABLE', True)
    def test_index_single_content_success(self, mock_book_content):
        """Test indexing a single content item"""
        # Create a RAGService instance with mocked Qdrant client
        rag_service = RAGService()
        rag_service.client = MagicMock()
        rag_service.client.upsert = MagicMock()
        
        # Mock database session
        mock_db = MagicMock(spec=Session)
        
        # Create mock content object
        mock_content = MagicMock(spec=BookContent)
        mock_content.id = "content-1"
        mock_content.title = "Test Title 1"
        mock_content.content = "Test content 1"
        mock_content.module = "ROS 2"
        mock_content.chapter_number = 1
        mock_content.content_type = "text"
        mock_content.version = "1.0.0"
        mock_content.created_at = datetime.now()
        mock_content.updated_at = datetime.now()
        
        # Mock the database query
        mock_db.query.return_value.filter.return_value.first.return_value = mock_content
        
        # Mock the generate_embedding method
        with patch.object(rag_service, 'generate_embedding', return_value=[0.1, 0.2, 0.3]):
            # Test the method
            result = rag_service.index_single_content(mock_db, "content-1")
            
            # Verify the result
            assert result is True
            # Verify upsert was called
            rag_service.client.upsert.assert_called_once()
            
            # Check the call arguments
            call_args = rag_service.client.upsert.call_args
            assert call_args[1]['collection_name'] == rag_service.collection_name
            points = call_args[1]['points']
            assert len(points) == 1
            assert points[0].id == "content-1"

    @patch('src.services.rag_service.BookContent')
    @patch('src.services.rag_service.QDRANT_AVAILABLE', True)
    def test_index_single_content_not_found(self, mock_book_content):
        """Test indexing a single content item that doesn't exist"""
        # Create a RAGService instance
        rag_service = RAGService()
        
        # Mock database session
        mock_db = MagicMock(spec=Session)
        
        # Mock the database query to return None
        mock_db.query.return_value.filter.return_value.first.return_value = None
        
        # Test the method
        result = rag_service.index_single_content(mock_db, "non-existent-content")
        
        # Verify the result
        assert result is False

    @patch('src.services.rag_service.BookContent')
    @patch('src.services.rag_service.QDRANT_AVAILABLE', False)
    def test_index_single_content_qdrant_not_available(self, mock_book_content):
        """Test indexing a single content item when Qdrant is not available"""
        # Create a RAGService instance
        rag_service = RAGService()
        
        # Mock database session
        mock_db = MagicMock(spec=Session)
        
        # Test the method
        result = rag_service.index_single_content(mock_db, "content-1")
        
        # Verify the result
        assert result is False

    @patch('src.services.rag_service.BookContent')
    @patch('src.services.rag_service.QDRANT_AVAILABLE', True)
    def test_sync_content_changes_update(self, mock_book_content):
        """Test syncing content changes when content exists in DB"""
        # Create a RAGService instance
        rag_service = RAGService()
        
        # Mock database session
        mock_db = MagicMock(spec=Session)
        
        # Create mock content object
        mock_content = MagicMock(spec=BookContent)
        mock_content.id = "content-1"
        mock_content.title = "Updated Title"
        mock_content.content = "Updated content"
        mock_content.module = "ROS 2"
        mock_content.chapter_number = 1
        mock_content.content_type = "text"
        mock_content.version = "1.0.0"
        mock_content.created_at = datetime.now()
        mock_content.updated_at = datetime.now()
        
        # Mock the database query
        mock_db.query.return_value.filter.return_value.first.return_value = mock_content
        
        # Mock the index_single_content method
        with patch.object(rag_service, 'index_single_content', return_value=True):
            # Test the method
            result = rag_service.sync_content_changes(mock_db, "content-1")
            
            # Verify the result
            assert result is True
            # Verify index_single_content was called
            rag_service.index_single_content.assert_called_once_with(mock_db, "content-1")

    @patch('src.services.rag_service.QDRANT_AVAILABLE', True)
    def test_sync_content_changes_delete(self):
        """Test syncing content changes when content doesn't exist in DB (should delete from Qdrant)"""
        # Create a RAGService instance
        rag_service = RAGService()
        rag_service.client = MagicMock()
        
        # Mock database session
        mock_db = MagicMock(spec=Session)
        
        # Mock the database query to return None
        mock_db.query.return_value.filter.return_value.first.return_value = None
        
        # Mock the delete_content_by_id method
        with patch.object(rag_service, 'delete_content_by_id', return_value=True):
            # Test the method
            result = rag_service.sync_content_changes(mock_db, "content-1")
            
            # Verify the result
            assert result is True
            # Verify delete_content_by_id was called
            rag_service.delete_content_by_id.assert_called_once_with("content-1")

    @patch('src.services.rag_service.QDRANT_AVAILABLE', False)
    def test_sync_content_changes_qdrant_not_available(self):
        """Test syncing content changes when Qdrant is not available"""
        # Create a RAGService instance
        rag_service = RAGService()
        
        # Mock database session
        mock_db = MagicMock(spec=Session)
        
        # Test the method
        result = rag_service.sync_content_changes(mock_db, "content-1")
        
        # Verify the result
        assert result is False