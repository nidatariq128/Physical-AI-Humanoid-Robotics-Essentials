"""
Unit tests for the RAG Agent API
"""
import pytest
import asyncio
from unittest.mock import Mock, patch, AsyncMock
from pydantic import ValidationError

from src.rag_agent_api.models.question import Question, APIResponse
from src.rag_agent_api.models.agent_response import AgentResponse
from src.rag_agent_api.models.retrieved_context import SourceCitation
from src.rag_agent_api.services.agent_orchestrator import AgentOrchestrator
from src.rag_agent_api.main import app
from fastapi.testclient import TestClient


@pytest.fixture
def test_client():
    """Create a test client for the FastAPI app"""
    return TestClient(app)


class TestQuestionModel:
    """Test the Question model validation"""

    def test_valid_question(self):
        """Test that a valid question is accepted"""
        question = Question(
            question="What is the meaning of life?",
            text_scope="Chapter 1",
            top_k=5
        )
        assert question.question == "What is the meaning of life?"
        assert question.text_scope == "Chapter 1"
        assert question.top_k == 5

    def test_question_required(self):
        """Test that question is required"""
        with pytest.raises(ValidationError):
            Question(question="")

    def test_question_whitespace_only(self):
        """Test that question with only whitespace raises error"""
        with pytest.raises(ValueError, match="Question cannot be empty or just whitespace"):
            Question(question="   ")

    def test_question_max_length(self):
        """Test that question exceeds max length"""
        long_question = "a" * 2001
        with pytest.raises(ValidationError):
            Question(question=long_question)

    def test_text_scope_max_length(self):
        """Test that text scope exceeds max length"""
        long_text_scope = "a" * 501
        with pytest.raises(ValidationError):
            Question(
                question="What is this?",
                text_scope=long_text_scope
            )

    def test_top_k_range(self):
        """Test that top_k is within valid range"""
        # Valid range
        Question(question="Test", top_k=1)
        Question(question="Test", top_k=20)

        # Invalid range
        with pytest.raises(ValidationError):
            Question(question="Test", top_k=0)
        with pytest.raises(ValidationError):
            Question(question="Test", top_k=21)


class TestAgentOrchestrator:
    """Test the Agent Orchestrator"""

    @pytest.mark.asyncio
    async def test_process_question_with_mocked_tools(self):
        """Test processing a question with mocked tools"""
        # Mock the Google Generative AI and config before creating the orchestrator
        with patch('src.rag_agent_api.services.agent_orchestrator.genai') as mock_genai, \
             patch('src.rag_agent_api.services.agent_orchestrator.config') as mock_config:
            # Setup mock config with API key
            mock_config.gemini_api_key = 'test-api-key'

            # Setup mock for genai.configure
            mock_genai.configure = Mock()

            # Create a mock model instance
            mock_model = Mock()
            mock_genai.GenerativeModel.return_value = mock_model

            # Mock the generate_content response
            mock_response = Mock()
            mock_response.text = "This is relevant content."
            mock_model.generate_content.return_value = mock_response

            orchestrator = AgentOrchestrator()

            # Mock the retrieval tool
            with patch('src.rag_agent_api.services.agent_orchestrator.retrieval_tool') as mock_retrieval:
                mock_retrieval.return_value = {
                    'results': [
                        {
                            'content': 'This is relevant content',
                            'metadata': {'source': 'test_source'},
                            'score': 0.9,
                            'id': 'test_id'
                        }
                    ],
                    'execution_time_ms': 100
                }

                result = await orchestrator.process_question(
                    question="What is this?",
                    text_scope="Chapter 1",
                    top_k=5
                )

                assert isinstance(result, AgentResponse)
                assert result.answer.lower() == "this is relevant content."
                assert len(result.citations) == 1
                assert result.grounded is True

    @pytest.mark.asyncio
    async def test_process_question_with_empty_retrieval(self):
        """Test processing a question when no results are found"""
        # Mock the Google Generative AI and config before creating the orchestrator
        with patch('src.rag_agent_api.services.agent_orchestrator.genai') as mock_genai, \
             patch('src.rag_agent_api.services.agent_orchestrator.config') as mock_config:
            # Setup mock config with API key
            mock_config.gemini_api_key = 'test-api-key'

            # Setup mock for genai.configure
            mock_genai.configure = Mock()

            # Create a mock model instance
            mock_model = Mock()
            mock_genai.GenerativeModel.return_value = mock_model

            # Mock the generate_content response
            mock_response = Mock()
            mock_response.text = "This is a test answer based on the context."
            mock_model.generate_content.return_value = mock_response

            orchestrator = AgentOrchestrator()

            # Mock the retrieval tool to return empty results
            with patch('src.rag_agent_api.services.agent_orchestrator.retrieval_tool') as mock_retrieval:
                mock_retrieval.return_value = {
                    'results': [],
                    'execution_time_ms': 100
                }

                result = await orchestrator.process_question(
                    question="What is this?",
                    text_scope=None,
                    top_k=5
                )

                assert isinstance(result, AgentResponse)
                assert "couldn't find any relevant information" in result.answer.lower()
                assert len(result.citations) == 0
                assert result.confidence_score == 0.0


class TestAPIEndpoints:
    """Test the API endpoints"""

    def test_health_endpoint(self, test_client):
        """Test the health endpoint"""
        response = test_client.get("/health")
        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert data["status"] == "healthy"
        assert "timestamp" in data

    def test_config_endpoint(self, test_client):
        """Test the config endpoint"""
        response = test_client.get("/config")
        assert response.status_code == 200
        data = response.json()
        assert "qdrant_collection" in data
        assert "default_top_k" in data
        assert "max_top_k" in data
        assert "agent_timeout_seconds" in data

    def test_question_endpoint_valid_request(self, test_client):
        """Test the question endpoint with a valid request"""
        from src.rag_agent_api.models.retrieved_context import SourceCitation

        # Mock the orchestrator to avoid calling real services
        with patch('src.rag_agent_api.main.AgentOrchestrator') as mock_orchestrator_class:
            mock_orchestrator = AsyncMock()
            mock_result = Mock()
            mock_result.answer = "This is a test answer"
            # Create SourceCitation objects instead of dictionaries
            mock_result.citations = [
                SourceCitation(
                    source="test_source",
                    content="test content",
                    score=0.9,
                    metadata={}
                )
            ]
            mock_result.grounded = True
            mock_result.execution_time_ms = 100.0
            mock_result.retrieved_context = []
            mock_result.confidence_score = 0.9

            mock_orchestrator.process_question.return_value = mock_result
            mock_orchestrator_class.return_value = mock_orchestrator

            request_data = {
                "question": "What is the meaning of life?",
                "text_scope": "Chapter 1",
                "top_k": 5
            }

            response = test_client.post("/question", json=request_data)
            assert response.status_code == 200
            data = response.json()
            assert "answer" in data
            assert "citations" in data
            assert "grounded" in data
            assert data["answer"] == "This is a test answer"

    def test_question_endpoint_invalid_request(self, test_client):
        """Test the question endpoint with an invalid request"""
        request_data = {
            "question": "",  # Empty question should fail validation
            "top_k": 25  # Above max value should fail validation
        }

        response = test_client.post("/question", json=request_data)
        assert response.status_code == 422  # Validation error

    def test_question_endpoint_missing_question(self, test_client):
        """Test the question endpoint with missing question"""
        request_data = {
            "text_scope": "Chapter 1",
            "top_k": 5
        }

        response = test_client.post("/question", json=request_data)
        assert response.status_code == 422  # Validation error


if __name__ == "__main__":
    pytest.main([__file__])