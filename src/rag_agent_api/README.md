# RAG Agent API

An API-driven RAG agent capable of answering questions grounded strictly in retrieved book content using Google Gemini AI and FastAPI.

## Overview

The RAG Agent API provides a FastAPI-based interface for question-answering using Retrieval-Augmented Generation (RAG) with Google Gemini AI. The system retrieves relevant context from a Qdrant vector database and generates responses that are strictly grounded in the retrieved information.

## Features

- FastAPI-based REST API for question-answering
- Integration with Google Gemini AI for agent orchestration
- Semantic retrieval from Qdrant vector database
- Response validation to ensure grounding in retrieved context
- Source citation generation
- Support for text scope queries
- Comprehensive logging and error handling
- Input validation and graceful degradation

## Architecture

The system is organized into several key components:

- **Models**: Pydantic models for request/response validation
- **Services**: Core business logic (agent orchestrator, response validation, error handling)
- **Tools**: Retrieval tool for semantic search
- **Utilities**: Helper functions (citation generation)
- **Logging**: Comprehensive event logging

## API Endpoints

### POST /question

Ask a question and receive a grounded response.

**Request Body:**
```json
{
  "question": "string (required, 1-2000 characters)",
  "text_scope": "string (optional, max 500 characters)",
  "top_k": "integer (optional, 1-20, default 5)"
}
```

**Response:**
```json
{
  "answer": "string",
  "citations": "array of citation objects",
  "retrieval_results_count": "integer",
  "grounded": "boolean",
  "execution_time_ms": "float"
}
```

### GET /health

Health check endpoint.

### GET /config

Return current configuration (excluding sensitive data).

## Configuration

The API uses the following environment variables:

### Required Variables
- `OPENROUTER_API_KEY`: OpenRouter API key (required)
- `QDRANT_URL`: Qdrant server URL (required)
- `QDRANT_API_KEY`: Qdrant API key (required)
- `QDRANT_COLLECTION_NAME`: Qdrant collection name (default: book_content_chunks)

### Optional Variables
- `OPENROUTER_MODEL`: Default OpenRouter model (default: xiaomi/mimo-v2-flash:free)
- `AGENT_MODEL`: Model used for agent operations (default: xiaomi/mimo-v2-flash:free)
- `DEFAULT_TOP_K`: Default number of results to retrieve (default: 5)
- `MAX_TOP_K`: Maximum number of results to retrieve (default: 20)
- `AGENT_TIMEOUT_SECONDS`: Agent timeout in seconds (default: 30)
- `MAX_TOKENS`: Maximum tokens (default: 2000)
- `TEMPERATURE`: Response temperature (default: 0.1)

### Getting Your OpenRouter API Key
1. Go to https://openrouter.ai/keys
2. Create an API key
3. Set the OPENROUTER_API_KEY environment variable with your key
4. The system is configured to use xiaomi/mimo-v2-flash:free model by default

## Usage

### Running the API

```bash
cd src/rag_agent_api
uvicorn main:app --host 0.0.0.0 --port 8000
```

### Example Request

```bash
curl -X POST "http://localhost:8000/question" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What are the main concepts in this book?",
    "text_scope": "Chapter 1",
    "top_k": 5
  }'
```

## Testing

Run the tests using pytest:

```bash
python -m pytest test_rag_agent_api.py -v
```

## Error Handling

The system implements comprehensive error handling:

- **Retrieval errors**: Graceful degradation when search fails
- **Rate limit errors**: Fallback responses when API limits are reached
- **Validation errors**: Proper grounding validation with fallback
- **Network errors**: Connectivity issue handling
- **Empty retrieval**: Appropriate responses when no relevant content is found

## Response Validation

The system validates that responses are grounded in the provided context by:
- Extracting key phrases from the response
- Checking if these phrases appear in the retrieved context
- Requiring at least 70% of key phrases to be supported by context
- Logging validation results for monitoring

## Security

- Input validation using Pydantic models
- No direct access to sensitive configuration in API responses
- Proper error message sanitization
- Rate limiting considerations (to be implemented in production)

## Performance

- Asynchronous processing for improved throughput
- Configurable timeout settings
- Efficient context retrieval from vector database
- Caching considerations (to be implemented in production)

## Development

The codebase follows these principles:

- Clear separation of concerns between models, services, tools, and utilities
- Comprehensive type hints for better maintainability
- Proper error handling and logging
- Test-driven development approach
- Configuration-driven behavior