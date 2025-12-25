# Running the Urdu RAG Chatbot with OpenRouter

This guide explains how to run the Urdu RAG Chatbot using your OpenRouter API key and the xiaomi/mimo-v2-flash:free model.

## Prerequisites

1. **Python 3.8+** installed on your system
2. **Qdrant vector database** running and configured with your documents
3. **OpenRouter API key**: `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`

## Setup

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

### 2. Configure Environment Variables

Create a `.env` file in the root directory with the following content:

```env
# OpenRouter API configuration
OPENROUTER_API_KEY=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
OPENROUTER_MODEL=xiaomi/mimo-v2-flash:free
AGENT_MODEL=xiaomi/mimo-v2-flash:free

# Qdrant configuration (update with your actual values)
QDRANT_URL=your-qdrant-url
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=book_content_chunks

# Optional configurations
TEMPERATURE=0.1
MAX_TOKENS=2000
DEFAULT_TOP_K=5
```

## Running the API

### 1. Start the API Server

```bash
cd src
uvicorn rag_agent_api.main:app --host 0.0.0.0 --port 8000 --reload
```

### 2. Test the API

Once the server is running, you can test it with curl:

```bash
# Health check
curl http://localhost:8000/health

# Configuration check
curl http://localhost:8000/config

# Ask a question
curl -X POST "http://localhost:8000/question" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is artificial intelligence?",
    "top_k": 5
  }'
```

## API Usage

The API endpoint `/question` accepts the following parameters:

- `question`: (required) The question to answer (1-2000 characters)
- `text_scope`: (optional) Text scope to limit the search (max 500 characters)
- `top_k`: (optional) Number of results to retrieve (1-20, default 5)

Example request:
```json
{
  "question": "Explain machine learning concepts",
  "text_scope": "Chapter 3",
  "top_k": 5
}
```

## Expected Behavior

- The system will respond in **Roman Urdu** as specified in the system prompt
- Responses will be strictly grounded in the retrieved context from your Qdrant database
- If the answer isn't available in the context, it will respond with "Provided documents mein is sawal ka jawab maujood nahi hai."
- The system prevents hallucinations by enforcing context-only responses

## Troubleshooting

- If you get "OPENROUTER_API_KEY is not set" error: Make sure to set the environment variable
- If you get Qdrant errors: Verify your Qdrant URL, API key, and collection name
- If responses aren't in Roman Urdu: Check that the system prompt is properly configured

## Notes

- The `xiaomi/mimo-v2-flash:free` model is used as specified
- The system enforces strict context-only rules to prevent hallucinations
- Responses are validated to ensure they're grounded in the provided context