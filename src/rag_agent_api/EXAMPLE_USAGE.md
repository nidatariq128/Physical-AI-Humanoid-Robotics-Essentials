# RAG Agent API Usage Examples

## Running the API Server

```bash
cd src/rag_agent_api
uvicorn main:app --host 0.0.0.0 --port 8000
```

## API Usage Examples

### Basic Question

```bash
curl -X POST "http://localhost:8000/question" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What are the main concepts in this book?"
  }'
```

### Question with Text Scope

```bash
curl -X POST "http://localhost:8000/question" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is the definition of AI?",
    "text_scope": "Chapter 3: Artificial Intelligence Concepts",
    "top_k": 3
  }'
```

### Response Format

The API returns a response in the following format:

```json
{
  "answer": "The main concepts in this book include RAG systems, semantic search, and AI agents...",
  "citations": [
    {
      "source": "book_chapter_1.pdf",
      "content": "This is relevant content from the knowledge base...",
      "score": 0.95,
      "metadata": {}
    }
  ],
  "retrieval_results_count": 1,
  "grounded": true,
  "execution_time_ms": 1250.5
}
```

## Python Client Example

```python
import requests
import json

def ask_question(question, text_scope=None, top_k=5):
    url = "http://localhost:8000/question"

    payload = {
        "question": question,
        "text_scope": text_scope,
        "top_k": top_k
    }

    response = requests.post(url, json=payload)

    if response.status_code == 200:
        return response.json()
    else:
        print(f"Error: {response.status_code} - {response.text}")
        return None

# Example usage
result = ask_question(
    question="What are the key principles of RAG systems?",
    text_scope="Chapter 2",
    top_k=5
)

if result:
    print(f"Answer: {result['answer']}")
    print(f"Citations: {len(result['citations'])}")
    print(f"Grounded: {result['grounded']}")
```

## Health Check

```bash
curl -X GET "http://localhost:8000/health"
```

## Configuration Info

```bash
curl -X GET "http://localhost:8000/config"
```

## Error Handling

The API handles various error scenarios:

- **Invalid input**: Returns 422 with validation details
- **Retrieval errors**: Graceful degradation with fallback responses
- **Gemini API errors**: Proper error messages without exposing sensitive info
- **Empty results**: Appropriate responses when no relevant content is found