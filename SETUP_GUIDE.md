# Environment Setup Guide for Urdu RAG Chatbot

## Current Status
✅ Your Qdrant database is working and has 17 data points in the 'my-book' collection
✅ System prompt is correctly configured with Urdu language and context rules
❌ Environment variables are not set

## Required Environment Variables

### 1. OpenRouter API Key (your free key)
```
OPENROUTER_API_KEY=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
```

### 2. Qdrant Configuration (you need to provide these)
You need to get these from your Qdrant setup:

**If using Qdrant Cloud:**
```
QDRANT_URL=https://your-cluster-url.qdrant.tech:6333
QDRANT_API_KEY=your-actual-api-key
QDRANT_COLLECTION_NAME=my-book
```

**If using local Qdrant:**
```
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY= (leave empty for local)
QDRANT_COLLECTION_NAME=my-book
```

## How to Set Environment Variables

### Option 1: Command Line (temporary)
```bash
# Windows Command Prompt
set OPENROUTER_API_KEY=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
set QDRANT_URL=your-actual-url
set QDRANT_API_KEY=your-actual-api-key
set QDRANT_COLLECTION_NAME=my-book

# Then run the API
uvicorn src.rag_agent_api.main:app --host 0.0.0.0 --port 8000
```

### Option 2: Create a .env file
Create a `.env` file in the project root directory:
```
OPENROUTER_API_KEY=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
QDRANT_URL=your-actual-url
QDRANT_API_KEY=your-actual-api-key
QDRANT_COLLECTION_NAME=my-book
```

### Option 3: PowerShell (temporary)
```powershell
$env:OPENROUTER_API_KEY="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"
$env:QDRANT_URL="your-actual-url"
$env:QDRANT_API_KEY="your-actual-api-key"
$env:QDRANT_COLLECTION_NAME="my-book"

uvicorn src.rag_agent_api.main:app --host 0.0.0.0 --port 8000
```

## Testing Your Setup

After setting the environment variables:

1. Run the diagnostic again:
```bash
python diagnose_chatbot.py
```

2. All checks should show [+] OK

3. Start the API:
```bash
uvicorn src.rag_agent_api.main:app --host 0.0.0.0 --port 8000
```

4. Test with a query:
```bash
curl -X POST "http://localhost:8000/question" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is this book about?",
    "top_k": 5
  }'
```

## Troubleshooting

If you still have issues:

1. **Check Qdrant connection:**
   ```bash
   python -c "from qdrant_client import QdrantClient; c=QdrantClient(url='your-url', api_key='your-key'); print(c.get_collections())"
   ```

2. **Verify your Qdrant credentials:**
   - Go to your Qdrant dashboard
   - Copy the exact URL and API key
   - Make sure the collection 'my-book' exists

3. **Check OpenRouter key:**
   - Verify the API key format starts with 'sk-or-v1-'
   - Make sure there are no extra spaces or characters

## Next Steps

Once you set the environment variables:
1. Your chatbot will connect to your existing Qdrant database
2. It will retrieve the 17 data points you already have
3. It will respond in Roman Urdu with context-grounded answers
4. It will use the free xiaomi/mimo-v2-flash:free model

The good news is: Your data is already there and ready to be used! You just need to connect the API keys.