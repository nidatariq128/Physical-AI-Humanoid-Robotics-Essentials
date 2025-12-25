@echo off
echo Setting up environment variables for Urdu RAG Chatbot...

REM Set OpenRouter API key
set OPENROUTER_API_KEY=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
echo OPENROUTER_API_KEY set

REM Set Qdrant configuration (update these with your actual values)
set QDRANT_URL=your-qdrant-url-here
set QDRANT_API_KEY=your-qdrant-api-key-here
set QDRANT_COLLECTION_NAME=my-book
echo Qdrant configuration set

echo.
echo Environment variables configured successfully!
echo.
echo To run the chatbot API:
echo 1. Update QDRANT_URL and QDRANT_API_KEY with your actual values
echo 2. Run: uvicorn src.rag_agent_api.main:app --host 0.0.0.0 --port 8000
echo.
pause