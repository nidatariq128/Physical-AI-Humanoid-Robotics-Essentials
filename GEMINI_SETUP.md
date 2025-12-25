#!/usr/bin/env python3
"""
Documentation: How to run the RAG Agent API with Gemini
"""

print("## Setting up Google Gemini API Integration")

print("\n### 1. Environment Variables Setup")
print("You need to set the following environment variables:")
print("```bash")
print("export GEMINI_API_KEY='your-actual-gemini-api-key'")
print("export GEMINI_MODEL='gemini-1.5-pro'  # or gemini-pro")
print("export QDRANT_URL='your-qdrant-url'")
print("export QDRANT_API_KEY='your-qdrant-api-key'")
print("export QDRANT_COLLECTION_NAME='your-collection-name'")
print("```")

print("\n### 2. Get a Google Gemini API Key")
print("1. Go to https://makersuite.google.com/app/apikey")
print("2. Create an API key for your Google Cloud project")
print("3. Make sure the Generative Language API is enabled")
print("4. Set the GEMINI_API_KEY environment variable with your key")

print("\n### 3. Start the API Server")
print("```bash")
print("cd src")
print("uvicorn rag_agent_api.main:app --host 0.0.0.0 --port 8000 --reload")
print("```")

print("\n### 4. Test the API")
print("```bash")
print("# Health check")
print("curl http://localhost:8000/health")
print("")
print("# Configuration info")
print("curl http://localhost:8000/config")
print("")
print("# Ask a question (replace with your actual question)")
print("curl -X POST http://localhost:8000/question \\")
print("  -H 'Content-Type: application/json' \\")
print("  -d '{")
print('    "question": "What is artificial intelligence?",')
print('    "top_k": 5')
print("  }'")
print("```")

print("\n### 5. Troubleshooting")
print("- If you get 'GEMINI_API_KEY is not set' error: Make sure to set the environment variable")
print("- If you get API quota errors: Check your Google Cloud billing and quotas")
print("- If you get Qdrant errors: Make sure your vector database is properly configured")
print("- Check the logs for detailed error messages")

print("\n### 6. Expected Behavior")
print("When properly configured with a valid GEMINI_API_KEY:")
print("- The API will retrieve relevant context from your Qdrant database")
print("- Gemini will generate answers based on the retrieved context")
print("- Answers will be grounded in the provided context with citations")
print("- The system will handle errors gracefully with appropriate fallbacks")