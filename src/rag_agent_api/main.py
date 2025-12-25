"""
Main FastAPI application for the RAG Agent API
"""
from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from typing import Optional
import time
import logging

from .models.question import Question, APIResponse
from .services.agent_orchestrator import AgentOrchestrator
from .config import config
from .logger import logger


# Initialize FastAPI app
app = FastAPI(
    title="RAG Agent API",
    description="An API-driven RAG agent capable of answering questions grounded strictly in retrieved book content",
    version="1.0.0"
)

# Add CORS middleware to allow requests from web interfaces
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins (you can restrict this to specific domains in production)
    allow_credentials=True,
    allow_methods=["*"],  # Allow all methods (GET, POST, etc.)
    allow_headers=["*"],  # Allow all headers
    # Expose headers that can be accessed by the client
    expose_headers=["Access-Control-Allow-Origin"]
)


@app.on_event("startup")
async def startup_event():
    """Initialize the agent orchestrator on startup"""
    # Validate configuration
    if not config.validate():
        missing_fields = config.get_missing_fields()
        logger.logger.error(f"Configuration validation failed. Missing fields: {missing_fields}")
        raise RuntimeError(f"Configuration validation failed. Missing fields: {missing_fields}")

    logger.logger.info("RAG Agent API started successfully")


@app.post("/question", response_model=APIResponse)
async def ask_question(question_request: Question):
    """
    Endpoint to ask a question and receive an answer grounded in retrieved context
    """
    start_time = time.time()

    try:
        # Log the incoming request
        logger.log_question_request(
            question=question_request.question,
            text_scope=question_request.text_scope,
            top_k=question_request.top_k or config.default_top_k,
            execution_time_ms=0  # Will update after processing
        )

        # Initialize the agent orchestrator
        try:
            agent_orchestrator = AgentOrchestrator()
        except ValueError as e:
            logger.log_error("orchestrator_init", e, f"Question: {question_request.question}")
            raise HTTPException(status_code=500, detail=f"Configuration error: {str(e)}")

        # Process the question through the agent
        result = await agent_orchestrator.process_question(
            question=question_request.question,
            text_scope=question_request.text_scope,
            top_k=question_request.top_k or config.default_top_k
        )

        # Calculate total execution time
        total_time_ms = (time.time() - start_time) * 1000

        # Convert SourceCitation objects to dictionaries for API response
        citation_dicts = []
        for citation in result.citations:
            citation_dicts.append({
                "source": citation.source,
                "content": citation.content,
                "score": getattr(citation, 'score', 0.0),
                "metadata": getattr(citation, 'metadata', {})
            })

        # Create and return the API response
        api_response = APIResponse(
            answer=result.answer,
            citations=citation_dicts,
            retrieval_results_count=len(result.citations),  # Count of citations
            grounded=result.grounded,
            execution_time_ms=total_time_ms
        )

        # Log the response
        logger.log_agent_response(
            question=question_request.question,
            answer_length=len(result.answer),
            citations_count=len(result.citations),
            grounded=result.grounded,
            execution_time_ms=total_time_ms
        )

        return api_response

    except Exception as e:
        execution_time_ms = (time.time() - start_time) * 1000
        logger.log_error("question_endpoint", e, f"Question: {question_request.question}")

        # Log the request that failed
        logger.log_question_request(
            question=question_request.question,
            text_scope=question_request.text_scope,
            top_k=question_request.top_k or config.default_top_k,
            execution_time_ms=execution_time_ms
        )

        raise HTTPException(status_code=500, detail=f"Error processing question: {str(e)}")


@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "timestamp": time.time()}


# Additional utility endpoints can be added here
@app.get("/config")
async def get_config():
    """
    Return current configuration (excluding sensitive data)
    """
    return {
        "qdrant_collection": config.qdrant_collection_name,
        "default_top_k": config.default_top_k,
        "max_top_k": config.max_top_k,
        "agent_timeout_seconds": config.agent_timeout_seconds
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)