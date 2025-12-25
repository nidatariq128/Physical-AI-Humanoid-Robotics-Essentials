"""
Agent Orchestrator for the RAG Agent API
Coordinates the RAG process using OpenRouter API
"""
import asyncio
import time
import json
from typing import Optional, Dict, Any, List
from pydantic import BaseModel
import openai

from ..tools.retrieval_tool import retrieval_tool
from ..models.agent_response import AgentResponse
from ..services.response_validator import ResponseValidator
from ..services.error_handler import ErrorHandler
from ..utils.citation_generator import CitationGenerator
from ..config import config
from ..logger import logger


class AgentOrchestrator:
    """
    Coordinate the RAG process using Google Gemini AI
    """

    def __init__(self):
        # Check if API key is available before configuring
        if not config.openrouter_api_key:
            logger.logger.error("OPENROUTER_API_KEY is not set in environment variables")
            raise ValueError("OPENROUTER_API_KEY environment variable is required but not set")

        # Configure OpenAI client to use OpenRouter
        self.client = openai.OpenAI(
            api_key=config.openrouter_api_key,
            base_url="https://openrouter.ai/api/v1",
        )
        self.response_validator = ResponseValidator()
        self.error_handler = ErrorHandler()
        self.citation_generator = CitationGenerator()

    async def process_question(
        self,
        question: str,
        text_scope: Optional[str] = None,
        top_k: int = 5
    ) -> AgentResponse:
        """
        Process a question through the agent and return a validated response

        Args:
            question: The question to answer
            text_scope: Optional text scope to limit the search
            top_k: Number of results to retrieve

        Returns:
            AgentResponse with answer, citations, and validation
        """
        start_time = time.time()

        try:
            # Prepare filters based on text scope if provided
            filters = {}
            if text_scope:
                # Filter by text scope - this would match against document sections, chapters, or other metadata
                filters = {"text_scope": text_scope}  # Filter by document section/chapter

            # First, retrieve relevant information from the knowledge base
            retrieval_result = retrieval_tool({
                "query": question,
                "top_k": top_k,
                "filters": filters
            })

            # Extract the retrieved results
            retrieved_results = retrieval_result['results']
            retrieval_time = retrieval_result['execution_time_ms']

            # If no results found, handle gracefully
            if not retrieved_results:
                empty_result = self.error_handler.handle_empty_retrieval(question)
                execution_time = (time.time() - start_time) * 1000
                return AgentResponse(
                    answer=empty_result["answer"],
                    citations=empty_result["citations"],
                    grounded=empty_result["grounded"],
                    execution_time_ms=execution_time,
                    confidence_score=0.0
                )

            # Generate citations from retrieved results
            citations = self.citation_generator.generate_citations(retrieved_results)

            # Prepare context from retrieved results
            context_content = []
            for result in retrieved_results:
                content = result.get('content', '')
                context_content.append(content)

            # Combine context for the agent
            context_str = "\n\n".join(context_content)

            # Generate the answer using Gemini with the retrieved context
            answer = await self._generate_answer_with_context(question, context_str)

            # Validate that the response is grounded in the context
            is_grounded = self.response_validator.validate_response(answer, context_content)

            # Calculate confidence based on citation scores
            avg_score = sum(c['score'] for c in retrieved_results if 'score' in c) / len(retrieved_results) if retrieved_results else 0.0

            # Log the retrieval operation
            logger.log_retrieval_operation(
                query_text=question,
                results_count=len(retrieved_results),
                execution_time_ms=retrieval_time,
                filters=filters
            )

            return AgentResponse(
                answer=answer,
                citations=citations,
                grounded=is_grounded,
                retrieved_context=retrieved_results,
                execution_time_ms=(time.time() - start_time) * 1000,
                confidence_score=avg_score
            )

        except Exception as e:
            execution_time = (time.time() - start_time) * 1000
            logger.log_error("agent_orchestration", e, f"Question: {question}")

            # Use error handler to get appropriate response
            error_result = self.error_handler.handle_error(e, "agent_orchestration", {"question": question})

            # Return a safe fallback response
            return AgentResponse(
                answer=error_result["fallback_response"],
                citations=[],
                grounded=False,
                execution_time_ms=execution_time,
                confidence_score=0.0
            )

    async def _generate_answer_with_context(self, question: str, context: str) -> str:
        """
        Generate an answer using the question and context.
        Uses OpenRouter API to generate a response based on the retrieved context.
        """
        # Create a system prompt with strict rules for context-only responses and Roman Urdu output
        system_prompt = """# SYSTEM PROMPT — RAG CHATBOT (MiMo-V2-Flash)

You are an AI assistant integrated into a Retrieval-Augmented Generation (RAG) system.

Your responsibility is to answer the user's question using **ONLY** the information provided in the **CONTEXT** section below.

## STRICT RULES
- Do NOT use any external or prior knowledge.
- Do NOT hallucinate, guess, or fabricate information.
- If the answer is not clearly available in the context, respond with:
  **"Provided documents mein is sawal ka jawab maujood nahi hai."**
- Keep responses clear, concise, and accurate.
- If multiple points exist, respond in bullet points.
- If the question is unclear, ask for clarification instead of assuming.

## LANGUAGE
- Respond in **Roman Urdu** unless the user explicitly asks for another language.

## CONTEXT
The following text has been retrieved from trusted documents. Use it as your **only source of truth:

{context}

## USER QUESTION
{question}

## RESPONSE FORMAT
- Direct answer first
- Bullet points if needed
- No unnecessary explanation"""

        # Format the system prompt with the actual context and question
        formatted_prompt = system_prompt.format(context=context, question=question)

        # Generate content using OpenRouter
        try:
            logger.logger.info(f"Sending prompt to OpenRouter API (length: {len(formatted_prompt)} chars)")
            response = self.client.chat.completions.create(
                model=config.agent_model,
                messages=[
                    {"role": "user", "content": formatted_prompt}
                ],
                temperature=config.temperature,  # Use configured temperature
                max_tokens=config.max_tokens,  # Use configured max tokens
            )

            # Check if response has valid content
            if response and response.choices and response.choices[0].message and response.choices[0].message.content:
                answer = response.choices[0].message.content
                logger.logger.info(f"OpenRouter API returned response (length: {len(answer)} chars)")
            else:
                logger.logger.warning("OpenRouter API returned empty response")
                answer = "Provided documents mein is sawal ka jawab maujood nahi hai."
        except Exception as e:
            # Handle any OpenRouter API errors
            logger.log_error("openrouter_generation", e, f"Question: {question}")
            logger.logger.error(f"OpenRouter API error details: {str(e)}")
            answer = "I encountered an issue generating a response. Please try again later."

        # Simulate async operation
        await asyncio.sleep(0.01)

        return answer