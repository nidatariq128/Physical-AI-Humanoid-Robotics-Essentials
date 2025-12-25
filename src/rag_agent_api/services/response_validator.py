"""
Response Validator for the RAG Agent API
Ensures responses are grounded in context and adds source citations
"""
import re
from typing import List, Dict, Any
from ..logger import logger


class ResponseValidator:
    """
    Ensure responses are grounded in context and add source citations
    """

    def __init__(self):
        pass

    def validate_response(self, response: str, context_list: List[str]) -> bool:
        """
        Validate that the response is grounded in the provided context.

        Args:
            response: The response to validate
            context_list: List of context strings that should support the response

        Returns:
            True if response is grounded in context, False otherwise
        """
        if not context_list:
            return len(response.strip()) == 0  # If no context, response should be empty

        # Check if response content is supported by context
        return self._check_contextual_support(response, context_list)

    def _check_contextual_support(self, response: str, context_list: List[str]) -> bool:
        """
        Check if the response content is supported by the provided context.

        Args:
            response: The response to validate
            context_list: List of context strings

        Returns:
            True if response is supported by context, False otherwise
        """
        if not response.strip():
            return True  # Empty response is valid when no context is available

        # Simple heuristic: check if key phrases from response appear in context
        response_lower = response.lower()
        context_combined = " ".join(context_list).lower()

        # Extract key phrases from response (simplified approach)
        key_phrases = self._extract_key_phrases(response_lower)

        # Check if most key phrases appear in context
        matched_phrases = 0
        total_phrases = len(key_phrases)

        for phrase in key_phrases:
            if phrase in context_combined:
                matched_phrases += 1

        # Consider grounded if at least 70% of key phrases are found in context
        if total_phrases == 0:
            return True

        grounding_ratio = matched_phrases / total_phrases
        is_grounded = grounding_ratio >= 0.7

        # Log validation result
        logger.log_validation_result(
            response=response,
            is_valid=is_grounded,
            validation_details={
                "matched_phrases": matched_phrases,
                "total_phrases": total_phrases,
                "grounding_ratio": grounding_ratio,
                "threshold": 0.7
            }
        )

        return is_grounded

    def _extract_key_phrases(self, text: str) -> List[str]:
        """
        Extract key phrases from text for validation.
        This is a simplified approach - in practice, you might use more sophisticated NLP.

        Args:
            text: Text to extract phrases from

        Returns:
            List of key phrases
        """
        # Remove extra whitespace and split into sentences
        sentences = re.split(r'[.!?]+', text)
        phrases = []

        for sentence in sentences:
            sentence = sentence.strip()
            if len(sentence) > 10:  # Only consider non-trivial sentences
                # Extract noun phrases or important content (simplified)
                words = sentence.split()
                if len(words) > 3:
                    # Take meaningful phrases (skip very short ones)
                    phrases.append(sentence)

        return phrases

    def detect_hallucination(self, response: str, context_list: List[str]) -> Dict[str, Any]:
        """
        Detect potential hallucinations in the response.

        Args:
            response: The response to check
            context_list: List of context strings

        Returns:
            Dictionary with hallucination detection results
        """
        if not context_list:
            return {
                "is_hallucinated": len(response.strip()) > 0,
                "confidence": 1.0,
                "details": "Response provided without any supporting context"
            }

        # Check for claims not supported by context
        unsupported_claims = self._find_unsupported_claims(response, context_list)

        return {
            "is_hallucinated": len(unsupported_claims) > 0,
            "confidence": min(len(unsupported_claims) / max(len(response.split()), 1), 1.0),
            "details": unsupported_claims
        }

    def _find_unsupported_claims(self, response: str, context_list: List[str]) -> List[str]:
        """
        Find specific parts of the response that are not supported by context.

        Args:
            response: The response to check
            context_list: List of context strings

        Returns:
            List of unsupported claims
        """
        # This is a simplified implementation
        # In practice, you might use semantic similarity or more advanced NLP
        context_combined = " ".join(context_list).lower()
        sentences = re.split(r'[.!?]+', response)

        unsupported = []
        for sentence in sentences:
            sentence = sentence.strip()
            if sentence and len(sentence) > 10:  # Skip very short sentences
                if sentence.lower() not in context_combined:
                    # Check if key parts of the sentence appear in context
                    words = sentence.lower().split()
                    if len(words) > 3:  # Only check non-trivial sentences
                        # Simplified check: if less than 50% of content words appear in context
                        context_words = set(context_combined.split())
                        sentence_words = set(words)
                        intersection = context_words.intersection(sentence_words)

                        if len(intersection) / len(sentence_words) < 0.3:  # Less than 30% overlap
                            unsupported.append(sentence)

        return unsupported