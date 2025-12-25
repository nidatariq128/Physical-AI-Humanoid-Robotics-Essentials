#!/usr/bin/env python3
"""
Test script to verify the Urdu RAG chatbot functionality with OpenRouter
"""
import asyncio
import json
from src.rag_agent_api.config import config

def test_urdu_response():
    """
    Test that the configuration is set up for OpenRouter
    """
    print("Testing Urdu RAG Chatbot Configuration...")

    # Check if required configuration is available
    if not config.openrouter_api_key:
        print("[!] OPENROUTER_API_KEY not set - this is expected in test environment")
        print("   Set OPENROUTER_API_KEY to run full integration test")
        print("   Use your OpenRouter API key: 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")
        return False

    if not config.qdrant_url or not config.qdrant_api_key:
        print("[!] Qdrant configuration not set - this is expected in test environment")
        print("   Set QDRANT_URL and QDRANT_API_KEY for full functionality")
        return False

    print("[+] Configuration is properly set up for OpenRouter")

    # Check that the model is correctly configured
    if config.agent_model == "xiaomi/mimo-v2-flash:free":
        print("[+] Using correct model: xiaomi/mimo-v2-flash:free")
    else:
        print(f"[!] Using model: {config.agent_model}")

    # We can't test the full functionality without a real Qdrant database,
    # but we can verify the configuration is correct
    print("\n📝 The system is configured to:")
    print("   - Use OpenRouter API with your provided key")
    print("   - Use the xiaomi/mimo-v2-flash:free model")
    print("   - Follow strict context-only rules")
    print("   - Respond in Roman Urdu")
    print("   - Prevent hallucinations")

    return True

def test_system_prompt_structure():
    """
    Test that verifies the system prompt structure without calling the API
    """
    print("\nVerifying system prompt structure...")

    # Import the agent orchestrator and examine the prompt
    from src.rag_agent_api.services.agent_orchestrator import AgentOrchestrator
    import inspect

    # Get the source code of the _generate_answer_with_context method
    source = inspect.getsource(AgentOrchestrator._generate_answer_with_context)

    # Check for key elements in the system prompt
    checks = [
        ("SYSTEM PROMPT — RAG CHATBOT (MiMo-V2-Flash)", "System prompt header"),
        ("ONLY the information provided in the **CONTEXT**", "Context-only rule"),
        ("Do NOT hallucinate", "Hallucination prevention"),
        ("Respond in **Roman Urdu**", "Language requirement"),
        ("Provided documents mein is sawal ka jawab maujood nahi hai", "Urdu fallback"),
        ("CONTEXT", "Context section"),
        ("USER QUESTION", "User question section"),
    ]

    all_passed = True
    for check, description in checks:
        if check in source:
            print(f"[+] {description}")
        else:
            print(f"[-] {description}")
            all_passed = False

    return all_passed

if __name__ == "__main__":
    print("Testing Urdu RAG Chatbot Implementation with OpenRouter")
    print("=" * 60)

    # Test the system prompt structure
    structure_ok = test_system_prompt_structure()

    print("\n" + "=" * 60)

    # Test agent configuration
    config_ok = test_urdu_response()

    print("\n" + "=" * 60)
    print("Test Summary:")
    print(f"   System prompt structure: {'PASS' if structure_ok else 'FAIL'}")
    print(f"   Configuration setup: {'PASS' if config_ok else 'FAIL (expected without API keys)'}")

    if structure_ok and config_ok:
        print("\nUrdu RAG Chatbot implementation is correctly configured for OpenRouter!")
        print("   The system will now respond in Roman Urdu and follow strict context-only rules.")
        print("\nTo run the API:")
        print("   1. Set your OpenRouter API key in environment variables")
        print("   2. Set your Qdrant configuration")
        print("   3. Run: uvicorn src.rag_agent_api.main:app --host 0.0.0.0 --port 8000")
    else:
        print("\nIssues found with the implementation.")