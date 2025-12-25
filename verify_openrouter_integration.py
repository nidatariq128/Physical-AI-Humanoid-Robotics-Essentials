#!/usr/bin/env python3
"""
Simple verification script to check that the OpenRouter changes were applied correctly
"""
import os

def check_file_contains(filepath, search_strings):
    """Check if file contains specific strings"""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()

        results = {}
        for search_string in search_strings:
            results[search_string] = search_string in content

        return results, content
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
        return {}, ""

def verify_changes():
    print("Verifying OpenRouter Integration Changes")
    print("=" * 50)

    # Check 1: Configuration changes
    print("\n[1] Checking config.py changes...")
    config_searches = [
        "openrouter_api_key",
        "OPENROUTER_API_KEY",
        "xiaomi/mimo-v2-flash:free"
    ]
    config_results, config_content = check_file_contains(
        "src/rag_agent_api/config.py",
        config_searches
    )

    for search, found in config_results.items():
        status = "[+] FOUND" if found else "[-] MISSING"
        print(f"   {status}: {search}")

    # Check 2: Agent orchestrator changes
    print("\n[2] Checking agent_orchestrator.py changes...")
    orchestrator_searches = [
        "openrouter",
        "openai.OpenAI",
        "base_url=\"https://openrouter.ai/api/v1\"",
        "chat.completions.create",
        "SYSTEM PROMPT — RAG CHATBOT (MiMo-V2-Flash)"
    ]
    orchestrator_results, orchestrator_content = check_file_contains(
        "src/rag_agent_api/services/agent_orchestrator.py",
        orchestrator_searches
    )

    for search, found in orchestrator_results.items():
        status = "[+] FOUND" if found else "[-] MISSING"
        print(f"   {status}: {search}")

    # Check 3: Requirements changes
    print("\n[3] Checking requirements.txt changes...")
    requirements_searches = [
        "openai>=1.0.0",
        "google-generativeai"
    ]
    requirements_results, requirements_content = check_file_contains(
        "requirements.txt",
        requirements_searches
    )

    for search, found in requirements_results.items():
        # For requirements, we want openai but NOT google-generativeai
        if search == "google-generativeai":
            status = "[-] CORRECTLY REMOVED" if not found else "[+] STILL PRESENT"
        else:
            status = "[+] FOUND" if found else "[-] MISSING"
        print(f"   {status}: {search}")

    # Check 4: Environment example file
    print("\n[4] Checking .env.example changes...")
    env_searches = [
        "OPENROUTER_API_KEY",
        "GEMINI_API_KEY"
    ]
    env_results, env_content = check_file_contains(
        ".env.example",
        env_searches
    )

    for search, found in env_results.items():
        status = "[+] FOUND" if found else "[-] MISSING"
        print(f"   {status}: {search}")

    # Summary
    print("\n" + "=" * 50)
    print("SUMMARY:")

    all_config_found = all(config_results.values())
    all_orchestrator_found = all(orchestrator_results.values())
    openai_found = requirements_results.get("openai>=1.0.0", False)
    google_gemini_removed = not requirements_results.get("google-generativeai", True)
    openrouter_in_env = env_results.get("OPENROUTER_API_KEY", False)

    print(f"   Config changes: {'[+] COMPLETE' if all_config_found else '[-] INCOMPLETE'}")
    print(f"   Orchestrator changes: {'[+] COMPLETE' if all_orchestrator_found else '[-] INCOMPLETE'}")
    print(f"   OpenAI dependency: {'[+] ADDED' if openai_found else '[-] MISSING'}")
    print(f"   Gemini dependency removed: {'[+] YES' if google_gemini_removed else '[-] NO'}")
    print(f"   OpenRouter in env example: {'[+] YES' if openrouter_in_env else '[-] NO'}")

    overall_success = all_config_found and all_orchestrator_found and openai_found and google_gemini_removed and openrouter_in_env

    print(f"\nOVERALL: {'[+] SUCCESS - OpenRouter integration complete!' if overall_success else '[-] INCOMPLETE - Some changes missing'}")

    if overall_success:
        print(f"\nTo use your API key (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):")
        print("   1. Set OPENROUTER_API_KEY environment variable")
        print("   2. Ensure Qdrant is configured")
        print("   3. Run: uvicorn src.rag_agent_api.main:app --host 0.0.0.0 --port 8000")

if __name__ == "__main__":
    verify_changes()