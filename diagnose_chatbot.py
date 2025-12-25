#!/usr/bin/env python3
"""
Diagnostic script to check why the chatbot is not fetching data
"""
import os
import sys
from dotenv import load_dotenv
sys.path.insert(0, '.')

# Load environment variables from .env file
load_dotenv()

def check_environment():
    """Check environment variables"""
    print("Checking Environment Configuration...")
    print("=" * 50)

    # Check OpenRouter API key
    openrouter_key = os.getenv('OPENROUTER_API_KEY', '')
    print(f"OPENROUTER_API_KEY: {'[+] SET' if openrouter_key else '[-] NOT SET'}")
    if openrouter_key:
        print(f"  Key starts with: {openrouter_key[:10]}...")

    # Check Qdrant configuration
    qdrant_url = os.getenv('QDRANT_URL', '')
    qdrant_api_key = os.getenv('QDRANT_API_KEY', '')
    qdrant_collection = os.getenv('QDRANT_COLLECTION_NAME', '')

    print(f"QDRANT_URL: {'[+] SET' if qdrant_url else '[-] NOT SET'}")
    print(f"QDRANT_API_KEY: {'[+] SET' if qdrant_api_key else '[-] NOT SET'}")
    print(f"QDRANT_COLLECTION_NAME: {'[+] SET' if qdrant_collection else '[-] NOT SET'}")

    if qdrant_collection:
        print(f"  Collection: {qdrant_collection}")

    return {
        'openrouter_key': bool(openrouter_key),
        'qdrant_url': bool(qdrant_url),
        'qdrant_api_key': bool(qdrant_api_key),
        'qdrant_collection': bool(qdrant_collection)
    }

def check_config_file():
    """Check the configuration file"""
    print("\nChecking Configuration File...")
    print("=" * 50)

    try:
        from src.rag_agent_api.config import config
        print(f"Config loaded: [+] YES")
        print(f"OpenRouter API key set: {'[+] YES' if config.openrouter_api_key else '[-] NO'}")
        print(f"Qdrant URL set: {'[+] YES' if config.qdrant_url else '[-] NO'}")
        print(f"Qdrant API key set: {'[+] YES' if config.qdrant_api_key else '[-] NO'}")
        print(f"Qdrant collection: {config.qdrant_collection_name}")
        print(f"Agent model: {config.agent_model}")

        validation_result = config.validate()
        print(f"Configuration validation: {'[+] PASSED' if validation_result else '[-] FAILED'}")

        if not validation_result:
            missing_fields = config.get_missing_fields()
            print(f"Missing fields: {missing_fields}")

        return validation_result
    except Exception as e:
        print(f"[-] Error loading config: {e}")
        return False

def check_retrieval_tool():
    """Check if retrieval tool can connect to Qdrant"""
    print("\nChecking Qdrant Connection...")
    print("=" * 50)

    try:
        # Try to import and test Qdrant connection
        from qdrant_client import QdrantClient

        from src.rag_agent_api.config import config

        if not config.qdrant_url or not config.qdrant_api_key:
            print("[-] Qdrant configuration not complete")
            return False

        # Test connection
        client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            timeout=10
        )

        # Try to list collections to test connection
        collections = client.get_collections()
        print(f"[+] Connected to Qdrant")
        print(f"Available collections: {[col.name for col in collections.collections]}")

        # Check if the expected collection exists
        collection_names = [col.name for col in collections.collections]
        if config.qdrant_collection_name in collection_names:
            print(f"[+] Collection '{config.qdrant_collection_name}' exists")

            # Check collection size
            collection_info = client.get_collection(config.qdrant_collection_name)
            print(f"Collection points count: {collection_info.points_count}")
            return True
        else:
            print(f"[-] Collection '{config.qdrant_collection_name}' does not exist")
            return False

    except Exception as e:
        print(f"[-] Error connecting to Qdrant: {e}")
        return False

def check_system_prompt():
    """Check if system prompt is properly formatted"""
    print("\nChecking System Prompt...")
    print("=" * 50)

    try:
        from src.rag_agent_api.services.agent_orchestrator import AgentOrchestrator
        import inspect

        source = inspect.getsource(AgentOrchestrator._generate_answer_with_context)

        # Check for required elements
        has_context_rule = "only source of truth" in source
        has_no_hallucinate = "Do NOT hallucinate" in source
        has_roman_urdu = "Respond in **Roman Urdu**" in source
        has_urdu_fallback = "Provided documents mein is sawal ka jawab maujood nahi hai" in source

        print(f"[+] Context-only rule: {'YES' if has_context_rule else 'NO'}")
        print(f"[+] No hallucination rule: {'YES' if has_no_hallucinate else 'NO'}")
        print(f"[+] Roman Urdu requirement: {'YES' if has_roman_urdu else 'NO'}")
        print(f"[+] Urdu fallback: {'YES' if has_urdu_fallback else 'NO'}")

        return has_context_rule and has_no_hallucinate and has_roman_urdu and has_urdu_fallback
    except Exception as e:
        print(f"[-] Error checking system prompt: {e}")
        return False

def main():
    print("Chatbot Data Fetching Diagnostic Tool")
    print("=" * 60)

    # Run all checks
    env_status = check_environment()
    config_ok = check_config_file()
    qdrant_ok = check_retrieval_tool()
    prompt_ok = check_system_prompt()

    print("\n" + "=" * 60)
    print("DIAGNOSTIC SUMMARY:")
    print(f"Environment variables: {'[+] OK' if all(env_status.values()) else '[-] MISSING'}")
    print(f"Configuration: {'[+] OK' if config_ok else '[-] FAILED'}")
    print(f"Qdrant connection: {'[+] OK' if qdrant_ok else '[-] FAILED'}")
    print(f"System prompt: {'[+] OK' if prompt_ok else '[-] FAILED'}")

    print("\n" + "=" * 60)
    print("RECOMMENDATIONS:")

    issues = []
    if not all(env_status.values()):
        issues.append("- Set all required environment variables (OPENROUTER_API_KEY, QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION_NAME)")
    if not config_ok:
        issues.append("- Fix configuration issues")
    if not qdrant_ok:
        issues.append("- Check Qdrant database connection and ensure collection exists with data")
    if not prompt_ok:
        issues.append("- Verify system prompt is correctly formatted")

    if issues:
        for issue in issues:
            print(issue)
    else:
        print("[+] All systems appear to be configured correctly!")
        print("- Ensure your Qdrant database has documents indexed")
        print("- Verify the collection contains data points")
        print("- Test with a simple query to the API")

    print("\nTIP: Run 'python -c \"from qdrant_client import QdrantClient; c=QdrantClient(url=your_url, api_key=your_key); print(c.get_collections())\"' to test Qdrant connection")

if __name__ == "__main__":
    main()