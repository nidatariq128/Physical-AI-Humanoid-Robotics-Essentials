#!/usr/bin/env python3
"""
Test script to check API connection and data fetching
"""
import os
import sys
import time
import requests
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def test_api_connection():
    """Test if the API server is running and responding"""
    print("[*] Testing API Connection")
    print("=" * 50)

    # Check if environment variables are loaded
    openrouter_key = os.getenv('OPENROUTER_API_KEY', '')
    qdrant_url = os.getenv('QDRANT_URL', '')
    qdrant_api_key = os.getenv('QDRANT_API_KEY', '')
    qdrant_collection = os.getenv('QDRANT_COLLECTION_NAME', '')

    print(f"OpenRouter API Key: {'[+] SET' if openrouter_key else '[-] NOT SET'}")
    print(f"Qdrant URL: {'[+] SET' if qdrant_url else '[-] NOT SET'}")
    print(f"Qdrant API Key: {'[+] SET' if qdrant_api_key else '[-] NOT SET'}")
    print(f"Qdrant Collection: {qdrant_collection}")

    # Test if the API server is running
    try:
        print("\nTesting server connection...")
        response = requests.get("http://localhost:8000/health", timeout=10)
        if response.status_code == 200:
            print("[+] API Server is running")
            health_data = response.json()
            print(f"Health status: {health_data}")
            return True
        else:
            print(f"[-] API Server returned status code: {response.status_code}")
            print(f"Response: {response.text}")
            return False
    except requests.exceptions.ConnectionError:
        print("[-] API Server is not running or not accessible at http://localhost:8000")
        print("[*] Please start the server with: uvicorn src.rag_agent_api.main:app --host 0.0.0.0 --port 8000")
        return False
    except requests.exceptions.Timeout:
        print("[!] API Server request timed out")
        return False
    except Exception as e:
        print(f"[-] Error connecting to API: {e}")
        return False

def test_config_endpoint():
    """Test the config endpoint to see current configuration"""
    try:
        response = requests.get("http://localhost:8000/config", timeout=10)
        if response.status_code == 200:
            print("[+] Config endpoint accessible")
            config_data = response.json()
            print(f"Current config: {config_data}")
            return True
        else:
            print(f"[-] Config endpoint returned status: {response.status_code}")
            return False
    except Exception as e:
        print(f"[-] Error accessing config endpoint: {e}")
        return False

def test_question_endpoint():
    """Test the question endpoint with a simple query"""
    try:
        test_question = {
            "question": "What is this book about?",
            "top_k": 3
        }
        response = requests.post(
            "http://localhost:8000/question",
            json=test_question,
            timeout=30  # Longer timeout for processing
        )

        print(f"Question endpoint response status: {response.status_code}")
        if response.status_code == 200:
            result = response.json()
            print("[+] Question endpoint responded successfully")
            print(f"Answer: {result.get('answer', 'No answer field')}")
            print(f"Citations count: {result.get('retrieval_results_count', 0)}")
            print(f"Grounded: {result.get('grounded', False)}")
            print(f"Execution time: {result.get('execution_time_ms', 0)}ms")
            return True
        else:
            print(f"[-] Question endpoint returned status: {response.status_code}")
            print(f"Response: {response.text}")
            return False
    except Exception as e:
        print(f"[-] Error testing question endpoint: {e}")
        return False

def main():
    print("API Connection and Data Fetching Test")
    print("=" * 60)

    # First, test basic connection
    api_running = test_api_connection()

    if not api_running:
        print("\n[-] The API server is not running!")
        print("[*] Please start the server first:")
        print("   cd src")
        print("   uvicorn rag_agent_api.main:app --host 0.0.0.0 --port 8000")
        return

    # Test config endpoint
    print("\n" + "=" * 60)
    print("[*] Testing Configuration Endpoint")
    config_ok = test_config_endpoint()

    # Test question endpoint
    print("\n" + "=" * 60)
    print("[*] Testing Question Endpoint")
    question_ok = test_question_endpoint()

    print("\n" + "=" * 60)
    print("TEST SUMMARY:")
    print(f"API Server Running: {'[+] YES' if api_running else '[-] NO'}")
    print(f"Config Endpoint: {'[+] ACCESSIBLE' if config_ok else '[-] FAILED'}")
    print(f"Question Endpoint: {'[+] WORKING' if question_ok else '[-] FAILED'}")

    if api_running and config_ok and question_ok:
        print("\n[+] All tests passed! Your chatbot should be working.")
    else:
        print("\n[!] Issues detected. Check the output above for details.")

if __name__ == "__main__":
    main()