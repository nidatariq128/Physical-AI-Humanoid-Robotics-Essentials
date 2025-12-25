import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize Qdrant client
client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
    https=True
)

collection_name = os.getenv("QDRANT_COLLECTION_NAME", "my-book")

print(f"Attempting to delete collection: {collection_name}")

try:
    # Delete the collection
    client.delete_collection(collection_name)
    print(f"Successfully deleted collection: {collection_name}")
except Exception as e:
    print(f"Error deleting collection: {str(e)}")

# Verify it's deleted by trying to get collection info
try:
    collection_info = client.get_collection(collection_name)
    print(f"Collection still exists: {collection_info}")
except Exception as e:
    print(f"Collection confirmed deleted (as expected): {str(e)}")