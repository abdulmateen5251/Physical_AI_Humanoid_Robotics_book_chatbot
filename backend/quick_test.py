import os
import sys
from dotenv import load_dotenv

# Load env first
load_dotenv()

# Test 1: Check API key
print(f"API Key loaded: {bool(os.getenv('OPENAI_API_KEY'))}")
print(f"Qdrant URL: {os.getenv('QDRANT_URL')}")

# Test 2: Try OpenAI embedding
try:
    from openai import OpenAI
    client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
    response = client.embeddings.create(
        model="text-embedding-3-small",
        input="test"
    )
    print(f"✓ OpenAI embedding works: {len(response.data[0].embedding)} dims")
except Exception as e:
    print(f"✗ OpenAI embedding failed: {e}")
    sys.exit(1)

# Test 3: Try Qdrant
try:
    from qdrant_client import QdrantClient
    client = QdrantClient(
        url=os.getenv('QDRANT_URL'),
        api_key=os.getenv('QDRANT_API_KEY')
    )
    info = client.get_collection('book_chunks')
    print(f"✓ Qdrant connection works: {info.points_count} points in collection")
except Exception as e:
    print(f"✗ Qdrant connection failed: {e}")
    sys.exit(1)

print("\n✓ All services working!")
