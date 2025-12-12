#!/usr/bin/env python
"""Debug the search process"""
import os
from dotenv import load_dotenv

load_dotenv()

from openai import OpenAI
from qdrant_client import QdrantClient

# Create clients
openai_client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
qdrant_client = QdrantClient(
    url=os.getenv('QDRANT_URL'),
    api_key=os.getenv('QDRANT_API_KEY')
)

# Embed a question
question = "What is ROS2?"
print(f"Question: {question}")

response = openai_client.embeddings.create(
    model="text-embedding-3-small",
    input=question
)
query_vector = response.data[0].embedding
print(f"✓ Embedded question: {len(query_vector)} dims")

# Try to search
print("\nTrying Qdrant.search()...")
try:
    results = qdrant_client.search(
        collection_name='book_chunks',
        query_vector=query_vector,
        limit=5,
        score_threshold=0.1
    )
    print(f"✓ Search returned {len(results)} results")
    for i, result in enumerate(results, 1):
        print(f"  {i}. Score: {result.score:.4f}, Text: {result.payload.get('text', '')[:60]}...")
except Exception as e:
    print(f"✗ Search error: {e}")

# Also try a simpler approach
print("\nDirect test - scrolling first 5 points...")
try:
    points, _ = qdrant_client.scroll('book_chunks', limit=5)
    print(f"✓ Got {len(points)} points")
    for p in points:
        print(f"  - ID: {p.id}, Has Vector: {p.vector is not None}, Text: {p.payload.get('text', '')[:40]}...")
except Exception as e:
    print(f"✗ Scroll error: {e}")
