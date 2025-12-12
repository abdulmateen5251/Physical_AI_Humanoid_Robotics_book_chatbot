#!/usr/bin/env python
"""Test vector search is working properly"""
import os
from dotenv import load_dotenv

load_dotenv()

# Test embedding and search
from openai import OpenAI
from qdrant_client import QdrantClient

client_openai = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
client_qdrant = QdrantClient(
    url=os.getenv('QDRANT_URL'),
    api_key=os.getenv('QDRANT_API_KEY')
)

# Test various questions
questions = [
    "What is ROS2?",
    "How does Gazebo simulation work?",
    "What is URDF?",
    "Explain Isaac Sim",
    "What is navigation planning?"
]

print("Testing Vector Search with top_k=5 and score_threshold=0.1\n")
print("=" * 80)

for question in questions:
    # Embed question
    response = client_openai.embeddings.create(
        model="text-embedding-3-small",
        input=question
    )
    query_vector = response.data[0].embedding
    
    # Search
    results = client_qdrant.search(
        collection_name="book_chunks",
        query_vector=query_vector,
        limit=5,
        score_threshold=0.1
    )
    
    print(f"\nQuestion: {question}")
    print(f"Results found: {len(results)}")
    
    for i, result in enumerate(results, 1):
        print(f"  {i}. Score: {result.score:.4f}")
        print(f"     Text: {result.payload.get('text', '')[:80]}...")

print("\n" + "=" * 80)
print("✓ Vector search test complete!")
