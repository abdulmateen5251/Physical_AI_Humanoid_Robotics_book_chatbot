#!/usr/bin/env python
"""Deep check if vectors are stored correctly"""
import os
from dotenv import load_dotenv

load_dotenv()

from qdrant_client import QdrantClient

client = QdrantClient(
    url=os.getenv('QDRANT_URL'),
    api_key=os.getenv('QDRANT_API_KEY')
)

# Get collection info
info = client.get_collection('book_chunks')
print(f"Collection: {info.config.params}")
print(f"Total points: {info.points_count}\n")

# Get a point with vectors
try:
    # Use retrieve to get full point including vector
    point = client.retrieve(
        collection_name='book_chunks',
        ids=[1],
        with_vectors=True
    )
    
    if point:
        p = point[0]
        print(f"Point ID: {p.id}")
        print(f"Has vector field: {p.vector is not None}")
        if p.vector:
            print(f"Vector type: {type(p.vector)}")
            print(f"Vector dims: {len(p.vector) if hasattr(p.vector, '__len__') else 'N/A'}")
            print(f"Vector sample (first 5): {list(p.vector)[:5] if hasattr(p.vector, '__iter__') else p.vector}")
        print(f"Payload: {p.payload}")
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
