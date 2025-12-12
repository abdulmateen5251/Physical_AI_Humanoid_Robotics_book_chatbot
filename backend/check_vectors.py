#!/usr/bin/env python
"""Check if vectors are saved in Qdrant"""
import os
from dotenv import load_dotenv

load_dotenv()

try:
    from qdrant_client import QdrantClient
    
    client = QdrantClient(
        url=os.getenv('QDRANT_URL'),
        api_key=os.getenv('QDRANT_API_KEY')
    )
    
    # Get collection info
    info = client.get_collection('book_chunks')
    print(f"✓ Collection exists: 'book_chunks'")
    print(f"✓ Total points: {info.points_count}")
    print(f"✓ Vector size: {info.config.params.vectors.size}")
    
    if info.points_count > 0:
        print("\n✓ VECTORS ARE SAVED IN QDRANT!")
        
        # Check a sample point
        sample = client.scroll('book_chunks', limit=1)
        if sample[0]:
            point = sample[0][0]
            has_vector = point.vector is not None
            print(f"✓ Sample point has vector: {has_vector}")
            if has_vector:
                print(f"  Vector dimensions: {len(point.vector) if hasattr(point.vector, '__len__') else 'N/A'}")
    else:
        print("\n✗ NO VECTORS FOUND - Need to run ingestion!")
        
except Exception as e:
    print(f"✗ Error: {e}")
    import traceback
    traceback.print_exc()
