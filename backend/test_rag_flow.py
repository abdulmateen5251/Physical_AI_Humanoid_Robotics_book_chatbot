#!/usr/bin/env python
"""
Test complete RAG flow
"""
import sys
from app.services.qdrant_service import qdrant_service
from app.services.chatkit_service import chatkit_service
from app.services.retrieval_service import retrieval_service
from app.config import get_settings

settings = get_settings()

def test_qdrant():
    """Test if Qdrant has data"""
    try:
        info = qdrant_service.client.get_collection('book_chunks')
        print(f"✓ Qdrant collection found: {info.points_count} points")
        return info.points_count > 0
    except Exception as e:
        print(f"✗ Qdrant error: {e}")
        return False

def test_embedding():
    """Test embedding service"""
    try:
        vec = chatkit_service.embed_text("What is ROS2?")
        print(f"✓ Embedding works: {len(vec)} dimensions")
        return True
    except Exception as e:
        print(f"✗ Embedding error: {e}")
        return False

def test_retrieval():
    """Test retrieval without selection"""
    try:
        vec = chatkit_service.embed_text("What is ROS2?")
        results = qdrant_service.search(vec, top_k=3, score_threshold=0.3)
        print(f"✓ Retrieval works: found {len(results)} chunks")
        if results:
            print(f"  Top result score: {results[0]['score']:.4f}")
            print(f"  Text preview: {results[0]['text'][:100]}...")
        return len(results) > 0
    except Exception as e:
        print(f"✗ Retrieval error: {e}")
        return False

def test_full_rag():
    """Test full RAG pipeline"""
    try:
        result = retrieval_service.retrieve(
            question="What is ROS2?",
            selection_only=False,
            selection=None
        )
        print(f"✓ Full RAG flow: {result['status']}")
        print(f"  Message: {result['message']}")
        print(f"  Chunks: {len(result['chunks'])}")
        return result['status'] == 'success' and len(result['chunks']) > 0
    except Exception as e:
        print(f"✗ Full RAG error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("=" * 60)
    print("RAG CHATBOT TEST FLOW")
    print("=" * 60)
    
    tests = [
        ("Qdrant Connection", test_qdrant),
        ("Embedding Service", test_embedding),
        ("Vector Retrieval", test_retrieval),
        ("Full RAG Pipeline", test_full_rag),
    ]
    
    results = []
    for name, test_func in tests:
        print(f"\n[{name}]")
        results.append(test_func())
    
    print("\n" + "=" * 60)
    passed = sum(results)
    total = len(results)
    print(f"Results: {passed}/{total} tests passed")
    
    if all(results):
        print("✓ All systems operational!")
        sys.exit(0)
    else:
        print("✗ Some tests failed")
        sys.exit(1)
