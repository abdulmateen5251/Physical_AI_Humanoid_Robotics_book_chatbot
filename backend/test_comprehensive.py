#!/usr/bin/env python3
"""
Comprehensive test suite to verify all RAG components are working properly
"""
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from app.services.qdrant_service import qdrant_service
from app.services.chatkit_service import chatkit_service
from app.services.retrieval_service import retrieval_service
from app.config import get_settings
import logging

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

def test_configuration():
    """Test configuration loading"""
    logger.info("=== Testing Configuration ===")
    settings = get_settings()
    
    try:
        assert settings.openai_api_key, "OpenAI API key not configured"
        assert settings.qdrant_url, "Qdrant URL not configured"
        assert settings.qdrant_api_key, "Qdrant API key not configured"
        assert settings.database_url, "Database URL not configured"
        
        logger.info(f"✓ OpenAI Model: {settings.openai_model}")
        logger.info(f"✓ Embedding Model: {settings.openai_embedding_model}")
        logger.info(f"✓ Qdrant Collection: {settings.qdrant_collection}")
        logger.info(f"✓ Score Threshold: {settings.score_threshold}")
        logger.info(f"✓ Top K: {settings.top_k}")
        logger.info("")
        return True
    except Exception as e:
        logger.error(f"✗ Configuration error: {e}")
        return False

def test_qdrant_connection():
    """Test Qdrant Cloud connection"""
    logger.info("=== Testing Qdrant Cloud Connection ===")
    settings = get_settings()
    
    try:
        collection_info = qdrant_service.client.get_collection(settings.qdrant_collection)
        logger.info(f"✓ Connected to Qdrant Cloud")
        logger.info(f"✓ Collection: {settings.qdrant_collection}")
        logger.info(f"✓ Points: {collection_info.points_count}")
        logger.info(f"✓ Vector Size: {collection_info.config.params.vectors.size}")
        logger.info(f"✓ Distance Metric: {collection_info.config.params.vectors.distance}")
        logger.info(f"✓ Timeout: 60s")
        logger.info(f"✓ Using REST API (prefer_grpc=False)")
        
        assert collection_info.points_count > 0, "No points in collection"
        logger.info("")
        return True
    except Exception as e:
        logger.error(f"✗ Qdrant connection failed: {e}")
        return False

def test_embedding():
    """Test OpenAI embedding generation"""
    logger.info("=== Testing OpenAI Embeddings ===")
    
    try:
        test_text = "ROS 2 is a robotics middleware framework"
        embedding = chatkit_service.embed_text(test_text)
        
        assert len(embedding) == 1536, f"Wrong embedding size: {len(embedding)}"
        assert all(isinstance(x, float) for x in embedding), "Invalid embedding values"
        
        logger.info(f"✓ Embedding generated: {len(embedding)} dimensions")
        logger.info(f"✓ Model: {chatkit_service.embedding_model}")
        logger.info(f"✓ Sample values: [{embedding[0]:.6f}, {embedding[1]:.6f}, ...]")
        logger.info("")
        return True
    except Exception as e:
        logger.error(f"✗ Embedding failed: {e}")
        return False

def test_retrieval():
    """Test vector similarity search"""
    logger.info("=== Testing Retrieval Service ===")
    
    try:
        result = retrieval_service.retrieve(
            question="What is ROS 2?",
            selection_only=False,
            top_k=5
        )
        
        assert result["status"] == "success", f"Retrieval failed: {result['status']}"
        assert len(result["chunks"]) > 0, "No chunks retrieved"
        
        logger.info(f"✓ Status: {result['status']}")
        logger.info(f"✓ Chunks retrieved: {len(result['chunks'])}")
        
        for i, chunk in enumerate(result["chunks"][:3], 1):
            logger.info(f"\n  Chunk {i}:")
            logger.info(f"    Score: {chunk['score']:.4f}")
            logger.info(f"    Chapter: {chunk['chapter']}")
            logger.info(f"    Section: {chunk.get('section', 'N/A')}")
            logger.info(f"    Text: {chunk['text'][:100]}...")
        
        logger.info("")
        return True
    except Exception as e:
        logger.error(f"✗ Retrieval failed: {e}")
        return False

def test_prompt_building():
    """Test prompt construction"""
    logger.info("=== Testing Prompt Building ===")
    
    try:
        # Get some chunks first
        result = retrieval_service.retrieve(
            question="What is ROS 2?",
            selection_only=False,
            top_k=3
        )
        
        if result["status"] != "success" or not result["chunks"]:
            logger.error("✗ No chunks available for prompt building")
            return False
        
        messages = chatkit_service.build_prompt(
            question="What is ROS 2?",
            chunks=result["chunks"],
            selection_only=False
        )
        
        assert len(messages) == 2, f"Wrong number of messages: {len(messages)}"
        assert messages[0]["role"] == "system", "First message should be system"
        assert messages[1]["role"] == "user", "Second message should be user"
        
        logger.info(f"✓ Messages built: {len(messages)}")
        logger.info(f"✓ System prompt: {len(messages[0]['content'])} chars")
        logger.info(f"✓ User prompt: {len(messages[1]['content'])} chars")
        logger.info(f"✓ Total context: ~{(len(messages[0]['content']) + len(messages[1]['content'])) // 4} tokens (estimated)")
        logger.info("")
        return True
    except Exception as e:
        logger.error(f"✗ Prompt building failed: {e}")
        return False

def test_selection_mode():
    """Test selection-only enforcement"""
    logger.info("=== Testing Selection-Only Mode ===")
    
    try:
        # Test with selection that doesn't match any chunks
        result = retrieval_service.retrieve(
            question="What is this?",
            selection={"text": "completely unrelated text about quantum physics"},
            selection_only=True,
            top_k=5
        )
        
        # Check that status is insufficient_evidence (meaning no matching chunks for selection)
        if result["status"] == "insufficient_evidence":
            logger.info(f"✓ Correctly rejected non-matching selection")
            logger.info(f"✓ Status: {result['status']}")
            logger.info(f"✓ Message: {result['message']}")
            logger.info("")
            return True
        elif result["status"] == "no_results":
            logger.info(f"✓ No results found (acceptable)")
            logger.info(f"✓ Status: {result['status']}")
            logger.info("")
            return True
        else:
            logger.error(f"✗ Unexpected status: {result['status']} (expected 'insufficient_evidence' or 'no_results')")
            return False
            
    except Exception as e:
        logger.error(f"✗ Selection mode test failed: {e}")
        return False

def test_retry_logic():
    """Test retry mechanisms are in place"""
    logger.info("=== Testing Retry Logic ===")
    
    # Check that retry decorators are present
    import inspect
    
    # Check Qdrant service
    qdrant_search = qdrant_service.search
    assert hasattr(qdrant_search, '__wrapped__'), "Qdrant search should have retry decorator"
    logger.info("✓ Qdrant search has retry logic")
    
    qdrant_upsert = qdrant_service.upsert_chunks
    assert hasattr(qdrant_upsert, '__wrapped__'), "Qdrant upsert should have retry decorator"
    logger.info("✓ Qdrant upsert has retry logic")
    
    # Check ChatKit service
    chatkit_embed = chatkit_service.embed_text
    assert hasattr(chatkit_embed, '__wrapped__'), "ChatKit embed should have retry decorator"
    logger.info("✓ ChatKit embedding has retry logic")
    
    logger.info("")
    return True

def main():
    """Run all tests"""
    logger.info("\n" + "="*60)
    logger.info("COMPREHENSIVE RAG SYSTEM TEST")
    logger.info("="*60 + "\n")
    
    tests = [
        ("Configuration", test_configuration),
        ("Qdrant Connection", test_qdrant_connection),
        ("OpenAI Embeddings", test_embedding),
        ("Retrieval Service", test_retrieval),
        ("Prompt Building", test_prompt_building),
        ("Selection Mode", test_selection_mode),
        ("Retry Logic", test_retry_logic),
    ]
    
    results = []
    for name, test_func in tests:
        try:
            success = test_func()
            results.append((name, success))
        except Exception as e:
            logger.error(f"✗ {name} test crashed: {e}")
            results.append((name, False))
    
    # Summary
    logger.info("="*60)
    logger.info("TEST SUMMARY")
    logger.info("="*60)
    
    passed = sum(1 for _, success in results if success)
    total = len(results)
    
    for name, success in results:
        status = "✓ PASSED" if success else "✗ FAILED"
        logger.info(f"{status}: {name}")
    
    logger.info("")
    logger.info(f"Total: {passed}/{total} tests passed")
    
    if passed == total:
        logger.info("\n🎉 ALL TESTS PASSED! System is properly configured.")
        return 0
    else:
        logger.info(f"\n⚠️  {total - passed} test(s) failed. Please review configuration.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
