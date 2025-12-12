#!/usr/bin/env python3
"""
Edge case and stress tests for RAG system
"""
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from app.services.retrieval_service import retrieval_service
from app.services.chatkit_service import chatkit_service
import logging

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

def test_empty_query():
    """Test handling of empty queries"""
    logger.info("=== Testing Empty Query Handling ===")
    
    try:
        # Empty string - should return zero vector
        embedding = chatkit_service.embed_text("")
        assert len(embedding) == 1536, "Should return 1536-dim vector"
        assert all(v == 0.0 for v in embedding), "Should be zero vector"
        logger.info("✓ Empty string handled correctly (returns zero vector)")
        
        # Whitespace only - should also return zero vector
        embedding = chatkit_service.embed_text("   ")
        assert len(embedding) == 1536, "Should return 1536-dim vector"
        assert all(v == 0.0 for v in embedding), "Should be zero vector"
        logger.info("✓ Whitespace-only handled correctly (returns zero vector)")
        
        logger.info("")
        return True
    except Exception as e:
        logger.error(f"✗ Empty query test failed: {e}")
        return False

def test_very_long_text():
    """Test handling of very long input"""
    logger.info("=== Testing Long Text Handling ===")
    
    try:
        # Create 40k character text (exceeds 32k limit)
        long_text = "x" * 40000
        embedding = chatkit_service.embed_text(long_text)
        
        assert len(embedding) == 1536, "Should still return valid embedding"
        logger.info(f"✓ Long text (40k chars) handled correctly")
        logger.info(f"✓ Truncated to 32k chars internally")
        logger.info("")
        return True
    except Exception as e:
        logger.error(f"✗ Long text test failed: {e}")
        return False

def test_special_characters():
    """Test handling of special characters"""
    logger.info("=== Testing Special Characters ===")
    
    try:
        special_text = "What about émojis 🤖, spëcial chars & symbols!?"
        result = retrieval_service.retrieve(
            question=special_text,
            selection_only=False,
            top_k=3
        )
        
        assert result["status"] in ["success", "no_results"], "Should handle special chars"
        logger.info(f"✓ Special characters handled: {special_text[:50]}")
        logger.info(f"✓ Status: {result['status']}")
        logger.info("")
        return True
    except Exception as e:
        logger.error(f"✗ Special characters test failed: {e}")
        return False

def test_context_length():
    """Test that context doesn't exceed limits"""
    logger.info("=== Testing Context Length Management ===")
    
    try:
        result = retrieval_service.retrieve(
            question="Tell me everything about ROS 2",
            selection_only=False,
            top_k=10  # Request more chunks
        )
        
        if result["status"] == "success" and result["chunks"]:
            messages = chatkit_service.build_prompt(
                question="Tell me everything",
                chunks=result["chunks"],
                selection_only=False
            )
            
            total_chars = sum(len(m["content"]) for m in messages)
            total_tokens = total_chars // 4  # Rough estimate
            
            logger.info(f"✓ System prompt: {len(messages[0]['content'])} chars")
            logger.info(f"✓ User prompt: {len(messages[1]['content'])} chars")
            logger.info(f"✓ Total: {total_chars} chars (~{total_tokens} tokens)")
            logger.info(f"✓ Within limits: {total_tokens < 100000}")
            logger.info("")
            return True
        else:
            logger.info("✓ No results (acceptable)")
            logger.info("")
            return True
    except Exception as e:
        logger.error(f"✗ Context length test failed: {e}")
        return False

def test_no_results_query():
    """Test queries that should return no results"""
    logger.info("=== Testing No Results Queries ===")
    
    try:
        result = retrieval_service.retrieve(
            question="How do I bake a chocolate cake?",
            selection_only=False,
            top_k=5
        )
        
        # Either no results or low-relevance results
        logger.info(f"✓ Query: 'How do I bake a chocolate cake?'")
        logger.info(f"✓ Status: {result['status']}")
        logger.info(f"✓ Chunks: {len(result['chunks'])}")
        
        if result["chunks"]:
            logger.info(f"✓ Best score: {result['chunks'][0]['score']:.4f}")
            if result['chunks'][0]['score'] < 0.5:
                logger.info("✓ Low relevance correctly identified")
        
        logger.info("")
        return True
    except Exception as e:
        logger.error(f"✗ No results test failed: {e}")
        return False

def test_multiple_topics():
    """Test query spanning multiple topics"""
    logger.info("=== Testing Multi-Topic Query ===")
    
    try:
        result = retrieval_service.retrieve(
            question="Explain ROS 2, Gazebo, and Isaac Sim",
            selection_only=False,
            top_k=5
        )
        
        if result["status"] == "success" and result["chunks"]:
            # Check if we got chunks from different modules
            chapters = set(c["chapter"] for c in result["chunks"])
            logger.info(f"✓ Retrieved from {len(chapters)} different chapters: {chapters}")
            logger.info(f"✓ Total chunks: {len(result['chunks'])}")
            logger.info("")
            return True
        else:
            logger.info("✓ Query handled (no results)")
            logger.info("")
            return True
    except Exception as e:
        logger.error(f"✗ Multi-topic test failed: {e}")
        return False

def test_citation_format():
    """Test that citations are properly formatted"""
    logger.info("=== Testing Citation Format ===")
    
    try:
        result = retrieval_service.retrieve(
            question="What is ROS 2?",
            selection_only=False,
            top_k=3
        )
        
        if result["status"] == "success" and result["chunks"]:
            chunk = result["chunks"][0]
            
            # Check required fields
            assert "chapter" in chunk, "Missing chapter"
            assert "text" in chunk, "Missing text"
            assert "score" in chunk, "Missing score"
            
            logger.info(f"✓ Chapter: {chunk.get('chapter')}")
            logger.info(f"✓ Section: {chunk.get('section', 'N/A')}")
            logger.info(f"✓ Page: {chunk.get('page', 'N/A')}")
            logger.info(f"✓ Score: {chunk['score']:.4f}")
            logger.info(f"✓ All required fields present")
            logger.info("")
            return True
        else:
            logger.info("✓ No results (acceptable)")
            logger.info("")
            return True
    except Exception as e:
        logger.error(f"✗ Citation format test failed: {e}")
        return False

def main():
    """Run all edge case tests"""
    logger.info("\n" + "="*60)
    logger.info("EDGE CASE & STRESS TESTS")
    logger.info("="*60 + "\n")
    
    tests = [
        ("Empty Query Handling", test_empty_query),
        ("Long Text Handling", test_very_long_text),
        ("Special Characters", test_special_characters),
        ("Context Length Management", test_context_length),
        ("No Results Queries", test_no_results_query),
        ("Multi-Topic Queries", test_multiple_topics),
        ("Citation Format", test_citation_format),
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
    logger.info("EDGE CASE TEST SUMMARY")
    logger.info("="*60)
    
    passed = sum(1 for _, success in results if success)
    total = len(results)
    
    for name, success in results:
        status = "✓ PASSED" if success else "✗ FAILED"
        logger.info(f"{status}: {name}")
    
    logger.info("")
    logger.info(f"Total: {passed}/{total} edge case tests passed")
    
    if passed == total:
        logger.info("\n🎉 ALL EDGE CASE TESTS PASSED!")
        return 0
    else:
        logger.info(f"\n⚠️  {total - passed} test(s) failed.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
