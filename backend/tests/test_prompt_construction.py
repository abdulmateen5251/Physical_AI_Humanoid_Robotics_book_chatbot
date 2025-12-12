import pytest
from app.services.chatkit_service import chatkit_service

def test_prompt_includes_selection_only_flag():
    """Test that selection-only mode is included in prompt"""
    
    chunks = [
        {
            "text": "ROS 2 is a robotics middleware.",
            "chapter": "module-01",
            "section": "introduction",
            "page": "1"
        }
    ]
    
    messages = chatkit_service.build_prompt(
        question="What is ROS 2?",
        chunks=chunks,
        selection_only=True
    )
    
    # Check that selection-only mode is mentioned
    user_message = messages[1]["content"]
    assert "SELECTION-ONLY MODE" in user_message or "selection" in user_message.lower()

def test_prompt_includes_citations_requirement():
    """Test that prompt requires citations"""
    
    chunks = [
        {
            "text": "ROS 2 is a robotics middleware.",
            "chapter": "module-01",
            "section": "introduction",
            "page": "1"
        }
    ]
    
    messages = chatkit_service.build_prompt(
        question="What is ROS 2?",
        chunks=chunks
    )
    
    # Check system prompt mentions citations
    system_message = messages[0]["content"]
    assert "citation" in system_message.lower() or "cite" in system_message.lower()

def test_chunks_included_in_context():
    """Test that chunks are properly formatted in context"""
    
    chunks = [
        {
            "text": "First chunk text",
            "chapter": "module-01",
            "section": "intro",
            "page": "1"
        },
        {
            "text": "Second chunk text",
            "chapter": "module-01",
            "section": "intro",
            "page": "2"
        }
    ]
    
    messages = chatkit_service.build_prompt(
        question="Test question",
        chunks=chunks
    )
    
    user_message = messages[1]["content"]
    assert "First chunk text" in user_message
    assert "Second chunk text" in user_message
    assert "Source 1" in user_message or "[Source" in user_message
