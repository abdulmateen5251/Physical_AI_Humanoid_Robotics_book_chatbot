import pytest
from app.services.retrieval_service import retrieval_service

def test_selection_only_enforcement():
    """Test that selection-only mode filters correctly"""
    
    # Mock selection that doesn't match any chunks
    selection = {
        "text": "This is a selection that won't match",
        "chapter": "module-01",
        "section": "introduction"
    }
    
    result = retrieval_service.retrieve(
        question="What is this about?",
        selection=selection,
        selection_only=True
    )
    
    # Should return insufficient evidence
    assert result["status"] == "insufficient_evidence"
    assert result["chunks"] == []
    assert "insufficient evidence" in result["message"].lower()

def test_normal_retrieval_without_selection():
    """Test that normal retrieval works without selection"""
    
    result = retrieval_service.retrieve(
        question="What is ROS 2?",
        selection_only=False
    )
    
    # Should retrieve chunks (assuming book is ingested)
    assert result["status"] == "success"
    # May have chunks if DB is populated

def test_selection_provided_but_flag_off():
    """Test that providing selection without flag doesn't filter"""
    
    selection = {
        "text": "ROS 2 is a robotics framework",
        "chapter": "module-01"
    }
    
    result = retrieval_service.retrieve(
        question="What is ROS 2?",
        selection=selection,
        selection_only=False  # Flag is off
    )
    
    # Should not enforce selection-only
    assert result["status"] == "success"
