import pytest
from fastapi.testclient import TestClient
from app.main import app

client = TestClient(app)

def test_chat_without_selection():
    """Test basic chat without selection"""
    response = client.post("/chat", json={
        "question": "What is ROS 2?",
        "selection_only": False
    })
    
    assert response.status_code == 200
    data = response.json()
    assert "answer" in data
    assert "citations" in data
    assert "status" in data

def test_chat_selection_only_without_selection_fails():
    """Test that selection_only=True without selection returns error"""
    response = client.post("/chat", json={
        "question": "What is this?",
        "selection_only": True
        # No selection provided
    })
    
    assert response.status_code == 400

def test_chat_with_selection():
    """Test chat with selection provided"""
    response = client.post("/chat", json={
        "question": "What is this about?",
        "selection_only": True,
        "selection": {
            "text": "ROS 2 is a robotics middleware",
            "chapter": "module-01",
            "section": "introduction"
        }
    })
    
    assert response.status_code == 200
    data = response.json()
    assert "answer" in data
    # May return insufficient evidence if no matching chunks

def test_history_endpoint():
    """Test history retrieval"""
    # First, create a chat
    session_id = "test-session-123"
    client.post("/chat", json={
        "question": "Test question",
        "session_id": session_id,
        "selection_only": False
    })
    
    # Get history
    response = client.get(f"/history?session_id={session_id}")
    assert response.status_code == 200
    data = response.json()
    assert "messages" in data
    assert "citations" in data
    assert "selections" in data
