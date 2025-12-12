from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from typing import Optional, List, Dict
from sqlalchemy.orm import Session
from app.models.database import get_db, Message, Citation, Selection
from datetime import datetime

router = APIRouter(prefix="/history", tags=["history"])

class HistoryResponse(BaseModel):
    messages: List[Dict]
    citations: List[Dict]
    selections: List[Dict]

@router.get("")
async def get_history(session_id: str, db: Session = Depends(get_db)):
    """Get chat history for a session"""
    
    # Get messages
    messages = db.query(Message).filter(
        Message.session_id == session_id
    ).order_by(Message.created_at).all()
    
    # Get citations for all assistant messages
    message_ids = [m.id for m in messages if m.role == "assistant"]
    citations = db.query(Citation).filter(
        Citation.message_id.in_(message_ids)
    ).all() if message_ids else []
    
    # Get selections
    selections = db.query(Selection).filter(
        Selection.session_id == session_id
    ).order_by(Selection.created_at).all()
    
    return HistoryResponse(
        messages=[
            {
                "id": m.id,
                "role": m.role,
                "content": m.content,
                "created_at": m.created_at.isoformat()
            }
            for m in messages
        ],
        citations=[
            {
                "message_id": c.message_id,
                "chapter": c.chapter,
                "section": c.section,
                "page": c.page,
                "uri": c.uri,
                "score": c.score
            }
            for c in citations
        ],
        selections=[
            {
                "id": s.id,
                "text": s.text,
                "chapter": s.chapter,
                "section": s.section,
                "page": s.page,
                "created_at": s.created_at.isoformat()
            }
            for s in selections
        ]
    )
