from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from typing import Optional, Dict, List
from sqlalchemy.orm import Session
from app.models.database import get_db, Message, Selection, Citation, RetrievalLog
from app.services.retrieval_service import retrieval_service
from app.services.chatkit_service import chatkit_service
import uuid
from datetime import datetime

router = APIRouter(prefix="/chat", tags=["chat"])

class SelectionInput(BaseModel):
    text: str
    chapter: Optional[str] = None
    section: Optional[str] = None
    page: Optional[str] = None
    start_offset: Optional[int] = None
    end_offset: Optional[int] = None

class ChatRequest(BaseModel):
    question: str
    session_id: Optional[str] = None
    selection_only: bool = False
    selection: Optional[SelectionInput] = None

class ChatResponse(BaseModel):
    answer: str
    citations: List[Dict]
    status: str

@router.post("")
async def chat(request: ChatRequest, db: Session = Depends(get_db)):
    """Handle chat request with selection-only support"""
    
    # Validate inputs
    if request.selection_only and not request.selection:
        raise HTTPException(400, "selection_only requires selection to be provided")
    
    session_id = request.session_id or str(uuid.uuid4())
    
    # Store user message
    user_msg_id = str(uuid.uuid4())
    user_msg = Message(
        id=user_msg_id,
        session_id=session_id,
        role="user",
        content=request.question,
        created_at=datetime.utcnow()
    )
    db.add(user_msg)
    
    # Store selection if provided
    if request.selection:
        selection_id = str(uuid.uuid4())
        selection = Selection(
            id=selection_id,
            session_id=session_id,
            text=request.selection.text,
            chapter=request.selection.chapter,
            section=request.selection.section,
            page=request.selection.page,
            start_offset=request.selection.start_offset,
            end_offset=request.selection.end_offset,
            created_at=datetime.utcnow()
        )
        db.add(selection)
    
    # Retrieve relevant chunks
    retrieval_result = retrieval_service.retrieve(
        question=request.question,
        selection=request.selection.dict() if request.selection else None,
        selection_only=request.selection_only
    )
    
    # Handle insufficient evidence
    if retrieval_result["status"] == "insufficient_evidence":
        answer = retrieval_result["message"]
        assistant_msg_id = str(uuid.uuid4())
        assistant_msg = Message(
            id=assistant_msg_id,
            session_id=session_id,
            role="assistant",
            content=answer,
            created_at=datetime.utcnow()
        )
        db.add(assistant_msg)
        db.commit()
        
        return ChatResponse(
            answer=answer,
            citations=[],
            status="insufficient_evidence"
        )
    
    chunks = retrieval_result["chunks"]
    
    # Log retrieval
    retrieval_log_id = str(uuid.uuid4())
    retrieval_log = RetrievalLog(
        id=retrieval_log_id,
        session_id=session_id,
        request_id=user_msg_id,
        top_chunks=[{"chunk_id": c["chunk_id"], "score": c["score"]} for c in chunks],
        selection_only=request.selection_only,
        created_at=datetime.utcnow()
    )
    db.add(retrieval_log)
    
    # Generate answer
    answer_parts = []
    async for part in chatkit_service.stream_answer(
        question=request.question,
        chunks=chunks,
        selection_only=request.selection_only
    ):
        answer_parts.append(part)
    
    answer = "".join(answer_parts)
    
    # Store assistant message
    assistant_msg_id = str(uuid.uuid4())
    assistant_msg = Message(
        id=assistant_msg_id,
        session_id=session_id,
        role="assistant",
        content=answer,
        created_at=datetime.utcnow()
    )
    db.add(assistant_msg)
    
    # Store citations
    citations = []
    for chunk in chunks:
        citation_id = str(uuid.uuid4())
        citation = Citation(
            id=citation_id,
            message_id=assistant_msg_id,
            chunk_id=chunk["chunk_id"],
            chapter=chunk.get("chapter"),
            section=chunk.get("section"),
            page=chunk.get("page"),
            uri=chunk.get("uri"),
            char_start=chunk.get("char_start"),
            char_end=chunk.get("char_end"),
            score=chunk.get("score")
        )
        db.add(citation)
        citations.append({
            "chapter": chunk.get("chapter"),
            "section": chunk.get("section"),
            "page": chunk.get("page"),
            "uri": chunk.get("uri"),
            "score": chunk.get("score")
        })
    
    db.commit()
    
    return ChatResponse(
        answer=answer,
        citations=citations,
        status="success"
    )
