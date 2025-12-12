from app.services.qdrant_service import qdrant_service
from app.services.chatkit_service import chatkit_service
from app.config import get_settings
from typing import List, Dict, Optional

settings = get_settings()

class RetrievalService:
    def __init__(self):
        self.qdrant = qdrant_service
        self.chatkit = chatkit_service
    
    def retrieve(
        self,
        question: str,
        top_k: Optional[int] = None,
        score_threshold: Optional[float] = None,
        selection: Optional[Dict] = None,
        selection_only: bool = False
    ) -> Dict:
        """Retrieve relevant chunks with selection-only enforcement"""
        top_k = top_k or settings.top_k
        score_threshold = score_threshold or settings.score_threshold
        
        # Embed question
        query_vector = self.chatkit.embed_text(question)
        
        # Search Qdrant (no filters on first search - get all relevant results)
        chunks = self.qdrant.search(
            query_vector=query_vector,
            top_k=top_k,
            score_threshold=score_threshold
        )
        
        # Check if we got any chunks
        if not chunks:
            return {
                "chunks": [],
                "status": "no_results",
                "message": "No relevant content found in the book."
            }
        
        # Only apply selection filter if selection_only is explicitly TRUE and selection has valid text
        if selection_only and selection and selection.get("text"):
            selection_text = selection.get("text", "").lower().strip()
            
            # Filter chunks that overlap with selection text
            filtered_chunks = [
                c for c in chunks 
                if selection_text in c.get("text", "").lower() or c.get("text", "").lower() in selection_text
            ]
            
            if not filtered_chunks:
                return {
                    "chunks": [],
                    "status": "insufficient_evidence",
                    "message": "Insufficient evidence from the selection."
                }
            
            return {
                "chunks": filtered_chunks,
                "status": "success",
                "message": f"Retrieved {len(filtered_chunks)} chunks from selection"
            }
        
        # Return regular search results (selection_only=false)
        return {
            "chunks": chunks,
            "status": "success",
            "message": f"Retrieved {len(chunks)} chunks"
        }

retrieval_service = RetrievalService()
