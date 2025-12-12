from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue
from app.config import get_settings
from typing import List, Dict, Optional
import uuid
import logging
from tenacity import retry, stop_after_attempt, wait_exponential

logger = logging.getLogger(__name__)
settings = get_settings()

class QdrantService:
    def __init__(self):
        """Initialize Qdrant client with proper configuration for Cloud"""
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            timeout=60,  # 60 second timeout for Cloud operations
            prefer_grpc=False  # Use REST API for better Cloud compatibility
        )
        self.collection_name = settings.qdrant_collection
        logger.info(f"Qdrant service initialized with collection: {self.collection_name}")
    
    def create_collection(self, vector_size: int = 1536):
        """Create collection if not exists"""
        try:
            self.client.get_collection(self.collection_name)
        except:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE)
            )
    
    @retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=2, max=10))
    def upsert_chunks(self, chunks: List[Dict]):
        """Upsert chunks with embeddings and metadata (with retry logic)"""
        if not chunks:
            logger.warning("No chunks to upsert")
            return
        
        points = []
        for chunk in chunks:
            # Validate chunk has required fields
            if "embedding" not in chunk or "text" not in chunk:
                logger.warning(f"Skipping invalid chunk: {chunk.get('chunk_id', 'unknown')}")
                continue
                
            point = PointStruct(
                id=chunk.get("chunk_id", str(uuid.uuid4())),
                vector=chunk["embedding"],
                payload={
                    "text": chunk["text"][:10000],  # Limit text size for payload
                    "chapter": chunk.get("chapter"),
                    "section": chunk.get("section"),
                    "page": chunk.get("page"),
                    "uri": chunk.get("uri"),
                    "char_start": chunk.get("char_start"),
                    "char_end": chunk.get("char_end"),
                }
            )
            points.append(point)
        
        if not points:
            logger.warning("No valid points to upsert after filtering")
            return
        
        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            logger.info(f"Successfully upserted {len(points)} points")
        except Exception as e:
            logger.error(f"Error upserting chunks: {e}")
            raise
    
    @retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=1, max=5))
    def search(
        self, 
        query_vector: List[float], 
        top_k: int = 5,
        score_threshold: float = 0.3,
        chapter: Optional[str] = None,
        section: Optional[str] = None,
        page: Optional[str] = None
    ):
        """Search with vector similarity - Qdrant Cloud compatible with retry logic"""
        from qdrant_client.models import Filter, FieldCondition, MatchValue
        
        if not query_vector or len(query_vector) == 0:
            logger.error("Empty query vector provided")
            return []
        
        filter_conditions = []
        
        if chapter:
            filter_conditions.append(
                FieldCondition(key="chapter", match=MatchValue(value=chapter))
            )
        if section:
            filter_conditions.append(
                FieldCondition(key="section", match=MatchValue(value=section))
            )
        if page:
            filter_conditions.append(
                FieldCondition(key="page", match=MatchValue(value=page))
            )
        
        search_filter = Filter(must=filter_conditions) if filter_conditions else None
        
        try:
            # Qdrant Cloud uses .query_points() with query parameter
            response = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=top_k,
                score_threshold=score_threshold,
                query_filter=search_filter
            )
            
            # Extract points from response
            results = response.points if hasattr(response, 'points') else response
            
            logger.info(f"Search returned {len(results)} results with score >= {score_threshold}")
            
            return [
                {
                    "chunk_id": str(result.id),
                    "text": result.payload.get("text", ""),
                    "score": result.score,
                    "chapter": result.payload.get("chapter"),
                    "section": result.payload.get("section"),
                    "page": result.payload.get("page"),
                    "uri": result.payload.get("uri"),
                    "char_start": result.payload.get("char_start"),
                    "char_end": result.payload.get("char_end"),
                }
                for result in results
            ]
        except Exception as e:
            logger.error(f"Error searching Qdrant: {e}")
            raise

qdrant_service = QdrantService()
