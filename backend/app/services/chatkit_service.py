from openai import OpenAI
from app.config import get_settings
from typing import List, Dict, Optional, AsyncIterator
import asyncio
import logging
from tenacity import retry, stop_after_attempt, wait_exponential

logger = logging.getLogger(__name__)
settings = get_settings()
client = OpenAI(
    api_key=settings.openai_api_key,
    timeout=60.0,  # 60 second timeout
    max_retries=3  # Automatic retry for transient errors
)

class ChatKitService:
    def __init__(self):
        self.model = settings.openai_model
        self.embedding_model = settings.openai_embedding_model
        logger.info(f"ChatKit service initialized with model: {self.model}")
    
    @retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=1, max=5))
    def embed_text(self, text: str) -> List[float]:
        """Generate embedding for text with retry logic"""
        if not text or not text.strip():
            logger.warning("Empty text provided for embedding")
            return [0.0] * 1536  # Return zero vector for empty text
        
        # Truncate text if too long (max 8191 tokens for text-embedding-3-small)
        if len(text) > 32000:  # Rough estimate: 4 chars per token
            text = text[:32000]
            logger.warning("Text truncated for embedding")
        
        try:
            response = client.embeddings.create(
                model=self.embedding_model,
                input=text.strip()
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            raise
    
    def build_prompt(
        self,
        question: str,
        chunks: List[Dict],
        selection_only: bool = False,
        system_prompt: Optional[str] = None
    ) -> List[Dict]:
        """Build messages for ChatKit with token management"""
        if not system_prompt:
            system_prompt = """You are an AI assistant for a robotics textbook on Physical AI and Humanoid Robotics.

RULES:
1. Answer ONLY from the provided book content
2. Always include inline citations: (Chapter: X, Section: Y, Page: Z)
3. If selection-only mode: Use ONLY the provided sources. If insufficient, say "Insufficient evidence from the selection."
4. Never use external knowledge or speculate
5. Be concise, technical, and accurate
6. If evidence is weak or unclear, acknowledge limitations

QUALITY:
- Cite sources for every major claim
- Use technical terminology from the book
- Structure answers clearly with bullet points when appropriate"""
        
        # Limit context to avoid token overflow (rough estimate: 4 chars per token)
        max_context_chars = settings.max_tokens * 3  # Leave room for question and response
        context_parts = []
        current_length = 0
        
        for i, c in enumerate(chunks):
            chunk_text = c['text'][:2000]  # Limit individual chunk size
            source_info = f"[Source {i+1}] Chapter: {c.get('chapter', 'N/A')}, Section: {c.get('section', 'N/A')}, Page: {c.get('page', 'N/A')}\n{chunk_text}"
            
            if current_length + len(source_info) > max_context_chars:
                logger.warning(f"Context truncated after {i} chunks to stay within token limit")
                break
            
            context_parts.append(source_info)
            current_length += len(source_info)
        
        context = "\n\n".join(context_parts)
        
        mode_note = "\n\n**SELECTION-ONLY MODE**: Answer using ONLY the provided sources below. If the sources don't contain enough information, respond with 'Insufficient evidence from the selection.'" if selection_only else ""
        
        user_message = f"""Question: {question}{mode_note}

Book Content:
{context}

Instructions: Answer the question based strictly on the content above. Include inline citations for all claims."""
        
        return [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_message}
        ]
    
    async def stream_answer(
        self,
        question: str,
        chunks: List[Dict],
        selection_only: bool = False
    ) -> AsyncIterator[str]:
        """Stream answer using ChatKit with proper error handling"""
        if not chunks:
            logger.warning("No chunks provided for answer generation")
            yield "No relevant content found to answer this question."
            return
        
        messages = self.build_prompt(question, chunks, selection_only)
        
        try:
            stream = client.chat.completions.create(
                model=self.model,
                messages=messages,
                stream=True,
                temperature=0.2,  # Lower temperature for more factual responses
                max_tokens=1000,  # Limit response length
                top_p=0.9,  # Nucleus sampling for better quality
            )
            
            has_content = False
            for chunk in stream:
                if chunk.choices[0].delta.content:
                    has_content = True
                    yield chunk.choices[0].delta.content
            
            if not has_content:
                logger.warning("Stream completed but no content generated")
                yield "Unable to generate a response. Please try again."
                
        except Exception as e:
            logger.error(f"Error streaming response: {e}")
            yield f"\n\nError: Unable to generate response. Please try again. ({str(e)[:100]})"

chatkit_service = ChatKitService()
