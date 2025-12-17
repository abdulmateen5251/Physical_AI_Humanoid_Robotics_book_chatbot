import os
import sys
from pydantic_settings import BaseSettings
from functools import lru_cache

class Settings(BaseSettings):
    # App
    app_name: str = "Book RAG Chatbot"
    environment: str = "development"
    
    # OpenAI
    openai_api_key: str = ""
    openai_model: str = "gpt-4o-mini"
    openai_embedding_model: str = "text-embedding-3-small"
    
    # Qdrant
    qdrant_url: str = ""
    qdrant_api_key: str = ""
    qdrant_collection: str = "book_chunks"
    
    # Neon Postgres
    database_url: str = ""
    
    # Retrieval
    top_k: int = 5
    score_threshold: float = 0.3  # Balanced threshold for good matches
    chunk_size: int = 500
    chunk_overlap: int = 50
    max_tokens: int = 4000  # Max tokens for context
    
    # Rate limiting
    rate_limit_per_minute: int = 10
    
    class Config:
        env_file = ".env"
        case_sensitive = False
        extra = "ignore"  # Ignore extra environment variables

def validate_required_env_vars():
    """Validate that all required environment variables are set"""
    required_vars = {
        "DATABASE_URL": os.getenv("DATABASE_URL"),
        "OPENAI_API_KEY": os.getenv("OPENAI_API_KEY"),
        "QDRANT_URL": os.getenv("QDRANT_URL"),
        "QDRANT_API_KEY": os.getenv("QDRANT_API_KEY"),
    }
    
    missing_vars = [var for var, value in required_vars.items() if not value]
    
    if missing_vars:
        error_msg = (
            f"\n{'='*60}\n"
            f"ERROR: Missing required environment variables:\n"
            f"{', '.join(missing_vars)}\n\n"
            f"Please create a .env file in the project root with:\n"
            f"  cp .env.example .env\n"
            f"Then fill in your credentials.\n"
            f"{'='*60}\n"
        )
        print(error_msg, file=sys.stderr)
        sys.exit(1)

@lru_cache()
def get_settings() -> Settings:
    validate_required_env_vars()
    return Settings()
