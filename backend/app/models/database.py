from sqlalchemy import create_engine, Column, String, Integer, Float, DateTime, Boolean, Text, JSON
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from datetime import datetime
from app.config import get_settings

settings = get_settings()

# SQLite for development, Postgres for production
db_url = settings.database_url
if db_url.startswith("sqlite"):
    engine = create_engine(db_url, connect_args={"check_same_thread": False})
else:
    engine = create_engine(db_url)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

class Session(Base):
    __tablename__ = "sessions"
    
    id = Column(String, primary_key=True)
    created_at = Column(DateTime, default=datetime.utcnow)

class Message(Base):
    __tablename__ = "messages"
    
    id = Column(String, primary_key=True)
    session_id = Column(String, nullable=False, index=True)
    role = Column(String, nullable=False)  # user/assistant
    content = Column(Text, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)

class Selection(Base):
    __tablename__ = "selections"
    
    id = Column(String, primary_key=True)
    session_id = Column(String, nullable=False, index=True)
    text = Column(Text, nullable=False)
    chapter = Column(String)
    section = Column(String)
    page = Column(String)
    start_offset = Column(Integer)
    end_offset = Column(Integer)
    created_at = Column(DateTime, default=datetime.utcnow)

class Citation(Base):
    __tablename__ = "citations"
    
    id = Column(String, primary_key=True)
    message_id = Column(String, nullable=False, index=True)
    chunk_id = Column(String, nullable=False)
    chapter = Column(String)
    section = Column(String)
    page = Column(String)
    uri = Column(String)
    char_start = Column(Integer)
    char_end = Column(Integer)
    score = Column(Float)

class RetrievalLog(Base):
    __tablename__ = "retrieval_logs"
    
    id = Column(String, primary_key=True)
    session_id = Column(String, nullable=False, index=True)
    request_id = Column(String)
    top_chunks = Column(JSON)
    selection_only = Column(Boolean, default=False)
    created_at = Column(DateTime, default=datetime.utcnow)

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

def init_db():
    Base.metadata.create_all(bind=engine)
