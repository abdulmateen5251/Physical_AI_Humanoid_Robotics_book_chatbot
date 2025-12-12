"""
Ingestion script to chunk book content, embed, and upsert to Qdrant
"""
import os
import sys
import re
from pathlib import Path
from typing import List, Dict
import uuid

# Add parent directory to path
sys.path.append(str(Path(__file__).parent.parent))

from app.services.qdrant_service import qdrant_service
from app.services.chatkit_service import chatkit_service
from app.config import get_settings

settings = get_settings()

def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50) -> List[str]:
    """Split text into chunks with overlap"""
    chunks = []
    start = 0
    text_length = len(text)
    
    while start < text_length:
        end = start + chunk_size
        chunk = text[start:end]
        chunks.append(chunk)
        start = end - overlap
    
    return chunks

def extract_metadata_from_path(file_path: str) -> Dict:
    """Extract chapter/section from file path"""
    # Example: frontend/docs/module-01-ros2/01-introduction.md
    parts = file_path.split(os.sep)
    
    metadata = {
        "chapter": None,
        "section": None,
        "page": None,
        "uri": file_path
    }
    
    # Extract module/chapter
    for part in parts:
        if part.startswith("module-"):
            metadata["chapter"] = part
        if re.match(r'^\d{2}-.*\.md$', part):
            metadata["section"] = part.replace('.md', '')
    
    return metadata

def process_markdown_file(file_path: str) -> List[Dict]:
    """Process a single markdown file"""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Extract metadata
    metadata = extract_metadata_from_path(file_path)
    
    # Split into chunks
    text_chunks = chunk_text(content, settings.chunk_size, settings.chunk_overlap)
    
    # Prepare chunks with metadata
    processed_chunks = []
    for i, chunk_text_item in enumerate(text_chunks):
        if chunk_text_item.strip():  # Skip empty chunks
            processed_chunks.append({
                "chunk_id": str(uuid.uuid4()),
                "text": chunk_text_item,
                "chapter": metadata["chapter"],
                "section": metadata["section"],
                "page": str(i + 1),
                "uri": metadata["uri"],
                "char_start": i * (settings.chunk_size - settings.chunk_overlap),
                "char_end": i * (settings.chunk_size - settings.chunk_overlap) + len(chunk_text_item)
            })
    
    return processed_chunks

def ingest_book(docs_path: str = "../frontend/docs"):
    """Main ingestion function"""
    print("Starting book ingestion...")
    
    # Create collection
    print("Creating Qdrant collection...")
    qdrant_service.create_collection()
    
    # Find all markdown files
    docs_dir = Path(docs_path)
    if not docs_dir.exists():
        print(f"ERROR: Docs path not found: {docs_dir.absolute()}")
        return
    
    md_files = list(docs_dir.rglob("*.md"))
    print(f"Found {len(md_files)} markdown files")
    
    if len(md_files) == 0:
        print("WARNING: No markdown files found. Skipping ingestion.")
        return
    
    all_chunks = []
    
    # Process each file
    for md_file in md_files:
        print(f"Processing: {md_file}")
        chunks = process_markdown_file(str(md_file))
        all_chunks.extend(chunks)
    
    print(f"Total chunks: {len(all_chunks)}")
    
    # Embed and upsert in batches
    batch_size = 10
    for i in range(0, len(all_chunks), batch_size):
        batch = all_chunks[i:i + batch_size]
        print(f"Processing batch {i // batch_size + 1}/{(len(all_chunks) + batch_size - 1) // batch_size}")
        
        # Embed each chunk
        for chunk in batch:
            chunk["embedding"] = chatkit_service.embed_text(chunk["text"])
        
        # Upsert to Qdrant
        qdrant_service.upsert_chunks(batch)
    
    print("Ingestion complete!")
    print(f"Total chunks ingested: {len(all_chunks)}")

if __name__ == "__main__":
    # Get docs path from command line or use default
    # Adjust path to go up one level from backend to find frontend/docs
    import os
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    default_docs = os.path.join(project_root, "frontend", "docs")
    docs_path = sys.argv[1] if len(sys.argv) > 1 else default_docs
    ingest_book(docs_path)
