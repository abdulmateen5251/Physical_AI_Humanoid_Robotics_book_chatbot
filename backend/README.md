# Book RAG Chatbot Backend

Embedded RAG chatbot for book Q&A with strict selection-only support.

## Setup

1. **Install dependencies**:
```bash
pip install -r requirements.txt
```

2. **Configure environment**:
```bash
cp .env.example .env
# Edit .env with your credentials
```

3. **Initialize database**:
```bash
python -m app.main
# Database tables will be created automatically
```

4. **Ingest book content**:
```bash
python scripts/ingest_book.py
```

## Running

```bash
uvicorn app.main:app --reload --port 8000
```

## API Endpoints

### POST /chat
Answer questions with optional selection-only mode.

**Request**:
```json
{
  "question": "What is ROS 2?",
  "session_id": "optional-session-id",
  "selection_only": false,
  "selection": {
    "text": "highlighted text",
    "chapter": "module-01",
    "section": "introduction",
    "page": "1"
  }
}
```

**Response**:
```json
{
  "answer": "ROS 2 is...",
  "citations": [
    {
      "chapter": "module-01",
      "section": "introduction",
      "page": "1",
      "uri": "frontend/docs/module-01/intro.md",
      "score": 0.85
    }
  ],
  "status": "success"
}
```

### GET /history?session_id=xxx
Get chat history for a session.

## Testing

```bash
pytest tests/
```

## Features

- ✅ Book-wide Q&A with Qdrant vector search
- ✅ Selection-only mode with strict enforcement
- ✅ Citation tracking (chapter/section/page/URI)
- ✅ OpenAI ChatKit orchestration (gpt-4o-mini)
- ✅ Session persistence in Neon Postgres
- ✅ "Insufficient evidence" fallback
- ✅ Input validation and error handling
