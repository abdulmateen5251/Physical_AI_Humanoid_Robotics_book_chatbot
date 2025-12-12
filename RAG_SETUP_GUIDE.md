# 🚀 RAG Chatbot Setup Guide

## Prerequisites

- Python 3.11+
- Node.js 18+
- Git
- OpenAI API Account
- Qdrant Cloud Account
- Neon Postgres Account

## Step 1: Get API Keys & Credentials

### 1.1 OpenAI API Key
```bash
1. Go to: https://platform.openai.com/api-keys
2. Click "Create new secret key"
3. Copy the key (starts with sk-proj-)
4. Save it safely
```

### 1.2 Qdrant Cloud Setup
```bash
1. Go to: https://cloud.qdrant.io
2. Sign up (free tier available)
3. Create new cluster
4. Copy the URL (looks like: https://xxxxx-xxxxx.qdrant.io)
5. Copy the API Key (starts with ey...)
```

### 1.3 Neon Postgres Setup
```bash
1. Go to: https://console.neon.tech
2. Sign up (free tier: 3 projects)
3. Create new project
4. Wait for project creation (~1 min)
5. Click "Connection string"
6. Copy the full URL (looks like: postgresql://user:password@host/database)
```

## Step 2: Configure Environment

```bash
# Clone repository (if not already)
git clone https://github.com/your-username/Physical_AI_Humanoid_Robotics_book.git
cd Physical_AI_Humanoid_Robotics_book

# Create .env from template
cd backend
cp .env.example .env

# Edit .env with your credentials
# Open in editor and fill in:
# - OPENAI_API_KEY
# - QDRANT_URL
# - QDRANT_API_KEY
# - DATABASE_URL
```

## Step 3: Install Dependencies

```bash
# Backend
cd backend
pip install -r requirements.txt

# Frontend
cd ../frontend
npm install
```

## Step 4: Initialize Database

```bash
# Create tables in Neon
cd backend
python -c "from app.models.database import init_db; init_db(); print('✅ Database initialized')"
```

## Step 5: Ingest Book Content

```bash
# This chunks book markdown, generates embeddings, and uploads to Qdrant
python scripts/ingest_book.py

# Expected output:
# Starting book ingestion...
# Creating Qdrant collection...
# Found XX markdown files
# Processing: frontend/docs/module-01/01-introduction.md
# ...
# ✅ Ingestion complete!
# Total chunks ingested: XXXX
```

## Step 6: Run Backend

```bash
# Terminal 1 - Backend
cd backend
uvicorn app.main:app --reload --port 8000

# Expected output:
# INFO:     Uvicorn running on http://127.0.0.1:8000
# INFO:     Application startup complete
```

## Step 7: Run Frontend

```bash
# Terminal 2 - Frontend
cd frontend
npm start

# Expected output:
# [INFO] ⚡️ Docusaurus is serving at http://localhost:3000
```

## Step 8: Test RAG Chatbot

1. Open browser: http://localhost:3000
2. Read book content in any module
3. Highlight some text
4. Scroll to bottom-right for Chat Widget
5. Toggle "Answer from selection only"
6. Ask a question
7. Verify:
   - ✅ Answer appears
   - ✅ Citations show chapter/section/page
   - ✅ Selection-only mode enforces selection

## Troubleshooting

### Error: "No such file or directory: 'frontend/docs'"
- Make sure you're running from project root
- Check that `frontend/docs/` exists

### Error: "Invalid OpenAI API Key"
- Check API key is correct (starts with `sk-proj-`)
- Verify key has usage quota remaining
- Go to: https://platform.openai.com/account/api-keys

### Error: "Failed to connect to Qdrant"
- Check URL format: `https://xxxxx-xxxxx.qdrant.io` (with https)
- Check API key is correct
- Verify Qdrant cluster is active in cloud.qdrant.io

### Error: "Connection refused to Neon"
- Check DATABASE_URL format
- Verify Neon cluster is active
- Check password doesn't have special characters (URL encode if needed)

### Chat widget not appearing
- Check backend is running on port 8000
- Check CORS settings in `app/main.py`
- Open browser console (F12) for errors

## Production Deployment

### Frontend (Docusaurus)
```bash
cd frontend
npm run build
# Deploy frontend/build/ to Vercel/Netlify/GitHub Pages
```

### Backend (FastAPI)
```bash
# Use production settings
uvicorn app.main:app --host 0.0.0.0 --port 8000
# Deploy with Docker or serverless platform
```

## FAQ

**Q: How many chunks can I ingest?**
A: Qdrant Free Tier allows ~1GB of storage. For 20+ book chapters, expect ~5k-10k chunks (0.5-1MB).

**Q: Does ingestion need to run again?**
A: Only when you add new book content. Updates are idempotent.

**Q: Can I use different LLM models?**
A: Yes! Edit `OPENAI_MODEL` in `.env`. Supports any OpenAI model.

**Q: Is selection-only mode mandatory?**
A: No, it's optional. Users can toggle it on/off.

**Q: What if Qdrant/Neon goes down?**
A: Backend returns graceful error. Chat widget shows: "Service temporarily unavailable."

---

**Need Help?**
- Backend docs: `backend/README.md`
- Implementation details: `IMPLEMENTATION_COMPLETE.md`
- Issues: Create GitHub issue with error details
