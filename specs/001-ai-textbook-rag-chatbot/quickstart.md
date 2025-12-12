# Quickstart Guide: Physical AI Textbook + RAG Chatbot

**Feature**: 001-ai-textbook-rag-chatbot  
**Last Updated**: 2025-12-06  
**Estimated Setup Time**: 30-45 minutes

## Overview

This guide will help you set up the Physical AI textbook with embedded RAG chatbot on your local machine. By the end, you'll have:
- ✅ Backend FastAPI server running with RAG endpoints
- ✅ Docusaurus frontend with chat widget
- ✅ Local Qdrant vector database
- ✅ Local PostgreSQL for user data
- ✅ Sample documents indexed
- ✅ Ability to test Q&A in both fullbook and selection modes

---

## Prerequisites

### Required Software
- **Python 3.11+**: [Download](https://www.python.org/downloads/)
- **Node.js 18+**: [Download](https://nodejs.org/)
- **Docker Desktop**: [Download](https://www.docker.com/products/docker-desktop/)
- **Git**: [Download](https://git-scm.com/downloads)

### API Keys (Required)
- **OpenAI API Key**: For embeddings and LLM responses ([Get key](https://platform.openai.com/api-keys))
- **Claude API Key** (optional): For content generation/translation ([Get key](https://console.anthropic.com/))

### API Keys (Optional - for production features)
- **Qdrant Cloud API Key**: For cloud vector storage ([Sign up](https://qdrant.tech/))
- **Neon Database URL**: For cloud Postgres ([Sign up](https://neon.tech/))
- **Better-Auth Credentials**: For OAuth2 integration ([Docs](https://better-auth.com/))

---

## Quick Setup (Docker Compose)

### Step 1: Clone the Repository

```bash
git clone https://github.com/<your-org>/Physical_AI_Humanoid_Robotics.git
cd Physical_AI_Humanoid_Robotics
```

If you already have the repo locally, pull the latest changes and continue to Step 2.

### Step 2: Set Environment Variables

Create a `.env` file in the project root:

```bash
# .env
# Backend configuration
OPENAI_API_KEY=sk-proj-...
CLAUDE_API_KEY=sk-ant-...  # Optional

# Database (local via Docker)
DATABASE_URL=postgresql://postgres:dev_password@postgres:5432/
QDRANT_URL=http://qdrant:6333

# Auth (optional for MVP)
BETTER_AUTH_CLIENT_ID=your_client_id
BETTER_AUTH_CLIENT_SECRET=your_client_secret

# Environment
ENVIRONMENT=development
LOG_LEVEL=INFO
```

**Windows users**: Use PowerShell or Git Bash to create the file:
```powershell
@"
OPENAI_API_KEY=sk-proj-...
CLAUDE_API_KEY=sk-ant-...  # Optional
DATABASE_URL=postgresql://postgres:dev_password@postgres:5432/
QDRANT_URL=http://qdrant:6333
"@ | Out-File -Encoding UTF8 .env
```

### Step 3: Start Services with Docker Compose

```bash
# Start all services (Postgres, Qdrant, Backend)
docker-compose up -d

# Verify services are running
docker-compose ps

# Expected output:
# NAME                   STATUS              PORTS
# postgres               Up                  5432->5432
# qdrant                 Up                  6333->6333, 6334->6334
# backend                Up                  8000->8000
```

**Troubleshooting**:
- If ports are already in use: Edit `docker-compose.yml` to change port mappings
- If containers fail to start: Check logs with `docker-compose logs <service_name>`

### Step 4: Run Database Migrations

```bash
# Apply schema migrations
docker-compose exec backend alembic upgrade head

# Expected output: users, user_profiles, answer_sessions, translations
```

### Step 5: Index Sample Documents

```bash
# Index textbook content to Qdrant
docker-compose exec backend python scripts/ingest_to_qdrant.py \
    --docs /app/frontend/docs \
    --collection physical_ai_humanoid_robotics_course

# Expected output:
# ✓ Parsed 20 markdown files
# ✓ Created 487 chunks
# ✓ Generated embeddings (batch 1/5)
# ✓ Indexed to Qdrant collection: physical_ai_humanoid_robotics_course
# ✓ Indexing complete in 3m 45s
```

**Verify indexing**:
```bash
# Check Qdrant collection stats
curl http://localhost:6333/collections/physical_ai_humanoid_robotics_course

# Expected response includes:
# "points_count": 487
# "status": "green"
```

### Step 6: Start Frontend (Docusaurus)

In a new terminal:

```bash
cd frontend
npm install
npm start

# Docusaurus dev server starts on http://localhost:3000
```

**Windows users**:
```powershell
cd frontend
npm install
npm start
```

---

## Verify Setup

### Test Backend API

**Health Check**:
```bash
curl http://localhost:8090/health

# Expected: {"status": "healthy", "version": "1.0.0"}
```

**Test RAG Retrieval**:
```bash
curl -X POST http://localhost:8090/v1/api/retrieve \
  -H "Content-Type: application/json" \
  -d '{
    "query": "how to create a publisher in rclpy",
    "top_k": 3
  }'

# Expected: JSON with 3 document chunks about rclpy publishers
```

**Test Q&A (Full-book Mode)**:
```bash
curl -X POST http://localhost:8090/v1/api/answer \
  -H "Content-Type: application/json" \
  -d '{
    "question": "How do I create a publisher in rclpy?",
    "scope": "fullbook"
  }'

# Expected: JSON with answer, sources, session_id
```

**Test Q&A (Selection Mode)**:
```bash
curl -X POST http://localhost:8090/v1/api/answer \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is the syntax?",
    "scope": "selected_text",
    "selected_text": "To create a publisher, use self.create_publisher(MsgType, '\''topic'\''', 10)"
  }'

# Expected: Answer constrained to selected text only
```

### Test Frontend

1. **Open Docusaurus**: Navigate to http://localhost:3000
2. **Verify Chat Widget**: Look for chat icon in bottom-right corner
3. **Ask a Question**:
   - Click chat widget
   - Type: "How do I create a publisher in rclpy?"
   - Press Enter
   - Verify answer appears with source citations

4. **Test Selection Mode**:
   - Select text on any chapter page
   - Click "Ask about selection" button (appears near selection)
   - Ask a question about the selected text
   - Verify answer references only the selection

---

## Manual Setup (Without Docker)

If you prefer not to use Docker:

### Backend Setup

```bash
# Create virtual environment
python -m venv .venv

# Activate (Mac/Linux)
source .venv/bin/activate

# Activate (Windows PowerShell)
.\.venv\Scripts\Activate.ps1

# Install dependencies
pip install -r backend/requirements.txt
pip install -r backend/requirements-dev.txt

# Install PostgreSQL locally (Mac with Homebrew)
brew install postgresql@15
brew services start postgresql@15

# Create database


# Install Qdrant locally (via Docker)
docker run -d -p 6333:6333 -p 6334:6334 \
    -v $(pwd)/qdrant_storage:/qdrant/storage \
    qdrant/qdrant



# Run migrations
cd backend
alembic upgrade head

# Start server
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

**Windows PowerShell**:
```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r backend\requirements.txt

# Install PostgreSQL: https://www.postgresql.org/download/windows/
# Install Qdrant via Docker Desktop

# Update .env
$env:DATABASE_URL=https://github.com/abdulmateen5251
$env:QDRANT_URL="http://localhost:6333"

cd backend
alembic upgrade head
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

### Frontend Setup

```bash
cd frontend
npm install
npm start
```

---

## Development Workflow

### Making Changes

**Backend Code**:
1. Edit Python files in `backend/app/`
2. Uvicorn auto-reloads on file changes
3. Run tests: `pytest backend/tests/`
4. Format code: `black backend/` and `ruff check backend/`

**Frontend Code**:
1. Edit React components in `frontend/src/components/`
2. Edit Docusaurus content in `frontend/docs/`
3. Changes hot-reload automatically
4. Run tests: `cd frontend && npm test`

**Database Changes**:
1. Create migration: `alembic revision --autogenerate -m "description"`
2. Apply migration: `alembic upgrade head`
3. Rollback: `alembic downgrade -1`

**Re-index Documents** (after content changes):
```bash
docker-compose exec backend python scripts/ingest_to_qdrant.py \
    --docs /app/frontend/docs \
    --collection physical_ai_humanoid_robotics_course \
    --force-reindex
```

### Running Tests

**Backend Unit Tests**:
```bash
docker-compose exec backend pytest backend/tests/unit/ -v
```

**Backend Integration Tests**:
```bash
docker-compose exec backend pytest backend/tests/integration/ -v
```

**RAG Acceptance Tests**:
```bash
docker-compose exec backend pytest backend/tests/acceptance/test_rag_accuracy.py -v

# Expected: >=90% accuracy on 50 Q/A pairs per module
```

**Frontend Tests**:
```bash
cd frontend
npm test

# E2E tests (requires both frontend and backend running)
npm run test:e2e
```

### Debugging

**Backend Logs**:
```bash
# Follow logs in real-time
docker-compose logs -f backend

# View specific service logs
docker-compose logs postgres
docker-compose logs qdrant
```

**Enable Debug Mode**:
Edit `.env`:
```bash
LOG_LEVEL=DEBUG
```

**Access Database**:
```bash
# Connect to Postgres
docker-compose exec postgres psql -U postgres -d 

# Common queries
SELECT COUNT(*) FROM users;
SELECT * FROM answer_sessions ORDER BY created_at DESC LIMIT 10;
```

**Access Qdrant UI**:
- Open http://localhost:6333/dashboard
- View collections, inspect vectors, test queries

---

## Common Issues & Solutions

### Issue: OpenAI API Rate Limit

**Symptom**: `429 Rate limit exceeded` errors

**Solution**:
1. Reduce batch size in `scripts/ingest_to_qdrant.py`:
   ```python
   BATCH_SIZE = 50  # Reduce from 100
   ```
2. Add delay between batches:
   ```python
   import time
   time.sleep(5)  # 5 second delay
   ```

### Issue: Qdrant Connection Failed

**Symptom**: `ConnectionError: Cannot connect to Qdrant`

**Solution**:
1. Verify Qdrant is running:
   ```bash
   docker-compose ps qdrant
   curl http://localhost:6333/health
   ```
2. Check `QDRANT_URL` in `.env` (should be `http://qdrant:6333` inside Docker)

### Issue: Database Migration Failed

**Symptom**: `alembic.util.exc.CommandError`

**Solution**:
1. Reset database:
   ```bash
   
   docker-compose exec postgres psql -U postgres -c "CREATE DATABASE _dev;"
   ```
2. Re-run migrations:
   ```bash
   docker-compose exec backend alembic upgrade head
   ```

### Issue: Frontend Not Connecting to Backend

**Symptom**: Chat widget shows "Connection failed"

**Solution**:
1. Check backend is running: `curl http://localhost:8090/health`
2. Verify CORS settings in `backend/app/main.py`:
   ```python
   app.add_middleware(
       CORSMiddleware,
       allow_origins=["http://localhost:3000"],
       allow_credentials=True,
       allow_methods=["*"],
       allow_headers=["*"],
   )
   ```
3. Check browser console for CORS errors

### Issue: Chat Widget Not Appearing

**Symptom**: Docusaurus loads but no chat widget

**Solution**:
1. Verify plugin is registered in `docusaurus.config.js`:
   ```js
   plugins: [
     './src/theme/ChatWidgetPlugin'
   ]
   ```
2. Check browser console for React errors
3. Clear browser cache: Ctrl+Shift+R (Cmd+Shift+R on Mac)

---

## Next Steps

### Adding Content

1. **Create a New Chapter**:
   ```bash
   # Create markdown file
   touch frontend/docs/module-01-ros2/06-new-chapter.md
   
   # Add frontmatter
   echo "---
   title: New Chapter
   sidebar_position: 6
   ---
   
   # New Chapter Content
   " > frontend/docs/module-01-ros2/06-new-chapter.md
   ```

2. **Re-index**:
   ```bash
   docker-compose exec backend python scripts/ingest_to_qdrant.py \
       --docs /app/frontend/docs \
       --force-reindex
   ```

3. **Verify**: Ask a question about the new chapter content

### Setting Up Authentication

1. **Sign up for Better-Auth**: https://better-auth.com/
2. **Add credentials to `.env`**:
   ```bash
   BETTER_AUTH_CLIENT_ID=your_client_id
   BETTER_AUTH_CLIENT_SECRET=your_client_secret
   ```
3. **Restart backend**: `docker-compose restart backend`
4. **Test signup**: Visit http://localhost:3000/signup

### Deploying to Production

See [deployment guide](./deployment.md) for:
- Deploying backend to Google Cloud Run
- Deploying frontend to GitHub Pages
- Setting up Qdrant Cloud
- Configuring Neon Serverless Postgres

---

## Useful Commands Reference

### Docker Compose
```bash
# Start all services
docker-compose up -d

# Stop all services
docker-compose down

# Restart a service
docker-compose restart backend

# View logs
docker-compose logs -f backend

# Execute command in container
docker-compose exec backend <command>

# Rebuild after code changes
docker-compose up -d --build
```

### Backend
```bash
# Run backend tests
pytest backend/tests/ -v

# Format code
black backend/
ruff check backend/ --fix

# Type checking
mypy backend/app/

# Generate OpenAPI docs
# Visit http://localhost:8090/docs
```

### Frontend
```bash
# Start dev server
npm start

# Build for production
npm run build

# Serve production build
npm run serve

# Run tests
npm test

# Lint
npm run lint
```

### Database
```bash
# Create migration
alembic revision --autogenerate -m "description"

# Apply migrations
alembic upgrade head

# Rollback one migration
alembic downgrade -1

# View migration history
alembic history
```

---

## Getting Help

- **Documentation**: See `specs/001-ai-textbook-rag-chatbot/spec.md`
- **API Reference**: http://localhost:8090/docs (OpenAPI)
- **Issues**: Report bugs on GitHub Issues
- **Discussions**: Join Discord channel (link TBD)

---

## Summary Checklist

- [ ] Docker Desktop installed and running
- [ ] Python 3.11+ and Node.js 18+ installed
- [ ] `.env` file created with OpenAI API key
- [ ] Services started with `docker-compose up -d`
- [ ] Database migrations applied
- [ ] Sample documents indexed (487 chunks)
- [ ] Frontend running on http://localhost:3000
- [ ] Backend API responding on http://localhost:8090
- [ ] Chat widget visible and functional
- [ ] Selection mode working (select text → ask question)

**Setup Complete!** 🎉 You're ready to develop the Physical AI textbook with RAG chatbot.
