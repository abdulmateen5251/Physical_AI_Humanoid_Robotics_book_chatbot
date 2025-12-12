# Research & Technology Decisions: Physical AI Textbook + RAG Chatbot

**Feature**: 001-ai-textbook-rag-chatbot  
**Phase**: 0 - Research & Technology Selection  
**Date**: 2025-12-06

## Overview

This document consolidates research findings and technology decisions for implementing an AI-native textbook with embedded RAG chatbot. All technical unknowns identified during planning have been resolved through industry best practices, documentation review, and architecture patterns.

---

## 1. RAG Architecture & Vector Search

### Decision: Qdrant Cloud + LangChain/LlamaIndex

**Rationale**:
- **Qdrant Cloud** chosen for vector storage:
  - Free tier supports 1M vectors (sufficient for ~1000 document chunks)
  - Managed service reduces operational overhead
  - Native filtering support for metadata (scope, language, module)
  - Python client with async support for FastAPI integration
  - Sub-100ms query latency for k=10 retrieval

- **LangChain vs LlamaIndex**:
  - **LangChain** preferred for this use case:
    - Better integration with custom retrieval logic (selection-mode enforcement)
    - More flexible prompt templating for personalization
    - Extensive agent orchestration capabilities for translation/content generation
    - Active community and OpenAI/Anthropic integration

**Alternatives Considered**:
- Pinecone: More expensive, no free tier with sufficient capacity
- Weaviate: More complex setup, heavier resource requirements
- ChromaDB: Good for local dev but limited cloud offering
- Pure LlamaIndex: Less flexible for custom retrieval logic

**Implementation Details**:
- Use `langchain.vectorstores.Qdrant` for vector operations
- Implement custom retriever for selection-mode that pre-filters by document IDs
- Chunk size: 400-800 tokens with 50-token overlap
- Embedding model: OpenAI `text-embedding-3-small` (1536 dimensions, cost-effective)
- Retrieval: Hybrid search (dense + keyword) for better accuracy

---

## 2. Document Chunking & Indexing Strategy

### Decision: Semantic Chunking with Markdown Structure Awareness

**Rationale**:
- Docusaurus content is markdown-based with clear heading hierarchy
- Semantic chunking preserves context better than fixed-size splits
- Heading-aware splitting maintains logical boundaries (H2/H3 sections)

**Chunking Strategy**:
1. Parse markdown AST to identify section boundaries
2. Split at H2/H3 headings (configurable)
3. Target 400-800 tokens per chunk (fit within ~2048 token LLM context)
4. Include parent heading in metadata for better retrieval context
5. Overlap: 50 tokens between adjacent chunks (preserve cross-boundary context)

**Metadata Schema**:
```python
{
    "chunk_id": "uuid",
    "chapter_id": "module-01-ros2/02-nodes-topics-services",
    "section": "Publisher/Subscriber Pattern",
    "heading_path": "Module 1 > ROS 2 > Publisher/Subscriber",
    "module": "module-01-ros2",
    "file_url": "/docs/module-01-ros2/02-nodes-topics-services",
    "lang": "en",
    "chunk_type": "content|code|exercise",
    "keywords": ["publisher", "subscriber", "rclpy", "topic"]
}
```

**Indexing Pipeline**:
- Script: `backend/scripts/ingest_to_qdrant.py`
- Process: Markdown → AST → Semantic chunks → Embeddings → Qdrant
- Batch size: 100 chunks per batch (avoid rate limits)
- Validation: Assert chunk count, sample retrieval test

**Alternatives Considered**:
- Fixed-size chunking: Loses semantic boundaries, lower retrieval quality
- Sentence-based: Too granular, increases vector count without benefit
- Full-document: Context too large, poor retrieval precision

---

## 3. Selection-Mode Enforcement

### Decision: Hybrid Fact-Checking with LLM Judge + String Matching

**Rationale**:
- Selection-mode requires 100% fact verification (all claims in selected text)
- Pure string matching too brittle (paraphrasing breaks it)
- Pure LLM judge too unreliable (hallucination risk)
- Hybrid approach: LLM generates answer → judge validates against selection

**Implementation**:
1. **Pre-filtering**: When user selects text, send selection + question to backend
2. **Retrieval bypass**: Don't search Qdrant; use selection as sole context
3. **System prompt**: "Answer ONLY using the provided text. If answer not in text, say 'I don't know.'"
4. **Post-generation validation**:
   - Extract claims from LLM response (using structured output)
   - For each claim, verify presence in selection text using:
     - Exact substring match (primary)
     - Semantic similarity >0.85 (fuzzy match for paraphrases)
   - If any claim fails: reject response, return "Unable to answer from selection"

**Validator Endpoint**:
```python
POST /api/validate-answer
{
    "answer": "LLM generated answer",
    "source_text": "User selected text",
    "mode": "strict|lenient"
}
Response: {
    "valid": true|false,
    "violations": ["claim X not found", ...],
    "confidence": 0.95
}
```

**Alternatives Considered**:
- Pure regex/string matching: Too brittle, high false negative rate
- No validation: Allows hallucination, violates acceptance criteria
- Separate fine-tuned judge model: Overkill for MVP, adds latency

---

## 4. User Authentication & Profile Storage

### Decision: Better-Auth + Neon Serverless Postgres

**Rationale**:
- **Better-Auth**:
  - Modern auth library with social OAuth2 support
  - Simple integration with FastAPI (session-based or JWT)
  - Built-in security best practices (password hashing, CSRF protection)
  - Lightweight vs Auth0/Firebase (no vendor lock-in)

- **Neon Serverless Postgres**:
  - Free tier: 0.5GB storage, sufficient for user profiles
  - Serverless architecture (auto-scale, pay-per-use)
  - PostgreSQL-compatible (standard SQL, migrations)
  - Built-in connection pooling for FastAPI async

**Schema**:
```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    name VARCHAR(255),
    password_hash VARCHAR(255),
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

CREATE TABLE user_profiles (
    user_id UUID PRIMARY KEY REFERENCES users(id),
    background VARCHAR(50) CHECK (background IN ('hardware', 'software', 'both', 'beginner')),
    difficulty_level VARCHAR(20) CHECK (difficulty_level IN ('beginner', 'intermediate', 'advanced')),
    examples_preference VARCHAR(20) CHECK (examples_preference IN ('code-heavy', 'theory-heavy', 'balanced')),
    localization VARCHAR(5) DEFAULT 'en' CHECK (localization IN ('en', 'ur')),
    consent_analytics BOOLEAN DEFAULT false,
    consent_personalization BOOLEAN DEFAULT false
);

CREATE TABLE answer_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id),
    question TEXT NOT NULL,
    scope VARCHAR(20) CHECK (scope IN ('fullbook', 'selected_text')),
    selected_text TEXT,
    retrieved_chunk_ids TEXT[],
    model_response TEXT,
    feedback_rating INTEGER CHECK (feedback_rating BETWEEN 1 AND 5),
    created_at TIMESTAMP DEFAULT NOW()
);
```

**Authentication Flow**:
1. Frontend: User signs up → POST `/api/auth/signup` → Better-Auth creates session
2. Backend: Store user in Neon, create default profile
3. Frontend: Store session token (httpOnly cookie)
4. Subsequent requests: Include session token → backend validates → retrieve user_id

**Alternatives Considered**:
- Firebase Auth: Vendor lock-in, overkill for simple use case
- Custom JWT: More secure but requires manual token management
- Supabase Auth: Good but less control over schema

---

## 5. LLM & Agent Orchestration

### Decision: OpenAI GPT-4o + LangChain Agents

**Rationale**:
- **GPT-4o** for primary Q&A:
  - Best balance of quality, speed, and cost
  - Strong instruction-following for selection-mode constraints
  - 128k context window (sufficient for multi-document retrieval)
  - Structured output support (JSON mode for fact extraction)

- **Claude 3.5 Sonnet** for content generation/translation:
  - Superior long-form content generation
  - Better multilingual performance (Urdu translation)
  - 200k context for processing full chapters

- **LangChain Agents**:
  - Orchestrate multi-step workflows (retrieve → answer → validate)
  - Tool integration for Qdrant, translation API, personalization
  - Streaming support for better UX (progressive answer display)

**Agent Architecture**:
```
User Question → RAG Agent → Tools:
  1. Retriever Tool (Qdrant search or selection-text)
  2. Answerer Tool (GPT-4o with system prompt)
  3. Validator Tool (fact-check for selection-mode)
  4. Personalizer Tool (rewrite based on user profile)
```

**Prompt Engineering**:
- **System Prompt (RAG)**:
```
You are a helpful assistant for the "Physical AI & Humanoid Robotics" textbook.
Answer questions based ONLY on the retrieved documents provided.
{if scope == 'selected_text':}
CRITICAL: You may ONLY use information from the selected text below. If the answer requires knowledge outside the selected text, respond with "I cannot answer this question using only the selected text."
{/if}
Be concise, technical, and cite specific sections when possible.
```

- **Personalization Prompt**:
```
Rewrite the following answer for a user with:
- Background: {user.background}
- Difficulty Level: {user.difficulty_level}
- Examples Preference: {user.examples_preference}

Adjust technical depth, terminology, and example complexity accordingly.
```

**Alternatives Considered**:
- Llama 3 70B: Good but requires self-hosting (complex infrastructure)
- Mistral Large: Competitive but less mature tooling/integration
- Pure retrieval (no LLM): Cannot synthesize answers from multiple chunks

---

## 6. Translation Strategy (Urdu)

### Decision: Claude Code Subagent with Domain Glossary

**Rationale**:
- Urdu technical translation requires domain expertise (ROS 2, robotics terminology)
- Claude 3.5 Sonnet has strong multilingual capabilities
- Agent-based approach allows:
  - Glossary lookup for technical terms (preserve English where appropriate)
  - Contextual translation (chapter-aware)
  - Quality validation (back-translation check)

**Translation Pipeline**:
1. User clicks "Urdu" toggle on chapter
2. Frontend checks cache: if translation exists, return cached
3. If not cached: POST `/api/translate` → Backend triggers translation agent
4. Agent workflow:
   - Load chapter markdown + domain glossary
   - Translate using Claude with glossary context
   - Preserve code blocks, technical terms in English
   - Store in Neon `translations` table
5. Return translated markdown → Frontend renders

**Domain Glossary** (examples):
```json
{
    "ROS 2": "ROS 2 (Robot Operating System)",
    "node": "نوڈ",
    "topic": "ٹاپک",
    "publisher": "پبلشر",
    "subscriber": "سبسکرائبر",
    "URDF": "URDF (Unified Robot Description Format)"
}
```

**Quality Validation**:
- BLEU score >0.6 (compare with reference translations)
- Human spot-check for first 3 chapters
- User feedback mechanism (rate translation quality)

**Alternatives Considered**:
- Google Translate API: Poor for technical content, no context awareness
- Pre-translate all content: Expensive, inflexible for updates
- No translation: Misses bonus points and accessibility goal

---

## 7. Personalization Strategy

### Decision: Client-Side Rendering with Profile-Aware Content Variants

**Rationale**:
- Personalization should not modify source docs (preserve canonical content)
- Real-time personalization via LLM rewriting on-demand
- Cache personalized content per user+chapter in browser storage

**Implementation**:
1. **Profile-Based Hints**: Store user preferences (background, difficulty) in Neon
2. **Per-Chapter Personalize Button**: User clicks → trigger personalization
3. **Backend Processing**:
   - Load chapter content + user profile
   - Send to GPT-4o with personalization prompt
   - Adjust:
     - **Beginner**: Add definitions, simplify analogies, more code comments
     - **Advanced**: Remove basic explanations, add edge cases
     - **Hardware-focused**: Emphasize sensor calibration, wiring
     - **Software-focused**: Emphasize API usage, code patterns
4. **Response**: Personalized markdown → Frontend renders in-place (toggle back to original)

**Caching Strategy**:
- Store in IndexedDB (browser) per user+chapter+profile hash
- Invalidate on profile change or chapter update
- Backend cache (Redis): Store for 24h per user+chapter

**Alternatives Considered**:
- Pre-generate variants: Combinatorial explosion (4 backgrounds × 3 levels = 12 variants/chapter)
- No personalization: Misses bonus points
- Pure keyword replacement: Too simplistic, poor quality

---

## 8. Frontend Architecture (Docusaurus + Chat Widget)

### Decision: Docusaurus 3 + Custom React Plugin

**Rationale**:
- Docusaurus provides:
  - Markdown-based content management
  - Built-in search, navigation, versioning
  - React-based theming (easy to embed custom components)
  - Static site generation (fast, SEO-friendly)
  - GitHub Pages deployment

**Chat Widget Integration**:
- Implement as Docusaurus theme plugin
- Position: Fixed bottom-right corner (collapsible)
- Features:
  - Text input for questions
  - Message history (session-based)
  - Selection mode: Detect text selection → show "Ask about selection" button
  - Loading states, error handling
  - Personalize/Urdu toggle at chapter level (context-aware)

**Component Structure**:
```tsx
// src/theme/ChatWidgetPlugin.tsx
export function ChatWidget() {
  const [messages, setMessages] = useState([]);
  const [selection, setSelection] = useState(null);
  
  useEffect(() => {
    // Listen for text selection events
    document.addEventListener('mouseup', handleSelection);
  }, []);
  
  const handleAsk = async (question: string, mode: 'fullbook' | 'selected_text') => {
    const response = await api.answer({
      question,
      scope: mode,
      selected_text: mode === 'selected_text' ? selection : null,
      user_id: currentUser?.id
    });
    setMessages([...messages, { role: 'assistant', content: response.answer }]);
  };
  
  return (
    <ChatContainer>
      <MessageList messages={messages} />
      {selection && <SelectionModeButton onClick={() => handleAsk(input, 'selected_text')} />}
      <ChatInput onSubmit={(q) => handleAsk(q, 'fullbook')} />
    </ChatContainer>
  );
}
```

**State Management**:
- React Context for user session, profile
- Local storage for chat history (per-session)
- No Redux needed (simple state)

**Alternatives Considered**:
- MkDocs: Less flexible React integration
- GitBook: Not open-source, vendor lock-in
- Custom SSG: Reinventing the wheel

---

## 9. Deployment & CI/CD

### Decision: GitHub Actions + GitHub Pages (Frontend) + Cloud Run (Backend)

**Rationale**:
- **Frontend (Docusaurus)**:
  - GitHub Pages: Free, integrated with GitHub, perfect for static sites
  - GitHub Actions: Build on push to main → deploy to gh-pages branch
  - Custom domain support

- **Backend (FastAPI)**:
  - Google Cloud Run: Serverless, auto-scale, pay-per-use
  - Container-based (Docker): Easy local dev parity
  - Supports async Python (Uvicorn)
  - Free tier: 2M requests/month (sufficient for demo)

**CI/CD Workflows**:

1. **Build & Test Backend** (`.github/workflows/ci-backend.yml`):
```yaml
on: [push, pull_request]
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-python@v4
        with:
          python-version: '3.11'
      - run: pip install -r requirements-dev.txt
      - run: pytest backend/tests/ --cov --cov-report=xml
      - uses: codecov/codecov-action@v3
```

2. **Build Docusaurus** (`.github/workflows/build-docs.yml`):
```yaml
on: [push]
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: '18'
      - run: cd frontend && npm ci && npm run build
      - uses: actions/upload-artifact@v3
        with:
          name: docs-build
          path: frontend/build/
```

3. **Deploy to GitHub Pages** (`.github/workflows/deploy-pages.yml`):
```yaml
on:
  push:
    branches: [main]
jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
      - run: cd frontend && npm ci && npm run build
      - uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./frontend/build
```

4. **Deploy Backend to Cloud Run**:
```yaml
on:
  push:
    branches: [main]
jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: google-github-actions/setup-gcloud@v1
      - run: gcloud builds submit --tag gcr.io/$PROJECT_ID/rag-backend
      - run: gcloud run deploy rag-backend --image gcr.io/$PROJECT_ID/rag-backend --region us-central1
```

**Secrets Management**:
- Store in GitHub Secrets:
  - `QDRANT_API_KEY`
  - `NEON_URL`
  - `BETTER_AUTH_CLIENT_ID`
  - `BETTER_AUTH_CLIENT_SECRET`
  - `OPENAI_API_KEY`
  - `CLAUDE_API_KEY`
  - `GCP_PROJECT_ID` (for Cloud Run)

**Alternatives Considered**:
- Vercel: Good but less control over backend deployment
- Heroku: More expensive, slower cold starts
- AWS Amplify: Overkill, steeper learning curve

---

## 10. Local Development Environment

### Decision: Docker Compose for Backend Services

**Rationale**:
- Consistent dev environment across team members
- Avoid "works on my machine" issues
- Easy to spin up dependencies (Postgres, Qdrant)

**Docker Compose Setup** (`docker-compose.yml`):
```yaml
version: '3.8'
services:
  postgres:
    image: postgres:15
    environment:
      POSTGRES_PASSWORD: dev_password
      POSTGRES_DB: 
    ports:
      - "5432:5432"
    volumes:
      - postgres_data:/var/lib/postgresql/data

  qdrant:
    image: qdrant/qdrant:latest
    ports:
      - "6333:6333"
      - "6334:6334"
    volumes:
      - qdrant_data:/qdrant/storage

  backend:
    build: ./backend
    command: uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
    ports:
      - "8000:8000"
    environment:
      - DATABASE_URL=postgresql://postgres:dev_password@postgres:5432/
      - QDRANT_URL=http://qdrant:6333
      - OPENAI_API_KEY=${OPENAI_API_KEY}
    volumes:
      - ./backend:/app
    depends_on:
      - postgres
      - qdrant

volumes:
  postgres_data:
  qdrant_data:
```

**Usage**:
```bash
# Start all services
docker-compose up -d

# Run migrations
docker-compose exec backend alembic upgrade head

# Index sample docs
docker-compose exec backend python scripts/ingest_to_qdrant.py --docs ./docs

# Stop all services
docker-compose down
```

**Frontend Development** (separate terminal):
```bash
cd frontend
npm install
npm start  # Docusaurus dev server on http://localhost:3000
```

**Alternatives Considered**:
- Manual setup: Error-prone, time-consuming onboarding
- Kubernetes: Overkill for local dev
- Vagrant: Heavier than Docker, slower

---

## Summary of Key Decisions

| Component | Technology | Rationale |
|-----------|-----------|-----------|
| Vector DB | Qdrant Cloud | Free tier, fast, managed |
| LLM Framework | LangChain | Flexible, agent orchestration |
| Primary LLM | OpenAI GPT-4o | Quality, speed, structured output |
| Content LLM | Claude 3.5 Sonnet | Long-form generation, multilingual |
| Auth | Better-Auth | Modern, lightweight, no vendor lock-in |
| User DB | Neon Postgres | Serverless, free tier, PostgreSQL-compatible |
| Frontend | Docusaurus 3 | Markdown-based, React, static site |
| Backend | FastAPI | Async, fast, OpenAPI docs |
| Embeddings | text-embedding-3-small | Cost-effective, 1536 dims |
| Deployment | GitHub Pages + Cloud Run | Free/cheap, auto-scale |
| Local Dev | Docker Compose | Consistent, easy setup |

---

## Next Steps (Phase 1)

1. Define data models (document, user, session) → `data-model.md`
2. Design API contracts (OpenAPI schema) → `contracts/`
3. Create quickstart guide for local dev → `quickstart.md`
4. Update agent context files with technology stack

---

**Phase 0 Complete** ✅  
All technical unknowns resolved. Proceed to Phase 1: Design & Contracts.
