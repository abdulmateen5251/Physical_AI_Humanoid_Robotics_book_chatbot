# Implementation Plan: Physical AI & Humanoid Robotics Textbook + RAG Chatbot

**Branch**: `001-ai-textbook-rag-chatbot` | **Date**: 2025-12-06 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-ai-textbook-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build an AI-native Docusaurus textbook for "Physical AI & Humanoid Robotics" course with an embedded RAG chatbot. The system enables:
- Full-book Q&A using vector search (Qdrant) and LLM responses
- Constrained Q&A mode where answers are strictly from user-selected text
- User signup/signin with Better-Auth and profile-based personalization
- Per-chapter Urdu translation toggle
- Four course modules covering ROS 2, Gazebo/Unity simulation, NVIDIA Isaac, and Vision-Language-Action integration

Technical approach combines Docusaurus for static site generation, FastAPI backend for RAG endpoints, Qdrant Cloud for vector storage, Neon Serverless Postgres for user data, and OpenAI/Claude agents for orchestration and content generation.

## Technical Context

**Language/Version**: Python 3.11+ (backend), Node.js 18+ (frontend), TypeScript 5.x  
**Primary Dependencies**: 
- Backend: FastAPI 0.104+, Uvicorn, Pydantic 2.x, LangChain/LlamaIndex, OpenAI SDK, Qdrant Client, psycopg2/asyncpg
- Frontend: Docusaurus 3.x, React 18+, Axios, Better-Auth client SDK
- Dev: pytest, pytest-asyncio, Black, Ruff, ESLint, Prettier  

**Storage**: 
- Vector DB: Qdrant Cloud (managed) for document embeddings
- Relational DB: Neon Serverless Postgres for user profiles, session metadata
- Static Assets: GitHub Pages or Vercel for Docusaurus site  

**Testing**: 
- Backend: pytest with pytest-asyncio for async tests, pytest-mock for mocking external APIs
- Frontend: Jest + React Testing Library for component tests
- E2E: Playwright for critical user flows (signup, chat widget, selection mode)
- RAG Accuracy: Custom test harness with 50 Q/A pairs per module (200 total)  

**Target Platform**: 
- Backend: Cloud-hosted (Cloud Run, Heroku, or Vercel Serverless Functions)
- Frontend: Static site on GitHub Pages or Vercel
- Local Dev: Docker Compose for backend + local Qdrant + Postgres  

**Project Type**: Web application (full-stack: frontend + backend)  

**Performance Goals**: 
- RAG retrieval: <500ms p95 for top-k retrieval (k=5-10)
- LLM response: <3s p95 for complete answer generation
- Indexing: Process 500+ document chunks in <5 minutes
- Frontend: Docusaurus build <2 minutes, site load <2s
- Concurrent users: Support 100 concurrent chat sessions  

**Constraints**: 
- RAG accuracy: >=90% on acceptance test suite (no hallucinations)
- Selection mode: 100% fact verification - all claims must be in selected text
- Token limits: Chunk size 400-800 tokens, max context 8k tokens to LLM
- Free-tier limits: Qdrant Cloud free tier (1M vectors), Neon free tier (0.5GB)
- Security: No PII in embeddings, API keys in secrets only  

**Scale/Scope**: 
- Content: 4 modules × ~5 chapters = 20 chapters (~50k words total)
- Vectors: ~500-1000 document chunks initially
- Users: Target 100-500 registered users for demo
- Code: Estimated 5k-10k LOC (backend + frontend + scripts)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

✅ **Mission Alignment**: Feature fully aligns with building an AI-native textbook with embedded RAG chatbot for Physical AI & Humanoid Robotics course.

✅ **Scope Compliance**: All components are in-scope:
- Docusaurus textbook ✓
- RAG chatbot with FastAPI backend ✓
- Qdrant vector store ✓
- Neon Postgres for user data ✓
- Better-Auth signup/signin ✓
- Personalization & Urdu translation ✓

✅ **Privacy & Security**: 
- No PII in vector embeddings (only sanitized content chunks)
- User credentials stored in Neon Postgres with explicit consent flags
- API keys managed via GitHub Secrets
- User data opt-in for personalization

✅ **Success Criteria**:
- Book deployed to GitHub Pages/Vercel ✓
- RAG accuracy >=90% on acceptance tests ✓
- Selection mode fact-checking enforced ✓
- Better-Auth integration with profile storage ✓
- Personalization per chapter ✓
- Urdu translation toggle ✓

✅ **Governance**: Spec-driven workflow followed; all changes tracked via spec IDs and task IDs in PRs.

✅ **Licensing**: MIT for code, CC-BY-SA 4.0 for content as specified.

**GATE STATUS**: ✅ PASS - No constitutional violations. Proceed to Phase 0.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Web application structure (frontend + backend)

backend/
├── app/
│   ├── main.py                    # FastAPI app entry point
│   ├── config.py                  # Environment & settings
│   ├── models/
│   │   ├── document.py            # Document/chunk models
│   │   ├── user.py                # User profile models
│   │   └── session.py             # Answer session models
│   ├── services/
│   │   ├── embeddings.py          # Embedding generation (OpenAI/HuggingFace)
│   │   ├── qdrant_client.py       # Qdrant vector operations
│   │   ├── retrieval.py           # RAG retrieval logic
│   │   ├── llm.py                 # LLM answer generation
│   │   ├── auth.py                # Better-Auth integration
│   │   └── translation.py         # Urdu translation agent
│   ├── api/
│   │   ├── index.py               # POST /api/index
│   │   ├── retrieve.py            # POST /api/retrieve
│   │   ├── answer.py              # POST /api/answer
│   │   ├── auth.py                # POST /api/signup, GET /api/user/{id}/profile
│   │   └── translate.py           # POST /api/translate
│   └── utils/
│       ├── validators.py          # Selection-mode fact checker
│       └── prompts.py             # System prompts
├── tests/
│   ├── unit/
│   │   ├── test_embeddings.py
│   │   ├── test_retrieval.py
│   │   ├── test_validators.py
│   │   └── test_llm.py
│   ├── integration/
│   │   ├── test_qdrant.py
│   │   ├── test_auth_flow.py
│   │   └── test_api_endpoints.py
│   ├── acceptance/
│   │   ├── module-01.json         # M1 Q/A test pairs
│   │   ├── module-02.json
│   │   ├── module-03.json
│   │   ├── module-04.json
│   │   └── test_rag_accuracy.py
│   └── conftest.py
├── scripts/
│   ├── ingest_to_qdrant.py        # Indexing pipeline
│   ├── generate_embeddings.py
│   └── validate_index.py
├── requirements.txt
├── requirements-dev.txt
└── Dockerfile

frontend/
├── docusaurus.config.js           # Docusaurus configuration
├── sidebars.js                    # Navigation structure
├── package.json
├── src/
│   ├── components/
│   │   ├── ChatWidget/
│   │   │   ├── index.tsx          # Main chat component
│   │   │   ├── ChatInput.tsx
│   │   │   ├── MessageList.tsx
│   │   │   └── SelectionMode.tsx
│   │   ├── PersonalizeButton/
│   │   │   └── index.tsx          # Per-chapter personalization
│   │   └── UrduToggle/
│   │       └── index.tsx          # Translation toggle
│   ├── services/
│   │   ├── api.ts                 # Backend API client
│   │   ├── auth.ts                # Better-Auth client
│   │   └── selection.ts           # Text selection handler
│   ├── pages/
│   │   ├── index.tsx              # Landing page
│   │   └── signup.tsx             # Signup page
│   └── theme/
│       └── ChatWidgetPlugin.tsx   # Docusaurus theme plugin
├── docs/
│   ├── module-01-ros2/
│   │   ├── 01-introduction.md
│   │   ├── 02-nodes-topics-services.md
│   │   ├── 03-actions-and-services.md
│   │   ├── 04-urdf-and-robot-description.md
│   │   ├── 05-agent-bridge.md
│   │   └── exercises.md
│   ├── module-02-digital-twin/
│   │   ├── 01-gazebo-setup.md
│   │   ├── 02-urdf-to-sdf.md
│   │   ├── 03-sensor-simulation.md
│   │   ├── 04-unity-visualization.md
│   │   └── exercises.md
│   ├── module-03-isaac/
│   │   ├── 01-intro-isaac.md
│   │   ├── 02-synthetic-data.md
│   │   ├── 03-isaac-ros-integration.md
│   │   ├── 04-nav2-for-humanoids.md
│   │   └── exercises.md
│   └── module-04-vla/
│       ├── 01-intro-vla.md
│       ├── 02-whisper-and-speech.md
│       ├── 03-llm-planning-patterns.md
│       ├── 04-safety-and-validators.md
│       └── exercises.md
├── static/
│   ├── img/
│   └── assets/
└── tests/
    └── e2e/
        ├── chat-widget.spec.ts
        ├── selection-mode.spec.ts
        └── auth-flow.spec.ts

examples/
├── module1/                        # ROS 2 code samples
│   ├── ros2_pkg_publisher_subscriber/
│   ├── ros2_pkg_service_action/
│   ├── urdf_example/
│   └── agent_bridge_stub/
├── module2/                        # Gazebo/Unity samples
│   ├── gazebo_worlds/
│   └── sensor_plugins/
├── module3/                        # Isaac samples
│   ├── isaac_sim_configs/
│   └── nav2_configs/
└── module4/                        # VLA samples
    ├── whisper_pipeline/
    └── planner_stub/

.github/
├── workflows/
│   ├── build-docs.yml             # Build Docusaurus
│   ├── ci-backend.yml             # Backend tests
│   ├── deploy-pages.yml           # Deploy to GitHub Pages
│   └── rag-acceptance-tests.yml   # RAG accuracy tests
└── copilot-instructions.md        # Already exists

docker-compose.yml                  # Local dev environment
README.md
LICENSE
```

**Structure Decision**: Web application structure chosen because the project has distinct frontend (Docusaurus + React) and backend (FastAPI) components. The `frontend/docs/` directory contains all course module content organized by module number. The `examples/` directory at root holds code samples referenced in chapters. Backend follows layered architecture (models, services, api) for clear separation of concerns.

## Complexity Tracking

**No violations detected** - Constitution Check passed. This section is not applicable as there are no gate violations requiring justification.

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
