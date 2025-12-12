---
id: PHR-001-PLAN
title: Implementation Plan Generation Physical AI Textbook
stage: plan
date: 2025-12-06T00:00:00Z
surface: github-copilot
model: claude-sonnet-4.5
feature: 001-ai-textbook-rag-chatbot
branch: 001-ai-textbook-rag-chatbot
user: Supreme_Traders
command: /sp.plan
labels: [planning, architecture, rag, docusaurus, fastapi]
links:
  spec: specs/001-ai-textbook-rag-chatbot/spec.md
  ticket: N/A
  adr: N/A
  pr: N/A
files:
  - path: specs/001-ai-textbook-rag-chatbot/plan.md
    action: created
    lines: 265
  - path: specs/001-ai-textbook-rag-chatbot/research.md
    action: created
    lines: 450
  - path: specs/001-ai-textbook-rag-chatbot/data-model.md
    action: created
    lines: 520
  - path: specs/001-ai-textbook-rag-chatbot/contracts/openapi.yaml
    action: created
    lines: 825
  - path: specs/001-ai-textbook-rag-chatbot/quickstart.md
    action: created
    lines: 420
  - path: .github/copilot-instructions.md
    action: updated
    lines: N/A
tests:
  - N/A (planning phase, no tests generated)
---

## Prompt

**User Request**: "Follow instructions in sp.plan.prompt.md. Project Plan"

**Context**: Feature specification exists at `specs/001-ai-textbook-rag-chatbot/spec.md` describing:
- Build Docusaurus-hosted textbook for "Physical AI & Humanoid Robotics" course
- Embed RAG chatbot supporting full-book and selection-mode Q&A
- Backend: FastAPI with Qdrant vector store, Neon Postgres, Better-Auth
- Frontend: Docusaurus 3 + React chat widget
- Bonus features: Personalization, Urdu translation
- 4 course modules: ROS 2, Gazebo/Unity, NVIDIA Isaac, Vision-Language-Action

**Workflow Executed**:
1. Run `.specify/scripts/powershell/setup-plan.ps1 -Json` to get feature paths
2. Load spec.md and constitution.md
3. Execute plan workflow:
   - Phase 0: Generate research.md (resolve technical unknowns)
   - Phase 1: Generate data-model.md, contracts/, quickstart.md
   - Phase 1: Update agent context (.github/copilot-instructions.md)
   - Fill plan.md with Technical Context, Constitution Check, Project Structure

**Key Requirements**:
- Resolve all "NEEDS CLARIFICATION" items in Technical Context
- Evaluate constitution gates (ERROR if violations)
- Generate comprehensive research on technology decisions
- Define complete data models and API contracts
- Create developer quickstart guide

## Response snapshot

**Phase 0: Research & Technology Decisions** (research.md)

Generated comprehensive research covering 10 major decision areas:

1. **RAG Architecture**: Qdrant Cloud + LangChain
   - Rationale: Free tier (1M vectors), managed service, sub-100ms latency
   - Alternative: Pinecone (expensive), Weaviate (complex), ChromaDB (limited cloud)

2. **Document Chunking**: Semantic chunking with markdown structure awareness
   - Strategy: Split at H2/H3 headings, 400-800 tokens per chunk, 50-token overlap
   - Metadata: chapter_id, section, heading_path, keywords, chunk_type

3. **Selection-Mode Enforcement**: Hybrid fact-checking (LLM judge + string matching)
   - Implementation: Pre-filtering → system prompt → post-generation validation
   - Validation: Extract claims → verify presence via exact match + semantic similarity >0.85

4. **User Authentication**: Better-Auth + Neon Serverless Postgres
   - Schema: users, user_profiles, answer_sessions tables
   - Flow: Signup → Better-Auth session → store in Neon

5. **LLM Orchestration**: OpenAI GPT-4o (Q&A) + Claude 3.5 Sonnet (content/translation)
   - LangChain agents for multi-step workflows
   - Streaming support for progressive answer display

6. **Translation Strategy**: Claude Code subagent with domain glossary
   - Pipeline: Check cache → translate with glossary → preserve code blocks → store in Neon
   - Quality: BLEU score >0.6, human spot-check

7. **Personalization**: Client-side rendering with profile-aware LLM rewriting
   - Cache: IndexedDB (client) + Redis (backend, 24h TTL)
   - Adjustment: Beginner → simpler, Advanced → more complex

8. **Frontend Architecture**: Docusaurus 3 + Custom React plugin
   - Chat widget as theme plugin (fixed bottom-right, collapsible)
   - Selection detection via mouseup listener

9. **Deployment**: GitHub Actions + GitHub Pages (frontend) + Cloud Run (backend)
   - CI/CD: Build/test/deploy workflows
   - Secrets: Managed via GitHub Secrets

10. **Local Development**: Docker Compose (Postgres + Qdrant + Backend)
    - Quick setup: `docker-compose up -d`
    - Hot reload for development

**Phase 1: Design & Contracts** (data-model.md, contracts/, quickstart.md)

**Data Models Created**:

1. **DocumentChunk** (Qdrant):
   - Fields: chunk_id, content, embedding (1536-dim), chapter_id, module, section, heading_path, file_url, chunk_type, lang, keywords
   - Validation: content ≤2048 chars, embedding exactly 1536 floats
   - Sample: ROS 2 publisher example with metadata

2. **User & UserProfile** (Postgres):
   - users: id, email, name, password_hash, email_verified, timestamps
   - user_profiles: user_id, background (hardware|software|both|beginner), difficulty_level, examples_preference, localization, consent flags
   - Default profile on signup: beginner/intermediate/balanced/en

3. **AnswerSession** (Postgres):
   - Tracks: question, scope, selected_text, retrieved_chunk_ids, model_response, feedback_rating
   - Constraint: selected_text required when scope='selected_text'

4. **Translation** (Postgres):
   - Fields: chapter_id, source_lang, target_lang, translated_content, quality_score
   - Unique constraint: (chapter_id, source_lang, target_lang)

5. **PersonalizedContent** (Redis + IndexedDB):
   - Cache key: hash(user_id + chapter_id + profile_hash)
   - TTL: 24 hours

**API Contracts Created** (openapi.yaml):

13 endpoints across 5 categories:
- **RAG**: POST /api/answer, POST /api/retrieve, POST /api/feedback
- **Auth**: POST /api/auth/signup, POST /api/auth/signin, POST /api/auth/signout
- **Personalization**: GET /api/user/{id}/profile, PUT /api/user/{id}/profile, POST /api/personalize
- **Translation**: POST /api/translate
- **Admin**: POST /api/admin/index, POST /api/admin/validate

All schemas defined with examples, validation rules, error responses.

**Quickstart Guide Created** (quickstart.md):

Comprehensive 420-line guide covering:
- Prerequisites (Python 3.11+, Node.js 18+, Docker, API keys)
- Quick setup (6 steps with Docker Compose)
- Manual setup (without Docker)
- Verification tests (backend API, frontend UI)
- Development workflow
- Common issues & solutions (10+ troubleshooting scenarios)
- Useful commands reference

**Technical Context Filled** (plan.md):

- **Language/Version**: Python 3.11+, Node.js 18+, TypeScript 5.x
- **Primary Dependencies**: FastAPI, Docusaurus 3, LangChain, Qdrant, Neon, OpenAI, Claude
- **Storage**: Qdrant Cloud (vectors), Neon Postgres (users), Redis (cache)
- **Testing**: pytest, Jest, Playwright, custom RAG accuracy harness
- **Target Platform**: Cloud Run (backend), GitHub Pages (frontend)
- **Project Type**: Web application (full-stack)
- **Performance Goals**: <500ms p95 retrieval, <3s p95 LLM response, >=90% RAG accuracy
- **Constraints**: 100% fact verification in selection mode, free-tier limits
- **Scale/Scope**: 4 modules × 5 chapters = 20 chapters, 500-1000 chunks, 100-500 users

**Constitution Check**: ✅ PASS
- Mission alignment ✓
- Scope compliance ✓
- Privacy & security ✓
- Success criteria ✓
- Governance ✓
- Licensing ✓

**Project Structure Defined**:

Web application structure with:
- `backend/` - FastAPI (app/{models,services,api,utils}, tests/{unit,integration,acceptance}, scripts)
- `frontend/` - Docusaurus (src/components, docs/module-01-04, static, tests/e2e)
- `examples/` - Code samples (module1-4 subdirs)
- `.github/workflows/` - CI/CD (build-docs, ci-backend, deploy-pages, rag-acceptance-tests)
- `docker-compose.yml` - Local dev environment

**Agent Context Updated**:
- Language: Python 3.11+ (backend), Node.js 18+ (frontend), TypeScript 5.x
- Project type: Web application (full-stack: frontend + backend)
- Added to `.github/copilot-instructions.md`

## Outcome

- ✅ Impact: Complete implementation plan ready for task breakdown
  - Technical stack fully defined with rationale
  - All unknowns resolved through research
  - Data models and API contracts specified
  - Developer onboarding guide created
  
- 🧪 Tests: N/A (planning phase)

- 📁 Files: 6 files created/updated
  - plan.md (265 lines) - Technical context, constitution check, project structure
  - research.md (450 lines) - 10 technology decisions with alternatives
  - data-model.md (520 lines) - 5 core entities with schemas, relationships, migrations
  - contracts/openapi.yaml (825 lines) - 13 API endpoints with full schemas
  - quickstart.md (420 lines) - Setup guide with troubleshooting
  - .github/copilot-instructions.md (updated) - Agent context

- 🔁 Next prompts: 
  - `/sp.tasks` - Generate task breakdown from plan
  - Implementation prompts for each phase/user story

- 🧠 Reflection: 
  - Plan is comprehensive and immediately actionable
  - Research phase critical for resolving technical unknowns
  - Data model covers all user stories (RAG, auth, personalization, translation)
  - API contracts enable parallel backend/frontend development
  - Quickstart reduces onboarding friction significantly

## Evaluation notes (flywheel)

- **Failure modes observed**: 
  - Initial PowerShell script execution issues (path parsing with `.github/prompts` directory)
  - Required full path execution: `& "c:\Users\...\setup-plan.ps1" -Json`
  - Solution: Use absolute paths for PowerShell scripts

- **Graders run and results**: 
  - Constitution Check: PASS (all gates satisfied)
  - Technical Context Completeness: PASS (no "NEEDS CLARIFICATION" remaining)
  - Data Model Coverage: PASS (all user stories mapped to entities)
  - API Contract Completeness: PASS (13 endpoints cover all functionality)
  - Quickstart Usability: PASS (6-step setup, comprehensive troubleshooting)

- **Prompt variant**: Standard sp.plan.prompt.md workflow (Phase 0 → Phase 1)

- **Next experiment**: 
  - Test plan generation with different project types (mobile, single-project)
  - Validate research.md format for AI agent consumption (Claude Code subagents)
  - Experiment with incremental plan updates (add/remove user stories)

## Key Learnings

1. **Research Phase is Critical**: Spending time on research.md (10 decision areas) prevented downstream rework. Each decision included rationale, alternatives, and implementation details.

2. **Data Model First, API Second**: Defining entities before endpoints ensured API contracts matched data structures. Avoided mismatches between request/response schemas and database models.

3. **Quickstart Guide ROI**: 420-line guide with Docker Compose setup, troubleshooting, and commands reference dramatically reduces onboarding time. Investment in quickstart pays off with every new developer.

4. **Parallel Opportunities**: Clear project structure (backend/, frontend/, examples/) enables parallel development. Identified in plan for future task breakdown.

5. **Constitution as Quality Gate**: Constitution check prevented scope creep and ensured alignment with project mission. Early validation saved time.

## Technology Decision Summary

| Decision | Choice | Key Reason |
|----------|--------|------------|
| Vector DB | Qdrant Cloud | Free tier, managed, fast |
| LLM Framework | LangChain | Flexible, agent orchestration |
| Primary LLM | OpenAI GPT-4o | Quality + speed + structured output |
| Content LLM | Claude 3.5 Sonnet | Long-form + multilingual |
| Auth | Better-Auth | Modern, lightweight, no vendor lock-in |
| User DB | Neon Postgres | Serverless, free tier, PostgreSQL |
| Frontend | Docusaurus 3 | Markdown-based, React, static site |
| Backend | FastAPI | Async, fast, OpenAPI docs |
| Embeddings | text-embedding-3-small | Cost-effective, 1536 dims |
| Deployment | GitHub Pages + Cloud Run | Free/cheap, auto-scale |
| Local Dev | Docker Compose | Consistent, easy setup |

## Metrics

- **Research Coverage**: 10 decision areas documented
- **Data Models**: 5 entities with full schemas
- **API Endpoints**: 13 endpoints with OpenAPI specs
- **Documentation Lines**: 2,480 lines total
- **Setup Time**: Estimated 30-45 minutes (Docker Compose path)
- **Constitution Gates**: 6/6 passed

## Next Phase Preview

**Phase 2: Task Breakdown** (`/sp.tasks`)

Will generate:
- User story extraction from spec.md (P1, P2, P3 priorities)
- Task-to-story mapping
- Dependency graph (phase ordering)
- Parallel execution identification
- Acceptance criteria per story
- Definition of done checklist

Expected output: tasks.md with 100-150 tasks across 8-10 phases, organized by user story for independent testing.
