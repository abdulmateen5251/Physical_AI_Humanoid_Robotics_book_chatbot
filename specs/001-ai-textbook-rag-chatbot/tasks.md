# Tasks: Physical AI & Humanoid Robotics Textbook + RAG Chatbot

**Feature**: 001-ai-textbook-rag-chatbot  
**Branch**: `001-ai-textbook-rag-chatbot`  
**Generated**: 2025-12-06  
**Status**: Ready for implementation

---

## Overview

This task breakdown implements an AI-native Docusaurus textbook with embedded RAG chatbot supporting:
- Full-book and selection-mode Q&A
- User authentication and personalization
- Urdu translation
- 4 course modules (ROS 2, Gazebo, Isaac, VLA)

**Tech Stack**: Python 3.11+, FastAPI, Docusaurus 3, React 18, Qdrant Cloud, Neon Postgres, LangChain, OpenAI GPT-4o, Claude 3.5 Sonnet

**Total Tasks**: 84 tasks across 8 phases  
**Parallel Opportunities**: 32 parallelizable tasks marked with [P]

---

## User Stories (Priority Order)

| ID | Priority | Story | Test Criteria |
|----|----------|-------|---------------|
| US1 | P1 | RAG Q&A (Full-book) | Chatbot answers questions with >=90% accuracy, cites sources |
| US2 | P1 | RAG Q&A (Selection) | Answers constrained to selected text, 100% fact verification |
| US3 | P2 | Auth & Profiles | Better-Auth signup creates user + profile in Neon |
| US4 | P2 | Personalization | Per-chapter content adjusted by user background/difficulty |
| US5 | P3 | Urdu Translation | Chapter toggle fetches/displays Urdu translation |
| US6 | P3 | Content Modules | 4 modules (20 chapters) indexed and retrievable |

---

## Implementation Strategy

**MVP Scope** (v0.1-alpha): US1 + US6 (partial: Module 1-2 only)
- Minimal viable product: Basic RAG chatbot with 2 modules of content
- Goal: Demonstrate end-to-end RAG pipeline

**Beta Scope** (v0.2-beta): + US2 + US6 (Module 3)
- Add selection-mode enforcement
- Expand content to 3 modules

**v1.0 Scope**: + US3 + US4 + US5 + US6 (Module 4)
- Complete all bonus features
- Full 4-module content

---

## Dependency Graph

```
Phase 1 (Setup) → Phase 2 (Foundational)
                       ↓
         ┌─────────────┴─────────────┐
         ↓                           ↓
    Phase 3 (US1)              Phase 4 (US2)
    Full-book RAG              Selection Mode
         ↓                           ↓
         └─────────────┬─────────────┘
                       ↓
              Phase 5 (US3: Auth)
                       ↓
         ┌─────────────┴─────────────┐
         ↓                           ↓
    Phase 6 (US4)              Phase 7 (US5)
    Personalization            Translation
         ↓                           ↓
         └─────────────┬─────────────┘
                       ↓
              Phase 8 (US6: Content)
                       ↓
              Phase 9 (Polish)
```

**Parallel Execution**:
- US1 & US2: Backend RAG implementation can proceed in parallel with frontend chat widget
- US3 independent after US1/US2
- US4 & US5 can be developed in parallel (both depend on US3)
- US6 (content) can be developed incrementally alongside feature development

---

## Phase 1: Setup & Project Initialization

**Goal**: Initialize project structure, dependencies, and development environment

**Duration**: 1-2 days

### Tasks

- [X] T001 Create backend directory structure per plan.md (backend/app/{models,services,api,utils}, backend/tests/{unit,integration,acceptance}, backend/scripts)
- [X] T002 Create frontend directory structure per plan.md (frontend/{src/components,docs/module-01-ros2,static,tests/e2e})
- [X] T003 Initialize backend Python project with pyproject.toml or requirements.txt (FastAPI, uvicorn, pydantic, langchain, qdrant-client, psycopg2, openai, anthropic, python-dotenv, alembic)
- [X] T004 Initialize frontend Node.js project with package.json (Docusaurus 3, React 18, axios, better-auth-client)
- [X] T005 Create docker-compose.yml with postgres, qdrant, backend services per quickstart.md
- [X] T006 Create .env.example with all required environment variables (OPENAI_API_KEY, QDRANT_URL, DATABASE_URL, BETTER_AUTH_CLIENT_ID/SECRET)
- [X] T007 Create backend/app/config.py for environment variable loading and validation using pydantic BaseSettings
- [X] T008 Create backend/app/main.py with FastAPI app initialization, CORS middleware, and /health endpoint
- [X] T009 Create .gitignore for Python (.venv, __pycache__, .env) and Node.js (node_modules, build, .env)
- [X] T010 Create README.md with project overview, setup instructions reference to quickstart.md, and contributing guidelines
- [X] T011 Set up Alembic for database migrations in backend/alembic/ with env.py configured for async Postgres
- [X] T012 Create GitHub Actions workflow .github/workflows/ci-backend.yml for backend linting (black, ruff) and tests (pytest)
- [X] T013 Create GitHub Actions workflow .github/workflows/build-docs.yml for Docusaurus build validation

**Verification**:
- [X] `docker-compose up -d` starts all services without errors
- [X] `curl http://localhost:8090/health` returns 200 OK
- [X] `cd frontend && npm start` launches Docusaurus on http://localhost:3000
- [X] GitHub Actions workflows pass on push

---

## Phase 2: Foundational Infrastructure

**Goal**: Implement core infrastructure needed by all user stories (database schema, vector store setup, shared utilities)

**Duration**: 2-3 days

**Dependencies**: Phase 1 complete

### Tasks

- [X] T014 [P] Create Alembic migration 001_initial_schema.py with users, user_profiles, answer_sessions, translations tables per data-model.md
- [X] T015 [P] Create backend/app/models/document.py with DocumentChunk Pydantic model matching Qdrant schema from data-model.md
- [X] T016 [P] Create backend/app/models/user.py with User and UserProfile Pydantic models matching Postgres schema
- [X] T017 [P] Create backend/app/models/session.py with AnswerSession and Translation Pydantic models
- [X] T018 Create backend/app/services/qdrant_client.py with QdrantService class (init_collection, upsert_chunks, search methods)
- [X] T019 Create backend/app/services/database.py with async Postgres connection pool using asyncpg and get_db() dependency
- [X] T020 [P] Create backend/app/utils/embeddings.py with generate_embedding() function using OpenAI text-embedding-3-small
- [X] T021 [P] Create backend/app/utils/chunking.py with semantic_chunk_markdown() function (parse markdown AST, split at H2/H3, 400-800 tokens)
- [X] T022 [P] Create backend/app/utils/validators.py with validate_selection_answer() function for fact-checking (string match + semantic similarity)
- [X] T023 [P] Create backend/app/utils/prompts.py with system prompt templates (RAG_SYSTEM_PROMPT, SELECTION_MODE_PROMPT, PERSONALIZATION_PROMPT)
- [X] T024 Create backend/scripts/ingest_to_qdrant.py indexing pipeline (parse docs, chunk, embed, upsert) with --docs, --collection, --force-reindex flags
- [X] T025 Create backend/tests/conftest.py with pytest fixtures (mock_qdrant, mock_postgres, test_client)
- [X] T026 [P] Write backend/tests/unit/test_embeddings.py to verify embedding generation (mock OpenAI API, assert 1536-dim vector)
- [X] T027 [P] Write backend/tests/unit/test_chunking.py to verify semantic chunking (sample markdown → expected chunks with metadata)
- [X] T028 [P] Write backend/tests/unit/test_validators.py to verify selection-mode fact checking (valid/invalid answer examples)

**Verification**:
- [X] `alembic upgrade head` creates all tables without errors
- [X] `pytest backend/tests/unit/` passes all foundational tests
- [X] `python backend/scripts/ingest_to_qdrant.py --docs ./sample_docs --collection test` indexes successfully

---

## Phase 3: User Story 1 - RAG Q&A (Full-book Mode)

**Goal**: Implement core RAG chatbot with full-book retrieval and answer generation

**Priority**: P1 (MVP Critical)

**Duration**: 3-4 days

**Dependencies**: Phase 2 complete

**Independent Test Criteria**:
- ✅ Given a question about textbook content, chatbot returns answer with relevant source citations
- ✅ Retrieval accuracy: Top-5 chunks contain answer for >=90% of test questions
- ✅ Answer quality: LLM response includes correct facts and no hallucinations (validated by acceptance test suite)
- ✅ Chat widget displays answer with clickable source links

### Backend Tasks

- [X] T029 [US1] Create backend/app/services/retrieval.py with RetrievalService class (retrieve_chunks method with top_k, filters, scope handling)
- [X] T030 [US1] Create backend/app/services/llm.py with LLMService class (generate_answer method using LangChain + GPT-4o, streaming support)
- [X] T031 [US1] Implement LangChain RAG agent in backend/app/services/rag_agent.py with tools (retriever, answerer) and orchestration logic
- [X] T032 [US1] Create backend/app/api/retrieve.py with POST /api/retrieve endpoint (accepts RetrieveRequest, returns RetrieveResponse per openapi.yaml)
- [X] T033 [US1] Create backend/app/api/answer.py with POST /api/answer endpoint (accepts AnswerRequest scope=fullbook, orchestrates retrieval + LLM, returns AnswerResponse)
- [X] T034 [US1] Create backend/app/api/feedback.py with POST /api/feedback endpoint (stores feedback_rating and feedback_comment in answer_sessions)
- [X] T035 [US1] Write backend/tests/integration/test_qdrant.py to verify Qdrant retrieval (index sample docs, query, assert top-k results)
- [X] T036 [US1] Write backend/tests/integration/test_api_endpoints.py for /api/retrieve and /api/answer (mock Qdrant/LLM, assert response format)
- [X] T037 [US1] Create backend/tests/acceptance/module-01.json with 10 Q/A pairs for Module 1 RAG testing
- [X] T038 [US1] Write backend/tests/acceptance/test_rag_accuracy.py to run acceptance tests and calculate accuracy (>=90% required)

### Frontend Tasks

- [X] T039 [P] [US1] Create frontend/src/services/api.ts with API client functions (answerQuestion, retrieveChunks, submitFeedback using axios)
- [X] T040 [P] [US1] Create frontend/src/components/ChatWidget/index.tsx with collapsible chat container (bottom-right, expand/collapse)
- [X] T041 [P] [US1] Create frontend/src/components/ChatWidget/ChatInput.tsx with text input, send button, loading state
- [X] T042 [P] [US1] Create frontend/src/components/ChatWidget/MessageList.tsx with message display (user/assistant, timestamps, source citations)
- [X] T043 [P] [US1] Implement ChatWidget state management with React Context (messages, loading, error handling)
- [X] T044 [US1] Create frontend/src/theme/ChatWidgetPlugin.tsx as Docusaurus theme plugin to inject ChatWidget globally
- [X] T045 [US1] Register ChatWidget plugin in frontend/docusaurus.config.js with plugin configuration
- [X] T046 [US1] Style ChatWidget with CSS module frontend/src/components/ChatWidget/styles.module.css (responsive, accessible)
- [X] T047 [US1] Write frontend/tests/e2e/chat-widget.spec.ts with Playwright (load page, ask question, verify answer appears)

### Content Tasks

- [X] T048 [P] [US1] Create frontend/docs/module-01-ros2/01-introduction.md with course overview, learning objectives (800-1000 words)
- [X] T049 [P] [US1] Create frontend/docs/module-01-ros2/02-nodes-topics-services.md with ROS 2 basics, rclpy examples (1500 words, 3 code blocks)

**Parallel Execution Example**:
- Backend: T029-T034 (API endpoints) can be developed in parallel with T039-T046 (Frontend components)
- Content: T048-T049 can be written while backend/frontend are being developed
- Testing: T035-T038 (backend tests) parallel with T047 (frontend E2E)

**Phase 3 Verification**:
- [ ] User can open chat widget, ask "How do I create a publisher in rclpy?", and receive accurate answer with sources
- [ ] `pytest backend/tests/acceptance/test_rag_accuracy.py` shows >=90% accuracy on Module 1 questions
- [ ] E2E test passes: `npm run test:e2e -- chat-widget.spec.ts`

---

## Phase 4: User Story 2 - RAG Q&A (Selection Mode)

**Goal**: Enable users to select text and get answers constrained to only that selection

**Priority**: P1 (MVP Critical)

**Duration**: 2-3 days

**Dependencies**: Phase 3 complete (US1 working)

**Independent Test Criteria**:
- ✅ User can select text on a chapter page, "Ask about selection" button appears
- ✅ Question answered using only selected text (no external knowledge)
- ✅ Fact validation: 100% of answer claims present in selected text (automated verification)
- ✅ If answer requires info outside selection, system responds "I cannot answer this question using only the selected text"

### Tasks

- [ ] T050 [US2] Update backend/app/api/answer.py to handle scope=selected_text (bypass Qdrant retrieval, use selected_text directly as context)
- [ ] T051 [US2] Implement selection-mode system prompt enforcement in backend/app/services/llm.py (CRITICAL instruction to use only selected text)
- [ ] T052 [US2] Add post-generation validation call to backend/app/utils/validators.py in answer endpoint (validate each claim against selection)
- [ ] T053 [US2] Create POST /api/validate-answer endpoint in backend/app/api/validate.py for manual validation testing
- [ ] T054 [US2] Write backend/tests/unit/test_selection_mode.py with passing/failing examples (facts in selection vs hallucinations)
- [ ] T055 [US2] Create backend/tests/acceptance/selection-mode-tests.json with 20 selection-mode Q/A pairs
- [ ] T056 [US2] Write backend/tests/acceptance/test_selection_mode.py to verify 100% fact verification rate
- [ ] T057 [P] [US2] Create frontend/src/services/selection.ts with text selection detection (mouseup listener, getSelection API)
- [ ] T058 [P] [US2] Create frontend/src/components/SelectionMode/SelectionButton.tsx (appears near selected text with "Ask about selection")
- [ ] T059 [US2] Integrate SelectionMode into ChatWidget (detect selection, show SelectionButton, send selected_text with question)
- [ ] T060 [US2] Add selection mode indicator in chat UI (show selected text snippet in message, highlight selection-mode answers)
- [ ] T061 [US2] Write frontend/tests/e2e/selection-mode.spec.ts (select text, click button, ask question, verify answer constrained)

**Parallel Execution Example**:
- Backend: T050-T053 (API changes) parallel with T057-T060 (Frontend selection detection)
- Testing: T054-T056 (backend tests) parallel with T061 (E2E test)

**Phase 4 Verification**:
- [ ] User selects text "To create a publisher, use self.create_publisher(MsgType, 'topic', 10)", asks "What is the syntax?", gets answer referencing only that text
- [ ] `pytest backend/tests/acceptance/test_selection_mode.py` shows 100% fact verification
- [ ] E2E test passes: selection triggers button, answer is constrained

---

## Phase 5: User Story 3 - Authentication & User Profiles

**Goal**: Implement Better-Auth signup/signin and user profile management

**Priority**: P2 (Bonus Feature)

**Duration**: 3-4 days

**Dependencies**: Phase 3 complete (US1 as baseline), US2 optional

**Independent Test Criteria**:
- ✅ User can sign up with email/password, account created in Neon Postgres
- ✅ User can sign in, receive JWT token, authenticated requests work
- ✅ User profile with background, difficulty_level, examples_preference saved
- ✅ Profile preferences retrievable via GET /api/user/{id}/profile

### Backend Tasks

- [ ] T062 [US3] Install and configure Better-Auth SDK in backend/app/services/auth.py (OAuth2 password flow, JWT generation)
- [ ] T063 [US3] Create backend/app/models/auth.py with SignupRequest, SigninRequest, AuthResponse schemas per openapi.yaml
- [ ] T064 [US3] Implement POST /api/auth/signup endpoint in backend/app/api/auth.py (create user, hash password with bcrypt, create default profile)
- [ ] T065 [US3] Implement POST /api/auth/signin endpoint (validate credentials, generate JWT token, update last_login)
- [ ] T066 [US3] Implement POST /api/auth/signout endpoint (invalidate token, clear session)
- [ ] T067 [US3] Create authentication middleware in backend/app/middleware/auth.py (verify JWT, extract user_id, add to request.state)
- [ ] T068 [US3] Implement GET /api/user/{user_id}/profile endpoint (return user + profile, require auth)
- [ ] T069 [US3] Implement PUT /api/user/{user_id}/profile endpoint (update profile fields, validate enums, require auth + ownership)
- [ ] T070 [US3] Write backend/tests/integration/test_auth_flow.py (signup → signin → authenticated request → profile update)
- [ ] T071 [US3] Add auth middleware to protected endpoints (/api/personalize, /api/user/*)

### Frontend Tasks

- [ ] T072 [P] [US3] Install Better-Auth client SDK in frontend and create frontend/src/services/auth.ts (signup, signin, signout, getSession)
- [ ] T073 [P] [US3] Create frontend/src/pages/signup.tsx with signup form (email, password, name, profile fields)
- [ ] T074 [P] [US3] Create frontend/src/pages/signin.tsx with signin form (email, password)
- [ ] T075 [P] [US3] Create React Context frontend/src/context/AuthContext.tsx for user session management (currentUser, token, setUser)
- [ ] T076 [US3] Add authentication state to ChatWidget (show user name if logged in, "Sign in to personalize" prompt)
- [ ] T077 [US3] Create profile settings page frontend/src/pages/profile.tsx (view/edit background, difficulty, preferences)
- [ ] T078 [US3] Write frontend/tests/e2e/auth-flow.spec.ts (signup → signin → view profile → update profile)

**Parallel Execution Example**:
- Backend: T062-T066 (auth endpoints) parallel with T072-T075 (frontend auth components)
- Profile: T067-T069 (backend profile) parallel with T076-T077 (frontend profile UI)

**Phase 5 Verification**:
- [ ] User can sign up at /signup, redirected to chat with authenticated session
- [ ] User can sign in at /signin, JWT token stored, authenticated requests succeed
- [ ] Profile preferences saved and retrievable
- [ ] E2E test passes: complete auth flow

---

## Phase 6: User Story 4 - Content Personalization

**Goal**: Rewrite chapter content based on user profile (background, difficulty, examples preference)

**Priority**: P2 (Bonus Feature)

**Duration**: 2-3 days

**Dependencies**: Phase 5 complete (US3 for user profiles)

**Independent Test Criteria**:
- ✅ Logged-in user sees "Personalize" button on chapter pages
- ✅ Clicking "Personalize" triggers LLM rewrite based on profile (beginner → simpler, advanced → more complex)
- ✅ Personalized content cached (browser IndexedDB + backend Redis 24h TTL)
- ✅ User can toggle back to original content

### Backend Tasks

- [ ] T079 [US4] Create backend/app/models/personalization.py with PersonalizeRequest, PersonalizeResponse, PersonalizedContent schemas
- [ ] T080 [US4] Create backend/app/services/personalization.py with PersonalizationService (generate_cache_key, rewrite_content using GPT-4o)
- [ ] T081 [US4] Implement POST /api/personalize endpoint in backend/app/api/personalize.py (fetch profile, check cache, call LLM, cache result)
- [ ] T082 [US4] Set up Redis client in backend/app/services/cache.py with get/set methods (24h TTL for personalized content)
- [ ] T083 [US4] Add personalization prompt to backend/app/utils/prompts.py (adjust technical depth, terminology, example complexity)
- [ ] T084 [US4] Write backend/tests/integration/test_personalization.py (mock profile, assert beginner gets simpler content, advanced gets complex)

### Frontend Tasks

- [ ] T085 [P] [US4] Create frontend/src/components/PersonalizeButton/index.tsx (shows on chapter pages, triggers personalization API)
- [ ] T086 [P] [US4] Implement IndexedDB storage in frontend/src/services/storage.ts (cache personalized content per user+chapter)
- [ ] T087 [US4] Add personalization state to chapter pages (original vs personalized toggle, loading state)
- [ ] T088 [US4] Style personalized content indicator (badge showing "Personalized for [background] [level]")
- [ ] T089 [US4] Write frontend/tests/e2e/personalization.spec.ts (sign in, navigate to chapter, click Personalize, verify content changes)

**Parallel Execution Example**:
- Backend: T079-T083 (API + LLM) parallel with T085-T087 (frontend UI)
- Testing: T084 parallel with T089

**Phase 6 Verification**:
- [ ] User with background="software", difficulty="advanced" sees code-heavy, complex examples after personalization
- [ ] User with background="beginner" sees simplified explanations with more definitions
- [ ] Personalized content cached and loads instantly on revisit
- [ ] E2E test passes: personalization flow works end-to-end

---

## Phase 7: User Story 5 - Urdu Translation

**Goal**: Provide per-chapter Urdu translation toggle with domain-aware translation

**Priority**: P3 (Bonus Feature)

**Duration**: 2-3 days

**Dependencies**: Phase 3 complete (US1 as baseline), US5 can run parallel to US3/US4

**Independent Test Criteria**:
- ✅ User sees "Urdu" toggle button on chapter pages
- ✅ Clicking toggle fetches Urdu translation (cached if exists, else generates via Claude agent)



### Backend Tasks

- [ ] T090 [US5] Create backend/app/models/translation.py with TranslateRequest, TranslateResponse, Translation schemas
- [ ] T091 [US5] Create backend/app/services/translation.py with TranslationService (translate_chapter using Claude 3.5 Sonnet + glossary)

- [ ] T093 [US5] Implement POST /api/translate endpoint in backend/app/api/translate.py (check cache, call Claude agent, store in translations table)
- [ ] T094 [US5] Add translation quality validation (BLEU score calculation, log quality_score in database)
- [ ] T095 [US5] Write backend/tests/integration/test_translation.py (sample chapter → Urdu output, verify technical terms preserved)

### Frontend Tasks


- [ ] T097 [P] [US5] Implement translation fetching in frontend/src/services/api.ts (translateChapter function)
- [ ] T098 [US5] Add Urdu content rendering to chapter pages (detect lang='ur', apply RTL CSS, render translated markdown)
- [ ] T099 [US5] Style Urdu content with appropriate fonts (Noto Nastaliq Urdu or similar) and RTL layout
- [ ] T100 [US5] Write frontend/tests/e2e/translation.spec.ts (click Urdu toggle, verify Urdu content loads, technical terms intact)

**Parallel Execution Example**:
- Backend: T090-T094 (translation service) parallel with T096-T098 (frontend toggle)
- Testing: T095 parallel with T100

**Phase 7 Verification**:
- [ ] User clicks "Urdu" toggle, sees translated content with RTL layout
- [ ] Technical terms like "ROS 2", "publisher", "URDF" preserved correctly
- [ ] Translation cached, instant load on revisit
- [ ] BLEU score >= 0.6 for sample chapters
- [ ] E2E test passes: translation toggle works

---

## Phase 8: User Story 6 - Course Content (Modules 1-4)

**Goal**: Create and index all 4 course modules (20 chapters total)

**Priority**: P3 (Content Generation)

**Duration**: 5-7 days (can be done incrementally)

**Dependencies**: Phase 2 complete (indexing pipeline ready), can run parallel to all feature development

**Independent Test Criteria**:
- ✅ All 20 chapters exist as markdown in frontend/docs/
- ✅ Each chapter has: learning objectives, summary, 3+ code examples, 3+ exercises
- ✅ All chapters indexed to Qdrant (500-1000 chunks)
- ✅ RAG retrieval works for queries across all modules

### Content Creation Tasks

- [ ] T101 [P] [US6] Create frontend/docs/module-01-ros2/03-actions-and-services.md (actions, service patterns, 1500 words)
- [ ] T102 [P] [US6] Create frontend/docs/module-01-ros2/04-urdf-and-robot-description.md (URDF syntax, xacro, examples, 1500 words)
- [ ] T103 [P] [US6] Create frontend/docs/module-01-ros2/05-agent-bridge.md (LLM → ROS 2 integration pattern, 1200 words)
- [ ] T104 [P] [US6] Create frontend/docs/module-01-ros2/exercises.md (5 lab exercises with rubrics)
- [ ] T105 [P] [US6] Create frontend/docs/module-02-digital-twin/01-gazebo-setup.md (Gazebo installation, world creation, 1000 words)
- [ ] T106 [P] [US6] Create frontend/docs/module-02-digital-twin/02-urdf-to-sdf.md (conversion, Gazebo plugins, 1200 words)
- [ ] T107 [P] [US6] Create frontend/docs/module-02-digital-twin/03-sensor-simulation.md (depth cameras, LiDAR, IMU, 1500 words)
- [ ] T108 [P] [US6] Create frontend/docs/module-02-digital-twin/04-unity-visualization.md (Unity basics, ROS integration, 1000 words)
- [ ] T109 [P] [US6] Create frontend/docs/module-02-digital-twin/exercises.md (5 lab exercises)
- [ ] T110 [P] [US6] Create frontend/docs/module-03-isaac/01-intro-isaac.md (Isaac Sim overview, USD pipeline, 1200 words)
- [ ] T111 [P] [US6] Create frontend/docs/module-03-isaac/02-synthetic-data.md (dataset generation, labeling, 1500 words)
- [ ] T112 [P] [US6] Create frontend/docs/module-03-isaac/03-isaac-ros-integration.md (Isaac ROS nodes, perception, 1500 words)
- [ ] T113 [P] [US6] Create frontend/docs/module-03-isaac/04-nav2-for-humanoids.md (Nav2 config, path planning, 1200 words)
- [ ] T114 [P] [US6] Create frontend/docs/module-03-isaac/exercises.md (5 lab exercises)
- [ ] T115 [P] [US6] Create frontend/docs/module-04-vla/01-intro-vla.md (vision-language-action overview, 1000 words)
- [ ] T116 [P] [US6] Create frontend/docs/module-04-vla/02-whisper-and-speech.md (Whisper integration, speech pipeline, 1200 words)
- [ ] T117 [P] [US6] Create frontend/docs/module-04-vla/03-llm-planning-patterns.md (LLM → action sequences, 1500 words)
- [ ] T118 [P] [US6] Create frontend/docs/module-04-vla/04-safety-and-validators.md (safety constraints, validation, 1200 words)
- [ ] T119 [P] [US6] Create frontend/docs/module-04-vla/exercises.md (5 lab exercises)

### Code Examples Tasks

- [ ] T120 [P] [US6] Create examples/module1/ros2_pkg_service_action/ (rclpy service server/client code)
- [ ] T121 [P] [US6] Create examples/module1/urdf_example/ (2-joint arm URDF with xacro)
- [ ] T122 [P] [US6] Create examples/module1/agent_bridge_stub/ (LLM → ROS 2 action client stub)
- [ ] T123 [P] [US6] Create examples/module2/gazebo_worlds/ (sample .world files, launch scripts)
- [ ] T124 [P] [US6] Create examples/module2/sensor_plugins/ (depth camera, IMU plugin configs)
- [ ] T125 [P] [US6] Create examples/module3/isaac_sim_configs/ (USD scenes, synthetic data scripts)
- [ ] T126 [P] [US6] Create examples/module3/nav2_configs/ (Nav2 YAML configs for humanoid)
- [ ] T127 [P] [US6] Create examples/module4/whisper_pipeline/ (Whisper recorder, transcription script)
- [ ] T128 [P] [US6] Create examples/module4/planner_stub/ (LLM planner + validator stub)

### Indexing & Testing Tasks

- [ ] T129 [US6] Run indexing pipeline for all modules: `python backend/scripts/ingest_to_qdrant.py --docs frontend/docs --collection physical_ai_humanoid_robotics_course`
- [ ] T130 [US6] Verify chunk count in Qdrant (expect 500-1000 chunks, query collection stats)
- [ ] T131 [US6] Create acceptance tests for Modules 2-4: backend/tests/acceptance/module-{02,03,04}.json (10 Q/A pairs each)
- [ ] T132 [US6] Run full RAG accuracy test across all modules (target >=90% accuracy)

**Parallel Execution Example**:
- All chapter creation tasks (T101-T119) can be done in parallel by different content authors
- Code examples (T120-T128) can be done in parallel with chapter writing
- Indexing (T129) runs after content is complete

**Phase 8 Verification**:
- [ ] All 20 chapters visible in Docusaurus navigation
- [ ] Each chapter renders correctly with code blocks, examples, exercises
- [ ] Qdrant contains 500-1000 chunks
- [ ] RAG chatbot can answer questions from any module with >=90% accuracy
- [ ] Code examples run without errors (manual verification)

---

## Phase 9: Polish & Cross-Cutting Concerns

**Goal**: Deployment configs, CI/CD, documentation, performance optimization

**Duration**: 2-3 days

**Dependencies**: All user stories complete

### Tasks

- [ ] T133 [P] Create .github/workflows/deploy-pages.yml for Docusaurus deployment to GitHub Pages
- [ ] T134 [P] Create .github/workflows/deploy-backend.yml for backend deployment to Cloud Run (or chosen platform)
- [ ] T135 [P] Create .github/workflows/rag-acceptance-tests.yml to run acceptance tests in CI (schedule: nightly)
- [ ] T136 [P] Create backend/Dockerfile with multi-stage build (dependencies → app copy → CMD uvicorn)
- [ ] T137 [P] Add rate limiting to backend API (fastapi-limiter, 60 requests/minute per IP)
- [ ] T138 [P] Add request logging middleware in backend/app/middleware/logging.py (log all requests/responses, exclude /health)
- [ ] T139 [P] Add error tracking integration (Sentry or similar) in backend/app/main.py
- [ ] T140 [P] Optimize frontend bundle size (lazy load ChatWidget, code splitting for heavy components)
- [ ] T141 [P] Add OpenAPI documentation UI at /docs endpoint (FastAPI's built-in Swagger UI)
- [ ] T142 Create deployment guide docs/deployment.md (Cloud Run setup, GitHub Pages setup, secrets configuration)
- [ ] T143 Create contributing guide CONTRIBUTING.md (branching strategy, PR template, coding standards)
- [ ] T144 Update README.md with badges (build status, coverage, license), quickstart link, architecture diagram
- [ ] T145 Add LICENSE file (MIT for code, CC-BY-SA 4.0 for content as per constitution)
- [ ] T146 Create .github/PULL_REQUEST_TEMPLATE.md per copilot-instructions.md (summary, linked tasks, test plan, checklist)
- [ ] T147 Run security audit (npm audit, safety check for Python deps, fix vulnerabilities)
- [ ] T148 Run performance profiling on backend (locust load test, optimize slow endpoints)
- [ ] T149 Run accessibility audit on frontend (axe-core, WCAG 2.1 AA compliance)
- [ ] T150 Create demo video and screenshots for README (5-minute walkthrough of key features)

**Verification**:
- [ ] `git push origin main` triggers GitHub Actions, site deploys to GitHub Pages
- [ ] Backend deploys to Cloud Run, health check passes
- [ ] Acceptance tests run nightly in CI, results logged
- [ ] Load test handles 100 concurrent users without errors
- [ ] Accessibility audit passes with 0 critical issues

---

## Task Summary by Phase

| Phase | Tasks | Parallel | Duration | Dependencies |
|-------|-------|----------|----------|--------------|
| 1. Setup | T001-T013 (13) | 0 | 1-2 days | None |
| 2. Foundational | T014-T028 (15) | 11 | 2-3 days | Phase 1 |
| 3. US1 (RAG Full-book) | T029-T049 (21) | 8 | 3-4 days | Phase 2 |
| 4. US2 (Selection Mode) | T050-T061 (12) | 4 | 2-3 days | Phase 3 |
| 5. US3 (Auth) | T062-T078 (17) | 5 | 3-4 days | Phase 3 |
| 6. US4 (Personalization) | T079-T089 (11) | 4 | 2-3 days | Phase 5 |
| 7. US5 (Translation) | T090-T100 (11) | 4 | 2-3 days | Phase 3 |
| 8. US6 (Content) | T101-T132 (32) | 28 | 5-7 days | Phase 2 |
| 9. Polish | T133-T150 (18) | 10 | 2-3 days | All |
| **Total** | **150** | **74** | **22-32 days** | |

---

## Parallel Execution Strategies

### Strategy 1: MVP First (Fastest to Demo)

**Week 1-2**: Phase 1 + Phase 2 (Setup + Foundational)
**Week 3**: Phase 3 (US1) + Phase 8 (US6, Module 1-2 only) in parallel
**Result**: Working RAG chatbot with 2 modules (MVP)

### Strategy 2: Full Feature Development

**Week 1-2**: Phase 1 + Phase 2
**Week 3-4**: Phase 3 (US1) + Phase 4 (US2) in parallel
**Week 5-6**: Phase 5 (US3) + Phase 7 (US5) in parallel
**Week 7**: Phase 6 (US4)
**Week 8-9**: Phase 8 (US6, all modules)
**Week 10**: Phase 9 (Polish)
**Result**: Full v1.0 with all features

### Strategy 3: Content-First Approach

**Week 1-2**: Phase 1 + Phase 2
**Week 3-4**: Phase 8 (US6, all content) in parallel with Phase 3 (US1)
**Week 5-6**: Phase 4 (US2) + Phase 5 (US3) + Phase 7 (US5) in parallel
**Week 7**: Phase 6 (US4)
**Week 8**: Phase 9 (Polish)
**Result**: Content-complete early, features added incrementally

---

## Testing Strategy

### Unit Tests
- **Coverage Target**: >=80% for backend services and utilities
- **Location**: `backend/tests/unit/`
- **Run**: `pytest backend/tests/unit/ -v --cov`

### Integration Tests
- **Scope**: API endpoints, database operations, external services (Qdrant, LLM)
- **Location**: `backend/tests/integration/`
- **Run**: `pytest backend/tests/integration/ -v`

### Acceptance Tests
- **Scope**: RAG accuracy (>=90%), selection-mode verification (100%)
- **Location**: `backend/tests/acceptance/`
- **Data**: `module-{01,02,03,04}.json`, `selection-mode-tests.json`
- **Run**: `pytest backend/tests/acceptance/ -v`

### E2E Tests
- **Scope**: User flows (chat, signup, personalization, translation)
- **Location**: `frontend/tests/e2e/`
- **Tool**: Playwright
- **Run**: `npm run test:e2e`

### Manual Testing Checklist
- [ ] Chat widget responsive on mobile/tablet/desktop
- [ ] Selection mode works on various text lengths
- [ ] Personalization produces noticeably different content
- [ ] Urdu translation readable and correctly formatted
- [ ] Code examples in chapters copy-pasteable and runnable

---

## Definition of Done (per User Story)

**User Story is DONE when**:
1. ✅ All tasks for that story completed
2. ✅ Unit tests pass with >=80% coverage
3. ✅ Integration tests pass
4. ✅ Acceptance criteria verified (automated tests)
5. ✅ E2E test passes
6. ✅ Code reviewed and merged to feature branch
7. ✅ Documentation updated (API docs, user guide)
8. ✅ Manual testing completed (if applicable)
9. ✅ Performance verified (no regressions)
10. ✅ Deployed to staging and smoke-tested

---

## Risk Mitigation

| Risk | Impact | Mitigation |
|------|--------|----------|
| OpenAI API rate limits | High | Batch embedding generation, add delays, use caching |
| LLM hallucinations in selection mode | High | Strict validation, fact-checking, user feedback loop |
| Translation quality poor | Medium | Human review first 3 chapters, glossary refinement |
| Qdrant free tier exceeded | Medium | Monitor usage, optimize chunk count, upgrade if needed |
| Complex auth integration | Medium | Start with simple email/password, add OAuth2 later |
| Content generation time | Low | Parallelize across modules, use Claude for bulk generation |
| E2E tests flaky | Low | Add retries, use stable selectors, mock external APIs |

---

## Next Steps

1. **Review Tasks**: Validate task breakdown with team, adjust estimates
2. **Assign Tasks**: Distribute tasks based on parallel execution strategy
3. **Set Up Project Board**: Create GitHub Projects board with columns per phase
4. **Kick Off Phase 1**: Start with setup tasks, establish dev environment
5. **Daily Standups**: Track progress, identify blockers, adjust plan

---

**Generated**: 2025-12-06  
**Last Updated**: 2025-12-06  
**Total Tasks**: 150  
**Estimated Duration**: 22-32 days with parallel execution  
**MVP Ready**: Week 3 (Phase 3 complete + Module 1-2 content)  
**v1.0 Ready**: Week 10 (all phases complete)
