---
id: PHR-002-TASKS
title: Task Breakdown Generation Physical AI Textbook
stage: tasks
date: 2025-12-06T00:00:00Z
surface: github-copilot
model: claude-sonnet-4.5
feature: 001-ai-textbook-rag-chatbot
branch: 001-ai-textbook-rag-chatbot
user: Supreme_Traders
command: /sp.tasks
labels: [tasks, user-stories, dependencies, parallel-execution]
links:
  spec: specs/001-ai-textbook-rag-chatbot/spec.md
  plan: specs/001-ai-textbook-rag-chatbot/plan.md
  ticket: N/A
  adr: N/A
  pr: N/A
files:
  - path: specs/001-ai-textbook-rag-chatbot/tasks.md
    action: created
    lines: 1050
tests:
  - N/A (task planning phase, no tests generated)
---

## Prompt

**User Request**: "Follow instructions in sp.tasks.prompt.md. Generate an actionable dependency order task.md for the future based on available design artifacts"

**Context**: All Phase 0 and Phase 1 artifacts complete:
- plan.md - Technical context, architecture, project structure
- research.md - 10 technology decisions documented
- data-model.md - 5 entities with schemas
- contracts/openapi.yaml - 13 API endpoints
- quickstart.md - Setup guide

**Workflow Executed**:
1. Run `.specify/scripts/powershell/check-prerequisites.ps1 -Json` to get feature dir and available docs
2. Load design documents (plan.md, spec.md, data-model.md, contracts/, research.md, quickstart.md)
3. Extract user stories from spec.md with priorities
4. Map data models to user stories
5. Map API endpoints to user stories
6. Generate task breakdown organized by user story (strict checklist format)
7. Create dependency graph showing story completion order
8. Identify parallel execution opportunities
9. Validate task completeness and format

**Key Requirements**:
- Tasks MUST follow strict format: `- [ ] [TaskID] [P?] [Story?] Description with file path`
- Organize by user story (not by technical layer)
- Each story must be independently testable
- Mark parallelizable tasks with [P]
- Include exact file paths in all implementation tasks
- Generate dependency graph
- Provide parallel execution examples per story

## Response snapshot

**User Stories Extracted** (from spec.md):

6 user stories identified in priority order:

1. **US1 (P1)**: RAG Q&A with Full-book Mode
   - Core chatbot: retrieve from Qdrant → LLM answer → cite sources
   - Test: >=90% accuracy on acceptance tests
   - Tasks: Backend (retrieval, LLM, RAG agent, APIs) + Frontend (chat widget) + Content (Module 1-2 samples)

2. **US2 (P1)**: RAG Q&A with Selection Mode
   - Constrained answers from user-selected text only
   - Test: 100% fact verification (all claims present in selection)
   - Tasks: Selection-mode enforcement, fact validator, frontend selection detection

3. **US3 (P2)**: User Authentication & Profiles
   - Better-Auth signup/signin + Neon Postgres profile storage
   - Test: Signup creates user + profile, authenticated requests work
   - Tasks: Auth endpoints, JWT middleware, profile CRUD, frontend auth UI

4. **US4 (P2)**: Content Personalization
   - Per-chapter content rewrite based on user profile (background, difficulty)
   - Test: Beginner gets simpler content, advanced gets complex
   - Tasks: Personalization service, LLM rewrite, cache (Redis + IndexedDB), frontend personalize button

5. **US5 (P3)**: Urdu Translation
   - Per-chapter translation toggle with domain glossary
   - Test: Translation quality >=0.6 BLEU, technical terms preserved
   - Tasks: Translation service (Claude agent), glossary, cache, frontend toggle

6. **US6 (P3)**: Course Content (Modules 1-4)
   - 20 chapters across 4 modules + code examples
   - Test: All chapters indexed, RAG retrieval works across modules
   - Tasks: Chapter creation (parallel), code examples, indexing, acceptance tests

**Task Organization Strategy**:

- **Phase 1**: Setup (13 tasks) - Project structure, Docker, CI/CD
- **Phase 2**: Foundational (15 tasks) - Database, models, core utilities
- **Phase 3**: US1 - RAG Full-book (21 tasks) - Core MVP
- **Phase 4**: US2 - Selection Mode (12 tasks)
- **Phase 5**: US3 - Auth (17 tasks)
- **Phase 6**: US4 - Personalization (11 tasks)
- **Phase 7**: US5 - Translation (11 tasks)
- **Phase 8**: US6 - Content (32 tasks)
- **Phase 9**: Polish (18 tasks) - Deployment, optimization

**Total**: 150 tasks across 9 phases

**Dependency Graph Generated**:

```
Setup → Foundational → US1 (Full-book RAG)
                         ↓
                    US2 (Selection Mode)
                         ↓
                    US3 (Auth & Profiles)
                         ↓
              ┌──────────┴──────────┐
              ↓                     ↓
        US4 (Personalization)  US5 (Translation)
              ↓                     ↓
              └──────────┬──────────┘
                         ↓
                    US6 (Content)
                         ↓
                      Polish
```

**Key Insights**:
- US1 & US2 are sequential (selection mode builds on full-book)
- US3 is independent after US1/US2
- US4 & US5 can run in parallel (both depend on US3)
- US6 (content) can start early and run parallel to feature development
- 74 out of 150 tasks (49%) are parallelizable

**Sample Tasks by Phase**:

**Phase 1: Setup** (T001-T013)
```
- [ ] T001 Create backend directory structure per plan.md (backend/app/{models,services,api,utils}, backend/tests/{unit,integration,acceptance}, backend/scripts)
- [ ] T002 Create frontend directory structure per plan.md (frontend/{src/components,docs/module-01-ros2,static,tests/e2e})
- [ ] T003 Initialize backend Python project with pyproject.toml or requirements.txt (FastAPI, uvicorn, pydantic, langchain, qdrant-client, psycopg2, openai, anthropic, python-dotenv, alembic)
...
```

**Phase 3: US1 - RAG Full-book** (T029-T049)
```
Backend:
- [ ] T029 [US1] Create backend/app/services/retrieval.py with RetrievalService class (retrieve_chunks method with top_k, filters, scope handling)
- [ ] T030 [US1] Create backend/app/services/llm.py with LLMService class (generate_answer method using LangChain + GPT-4o, streaming support)
- [ ] T031 [US1] Implement LangChain RAG agent in backend/app/services/rag_agent.py with tools (retriever, answerer) and orchestration logic

Frontend (parallel):
- [ ] T039 [P] [US1] Create frontend/src/services/api.ts with API client functions (answerQuestion, retrieveChunks, submitFeedback using axios)
- [ ] T040 [P] [US1] Create frontend/src/components/ChatWidget/index.tsx with collapsible chat container (bottom-right, expand/collapse)
- [ ] T041 [P] [US1] Create frontend/src/components/ChatWidget/ChatInput.tsx with text input, send button, loading state

Content (parallel):
- [ ] T048 [P] [US1] Create frontend/docs/module-01-ros2/01-introduction.md with course overview, learning objectives (800-1000 words)
- [ ] T049 [P] [US1] Create frontend/docs/module-01-ros2/02-nodes-topics-services.md with ROS 2 basics, rclpy examples (1500 words, 3 code blocks)
```

**Phase 8: US6 - Content** (T101-T132)
```
All 19 chapter creation tasks parallelizable:
- [ ] T101 [P] [US6] Create frontend/docs/module-01-ros2/03-actions-and-services.md (actions, service patterns, 1500 words)
- [ ] T102 [P] [US6] Create frontend/docs/module-01-ros2/04-urdf-and-robot-description.md (URDF syntax, xacro, examples, 1500 words)
- [ ] T103 [P] [US6] Create frontend/docs/module-01-ros2/05-agent-bridge.md (LLM → ROS 2 integration pattern, 1200 words)
...
- [ ] T119 [P] [US6] Create frontend/docs/module-04-vla/exercises.md (5 lab exercises)
```

**Format Validation**:

✅ All 150 tasks follow required format:
- Checkbox: `- [ ]` (100%)
- Task ID: T001-T150 sequential (100%)
- [P] marker: 74 tasks marked parallelizable (49%)
- [Story] label: US1-US6 on all story tasks (Phases 3-8)
- File paths: Included in all implementation tasks (100%)

**Independent Test Criteria per Story**:

- **US1**: Chatbot answers with >=90% accuracy, cites sources
- **US2**: Answers constrained to selection, 100% fact verification
- **US3**: Signup creates user + profile, auth requests work
- **US4**: Content adjusted by profile (beginner vs advanced)
- **US5**: Translation quality >=0.6 BLEU, terms preserved
- **US6**: 20 chapters indexed, retrieval works across modules

**Parallel Execution Strategies Provided**:

1. **MVP First** (Fastest to Demo)
   - Week 1-2: Setup + Foundational
   - Week 3: US1 + US6 (Module 1-2 only)
   - Result: Working RAG chatbot (MVP)

2. **Full Feature Development**
   - Week 1-2: Setup + Foundational
   - Week 3-4: US1 + US2 parallel
   - Week 5-6: US3 + US5 parallel
   - Week 7: US4
   - Week 8-9: US6 (all modules)
   - Week 10: Polish
   - Result: Full v1.0

3. **Content-First Approach**
   - Week 1-2: Setup + Foundational
   - Week 3-4: US6 (all content) parallel with US1
   - Week 5-6: US2 + US3 + US5 parallel
   - Week 7: US4
   - Week 8: Polish
   - Result: Content-complete early

**Additional Sections Created**:

- **Implementation Strategy**: MVP → Beta → v1.0 roadmap
- **Task Summary by Phase**: Table with task counts, parallel opportunities, duration, dependencies
- **Testing Strategy**: Unit, integration, acceptance, E2E with coverage targets
- **Definition of Done**: 10-point checklist per user story
- **Risk Mitigation**: 7 risks with impact and mitigation strategies
- **Next Steps**: Review, assign, project board setup, kickoff

## Outcome

- ✅ Impact: Complete, actionable task breakdown ready for implementation
  - 150 tasks organized by user story (not technical layer)
  - Each story independently testable
  - Clear dependencies and parallel opportunities
  - 3 execution strategies provided
  
- 🧪 Tests: Test strategy defined
  - Unit tests: >=80% coverage
  - Integration tests: API endpoints, database, external services
  - Acceptance tests: RAG accuracy >=90%, selection mode 100% verification
  - E2E tests: User flows with Playwright

- 📁 Files: 1 file created
  - tasks.md (1050 lines) - 150 tasks across 9 phases with dependency graph

- 🔁 Next prompts:
  - Implementation prompts for each task (T001-T150)
  - Red-green-refactor cycle for TDD approach
  - Feature-specific refinements

- 🧠 Reflection:
  - User story organization enables independent testing (critical for parallel teams)
  - 49% parallelizable tasks significantly reduces timeline (22-32 days vs 45+ sequential)
  - Content creation (US6) can start immediately and run parallel to all features
  - MVP scope (Week 3) provides early validation opportunity
  - Format validation ensures consistency for automation/parsing

## Evaluation notes (flywheel)

- **Failure modes observed**: None
  - Prerequisites check executed successfully
  - All design documents loaded and parsed correctly
  - User story extraction from spec.md worked well

- **Graders run and results**:
  - Format Validation: PASS (150/150 tasks follow checklist format)
  - Story Organization: PASS (tasks grouped by user story, not technical layer)
  - Independent Testability: PASS (each story has clear, verifiable test criteria)
  - Parallel Opportunities: PASS (74 tasks marked [P], 49% parallelizable)
  - File Path Completeness: PASS (all implementation tasks include exact paths)
  - Dependency Graph: PASS (9 phases with clear ordering, parallel branches identified)

- **Prompt variant**: Standard sp.tasks.prompt.md workflow

- **Next experiment**:
  - Test task generation with different story extraction methods (automated vs manual)
  - Experiment with finer-grained task breakdown (150 tasks might be too high-level for some teams)
  - Validate parallel execution strategies with actual team assignment

## Key Learnings

1. **User Story Organization is Superior**: Organizing by story (not layer) enables:
   - Independent testing per story (each story is a complete vertical slice)
   - Parallel team assignment (Team A: US1, Team B: US2)
   - Incremental delivery (ship US1, then US2, etc.)
   - Clear definition of done per story

2. **Parallelization Multiplier**: 49% of tasks parallelizable means:
   - 150 tasks × 1 day each = 150 days sequential
   - With parallelization: ~22-32 days with multiple contributors
   - ROI of clear parallel markers: 5-7x speedup

3. **Content as Parallel Track**: Content creation (US6, 32 tasks) can run parallel to all feature development:
   - Content authors work independently of engineers
   - Early content enables earlier testing of RAG pipeline
   - Avoids bottleneck at end ("waiting for content")

4. **MVP Scope Critical**: Defining MVP (US1 + partial US6) enables:
   - Week 3 demo/validation
   - Early feedback on core value proposition
   - De-risks architecture decisions before building bonus features

5. **Format Consistency Enables Automation**: Strict checklist format allows:
   - Automated parsing for project boards (GitHub Issues)
   - Progress tracking (X/150 tasks complete)
   - Dependency validation (check all [Story] tasks have dependencies met)

## Task Breakdown Statistics

| Metric | Value |
|--------|-------|
| Total Tasks | 150 |
| Parallelizable Tasks | 74 (49%) |
| Phases | 9 |
| User Stories | 6 |
| Avg Tasks per Story | 25 |
| Setup Tasks | 13 (9%) |
| Foundational Tasks | 15 (10%) |
| Feature Tasks | 104 (69%) |
| Polish Tasks | 18 (12%) |

## Dependency Analysis

**Critical Path** (must be sequential):
1. Phase 1: Setup (13 tasks) → 1-2 days
2. Phase 2: Foundational (15 tasks) → 2-3 days
3. Phase 3: US1 (21 tasks, 8 parallel) → 3-4 days
4. Phase 4: US2 (12 tasks, 4 parallel) → 2-3 days

**Parallel Branches** (can overlap):
- After US1/US2: US3 (Auth) → US4 (Personalization) & US5 (Translation) in parallel
- Anytime after Foundational: US6 (Content) runs independent

**Bottleneck Risks**:
- Phase 2 (Foundational) blocks all user stories → prioritize early
- US3 (Auth) blocks US4 & US5 → consider mock auth for parallel development
- Indexing (T129) requires content complete → incremental indexing helps

## Next Phase Preview

**Phase 3: Implementation** (Task execution)

Will execute:
- T001-T013: Setup phase (Docker, project structure, CI/CD)
- T014-T028: Foundational phase (database, models, utilities)
- T029-T049: US1 implementation (RAG full-book mode)
- Iterative red-green-refactor cycles for TDD

Expected approach:
- Use `/sp.red` for test writing (given task specification)
- Use `/sp.green` for implementation (make tests pass)
- Use `/sp.refactor` for code cleanup and optimization
- Track progress: tasks.md checkboxes → GitHub Issues → Project board
