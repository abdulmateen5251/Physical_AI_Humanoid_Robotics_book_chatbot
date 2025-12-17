---
description: "Task list for Developer Personalization System implementation"
---

# Tasks: Developer Personalization System

**Input**: Design documents from `/specs/002-developer-personalization/`  
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Tests are optional and included as separate tasks where beneficial for TDD approach.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `- [ ] [ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This is a **web application** with:
- Backend: `backend/src/`
- Frontend: `frontend/src/`
- Tests: `backend/tests/` and `frontend/tests/`

## Labels

- **infra**: Infrastructure, CI/CD, tooling
- **auth**: Authentication and session management
- **questionnaire**: Signup questionnaire UI and logic
- **profile**: Profile management and settings
- **content**: Content catalog and admin CRUD
- **recs**: Recommendation engine and scoring
- **privacy**: Privacy controls, consent, data rights
- **a11y**: Accessibility features and compliance
- **perf**: Performance optimization
- **obs**: Observability, analytics, metrics

---

## Phase 1: Setup (M0 - Week 1)

**Purpose**: Project bootstrap, infrastructure, and CI/CD setup

**Reference**: Milestone M0 from plan.md

### T001-T010: Repository & Infrastructure

- [x] T001 [P] [infra] Create repository structure per plan.md (backend/, frontend/, specs/, .github/)
- [x] T002 [P] [infra] Initialize package.json for backend with dependencies: better-auth, @prisma/client, express, typescript
- [ ] T003 [P] [infra] Initialize package.json for frontend with dependencies: next@14+, react@18+, better-auth (client)
- [x] T004 [infra] Configure TypeScript for backend in backend/tsconfig.json (Node.js 20.x target, strict mode)
- [ ] T005 [infra] Configure TypeScript for frontend in frontend/tsconfig.json (Next.js config, React JSX)
- [x] T006 [P] [infra] Set up ESLint configuration in .eslintrc.json (TypeScript rules, React hooks, accessibility)
- [x] T007 [P] [infra] Set up Prettier configuration in .prettierrc (consistent formatting)
- [x] T008 [infra] Create CI pipeline .github/workflows/ci.yml (lint, type-check, test on PRs)
- [x] T009 [P] [infra] Add CODEOWNERS file .github/CODEOWNERS (maintainer review required for specs/, src/)
- [x] T010 [P] [infra] Create PR template .github/pull_request_template.md (spec reference, test plan, checklist)

**Acceptance**: CI runs on PRs; lint and type-check pass; CODEOWNERS enforced

---

### T011-T015: Speckit Plus Workflow

- [ ] T011 [infra] Register SpecKit+ prompt in .specify/templates/commands/ (if not already present)
- [x] T012 [infra] Configure doc synchronization check in CI (.github/workflows/doc-sync.yml)
- [x] T013 [P] [infra] Create constitution compliance check script .specify/scripts/check-constitution.sh
- [x] T014 [infra] Add spec-plan-tasks sync validation to CI pipeline
- [x] T015 [infra] Test SpecKit+ workflow by running /sp.constitution, /sp.specify, /sp.plan, /sp.tasks

**Acceptance**: SpecKit+ commands execute successfully; doc sync job passes in CI

---

### T016-T020: Environment & Secrets

- [x] T016 [P] [infra] Create .env.example files for backend and frontend (all required env vars documented)
- [ ] T017 [P] [infra] Configure GitHub Secrets for CI (DATABASE_URL, AUTH_SECRET, test credentials)
- [ ] T018 [infra] Set up local PostgreSQL database (pdcp_dev) per research.md R5
- [ ] T019 [infra] Set up local Redis instance for caching (optional for M1-M3, required for M4)
- [x] T020 [infra] Document environment setup in specs/002-developer-personalization/quickstart.md (already exists, verify completeness)

**Acceptance**: Developers can clone repo and start dev servers following quickstart.md

---

## Phase 2: Foundational (M1 - Weeks 2-3)

**Purpose**: Authentication infrastructure and database schema (blocking prerequisites for all user stories)

**Reference**: Milestone M1 from plan.md, FR-001 from spec.md

### T021-T030: Database Schema & Migrations

- [x] T021 [infra] Initialize Prisma in backend: npx prisma init
- [x] T022 [infra] Define User model in backend/prisma/schema.prisma (per data-model.md)
- [x] T023 [P] [infra] Define DeveloperProfile model in backend/prisma/schema.prisma
- [x] T024 [P] [infra] Define ContentItem model in backend/prisma/schema.prisma
- [x] T025 [P] [infra] Define StarterTemplate model in backend/prisma/schema.prisma
- [x] T026 [P] [infra] Define RecommendationScore model in backend/prisma/schema.prisma
- [x] T027 [P] [infra] Define AnalyticsEvent model in backend/prisma/schema.prisma
- [ ] T028 [infra] Run first migration: npx prisma migrate dev --name init
- [ ] T029 [infra] Generate Prisma client: npx prisma generate
- [x] T030 [P] [infra] Create seed script backend/prisma/seed.ts with sample content (10 items, 5 templates)

**Acceptance**: Database schema matches data-model.md; migrations run successfully

---

### T031-T045: Better Auth Integration (auth)

- [ ] T031 [auth] Install better-auth in backend: npm install better-auth
- [ ] T032 [auth] Create auth configuration in backend/src/modules/auth/config.ts per research.md R1
- [ ] T033 [auth] Configure email/password provider in auth config
- [ ] T034 [P] [auth] Configure GitHub OAuth provider in auth config (optional, env-gated)
- [ ] T035 [P] [auth] Configure Google OAuth provider in auth config (optional, env-gated)
- [ ] T036 [auth] Implement auth routes in backend/src/modules/auth/routes.ts per contracts/auth.openapi.yml
- [ ] T037 [auth] POST /api/auth/signup endpoint (email validation, password hashing)
- [ ] T038 [auth] POST /api/auth/signin endpoint (credentials validation, session creation)
- [ ] T039 [auth] POST /api/auth/signout endpoint (session invalidation)
- [ ] T040 [auth] GET /api/auth/session endpoint (current session retrieval)
- [ ] T041 [P] [auth] GET /api/auth/oauth/:provider endpoint (OAuth initiation)
- [ ] T042 [P] [auth] GET /api/auth/oauth/:provider/callback endpoint (OAuth callback handler)
- [ ] T043 [auth] Create authentication middleware in backend/src/middleware/auth.ts (session verification, route protection)
- [ ] T044 [auth] Install better-auth client in frontend: npm install better-auth (client)
- [ ] T045 [auth] Create auth context provider in frontend/src/contexts/AuthContext.tsx (session state, auth methods)

**Acceptance**: Email/password signup works; OAuth providers configured (if env vars present); protected routes reject unauthenticated requests

---

### T046-T055: Signup UI Flow (auth)

- [ ] T046 [P] [auth] Create signup page in frontend/src/pages/auth/signup.tsx
- [ ] T047 [P] [auth] Create signin page in frontend/src/pages/auth/signin.tsx
- [ ] T048 [P] [auth] Create signout functionality in frontend (button/menu item)
- [ ] T049 [auth] Implement email/password signup form with validation (React Hook Form)
- [ ] T050 [auth] Implement email/password signin form with validation
- [ ] T051 [P] [auth] Add OAuth buttons for GitHub/Google signin (conditionally rendered if configured)
- [ ] T052 [auth] Handle authentication errors in UI (display user-friendly messages per contracts/auth.openapi.yml)
- [ ] T053 [auth] Implement session persistence (cookies, auto-redirect on auth state change)
- [ ] T054 [auth] Create protected route wrapper in frontend/src/components/auth/ProtectedRoute.tsx
- [ ] T055 [auth] Test authentication flow end-to-end (signup → signin → session → signout)

**Acceptance**: Happy-path auth works (signup, signin, signout); error handling UX meets accessibility standards

---

## Phase 3: User Story 1 - Developer Profile Creation (M1-M2 - Weeks 2-4)

**Goal**: Users complete signup questionnaire and create developer profile

**Reference**: User Story 1 (Priority P1) from spec.md, FR-002, FR-003, FR-004, FR-005

**Independent Test**: Create new account → complete questionnaire → verify profile saved → edit profile in settings

### T056-T070: Questionnaire UI (questionnaire, a11y)

- [ ] T056 [P] [US1] [questionnaire] Create questionnaire page frontend/src/pages/auth/questionnaire.tsx (rendered after signup)
- [ ] T057 [US1] [questionnaire] Define question schema in frontend/src/lib/validation/questionnaire-schema.ts (Zod validation per data-model.md)
- [ ] T058 [P] [US1] [questionnaire, a11y] Create Software Stack section component frontend/src/components/profile/SoftwareStackForm.tsx (languages, frameworks, tools)
- [ ] T059 [P] [US1] [questionnaire, a11y] Create Hardware Environment section component frontend/src/components/profile/HardwareForm.tsx (OS, device type, GPU)
- [ ] T060 [P] [US1] [questionnaire, a11y] Create Learning Goals section component frontend/src/components/profile/LearningGoalsForm.tsx (skill level, interests, project types)
- [ ] T061 [US1] [questionnaire, a11y] Implement progressive disclosure (collapse/expand sections, save progress between sections)
- [ ] T062 [US1] [questionnaire, a11y] Add ARIA labels and descriptions per research.md R4 (semantic HTML, keyboard navigation)
- [ ] T063 [US1] [questionnaire, a11y] Implement real-time validation with accessible error messages (aria-invalid, aria-errormessage)
- [ ] T064 [US1] [questionnaire] Add "Skip" functionality for optional fields (partial profile save per FR-004)
- [ ] T065 [US1] [questionnaire] Implement Privacy Consent toggles (personalizationOptIn, analyticsOptIn) with plain-language explanations
- [ ] T066 [US1] [questionnaire] Create onboarding completion handler (redirect to dashboard after submit)
- [ ] T067 [US1] [questionnaire] Add feature flag check FEATURE_QUESTIONNAIRE_ENABLED (skip if disabled)
- [ ] T068 [US1] [questionnaire, perf] Optimize questionnaire load time (<90 seconds to complete per FR-002 and SC-001)
- [ ] T069 [US1] [questionnaire, a11y] Run axe-core accessibility audit on questionnaire page (WCAG AA compliance)
- [ ] T070 [US1] [questionnaire, a11y] Manual keyboard navigation testing (tab order, skip links, submit on Enter)

**Acceptance**: Questionnaire renders; < 90 seconds to complete; keyboard navigable; skippable optional fields; privacy consents clear

---

### T071-T085: Profile API (profile)

- [ ] T071 [P] [US1] [profile] Create profile routes in backend/src/modules/profile/routes.ts per contracts/profiles.openapi.yml
- [ ] T072 [US1] [profile] POST /api/profile endpoint (create profile from questionnaire data per FR-002, FR-003)
- [ ] T073 [US1] [profile] Validate profile input with Zod schema backend/src/lib/validation/profile-schema.ts
- [ ] T074 [US1] [profile] Store profile in database with encrypted sensitive fields (per data-model.md privacy considerations)
- [ ] T075 [US1] [profile] GET /api/profile endpoint (retrieve current user profile per FR-005)
- [ ] T076 [US1] [profile] PATCH /api/profile endpoint (update existing profile per FR-005, FR-011)
- [ ] T077 [US1] [profile] Handle partial profile updates (allow updating individual sections)
- [ ] T078 [US1] [profile] Invalidate recommendation cache on profile update (delete Redis keys, trigger recompute)
- [ ] T079 [P] [US1] [profile] GET /api/profile/export endpoint (data portability, JSON export)
- [ ] T080 [P] [US1] [profile, privacy] DELETE /api/profile/delete endpoint (soft delete, 30-day fulfillment per FR-006 and constitution)
- [ ] T081 [US1] [profile] Create profile service backend/src/services/profile-service.ts (business logic layer)
- [ ] T082 [US1] [profile] Implement profile encryption utility backend/src/lib/crypto/encrypt-profile.ts (if threat modeling requires)
- [ ] T083 [US1] [profile] Add profile validation middleware backend/src/middleware/validate-profile.ts
- [ ] T084 [US1] [profile] Test profile CRUD operations (unit tests for service, integration tests for endpoints)
- [ ] T085 [US1] [profile] Test edge cases: duplicate profile creation (409 conflict), missing required fields (400 validation error)

**Acceptance**: Profile API endpoints work per OpenAPI spec; schema validation enforced; encryption applied (if required)

---

### T086-T095: Profile Settings UI (profile)

- [ ] T086 [P] [US1] [profile] Create profile settings page frontend/src/pages/settings/profile.tsx
- [ ] T087 [US1] [profile] Display current profile data (read-only view of softwareStack, hardwareEnvironment, learningGoals)
- [ ] T088 [US1] [profile] Create edit mode toggle (switch between view/edit modes)
- [ ] T089 [P] [US1] [profile] Reuse questionnaire components for editing (SoftwareStackForm, HardwareForm, LearningGoalsForm)
- [ ] T090 [US1] [profile] Implement save functionality (PATCH /api/profile)
- [ ] T091 [US1] [profile] Show success/error feedback on save
- [ ] T092 [P] [US1] [profile, privacy] Add privacy preferences section (toggle personalizationOptIn, analyticsOptIn)
- [ ] T093 [P] [US1] [profile, privacy] Add data export button (download JSON)
- [ ] T094 [P] [US1] [profile, privacy] Add data deletion button (request deletion with confirmation modal)
- [ ] T095 [US1] [profile] Test profile settings page end-to-end (load → edit → save → verify persisted)

**Acceptance**: Users can view and edit profile; changes persist; privacy controls functional

---

### T096-T100: User Story 1 Integration & Validation

- [ ] T096 [US1] Run end-to-end test: signup → questionnaire → profile creation → settings edit (Playwright test frontend/tests/e2e/user-story-1.spec.ts)
- [ ] T097 [US1] Verify acceptance scenario 1: new user completes auth + questionnaire → profile saved → redirected to dashboard
- [ ] T098 [US1] Verify acceptance scenario 2: user skips optional fields → partial profile saved → signup completes
- [ ] T099 [US1] Verify acceptance scenario 3: user edits profile in settings → changes reflected in GET /api/profile
- [ ] T100 [US1] Validate success criteria SC-001: 80% of users complete >= 50% of optional fields (instrumentation for M6)

**Acceptance**: User Story 1 fully functional and independently testable

---

## Phase 4: User Story 2 - Personalized Content Recommendations (M3-M4 - Weeks 5-8)

**Goal**: Users see personalized content recommendations based on their profile

**Reference**: User Story 2 (Priority P2) from spec.md, FR-007, FR-008, FR-010, FR-011

**Dependencies**: Requires User Story 1 (developer profiles) to be complete

**Independent Test**: Login with pre-configured profile → view recommendations → verify relevance matches profile attributes

### T101-T115: Content Catalog Infrastructure (content)

- [ ] T101 [P] [US2] [content] Create content routes in backend/src/modules/content/routes.ts per contracts/content.openapi.yml
- [ ] T102 [P] [US2] [content] Create admin content routes in backend/src/modules/content/admin-routes.ts
- [ ] T103 [US2] [content] POST /api/admin/content endpoint (create content item per data-model.md ContentItem)
- [ ] T104 [US2] [content] PATCH /api/admin/content/:id endpoint (update content tags, description, URL)
- [ ] T105 [US2] [content] DELETE /api/admin/content/:id endpoint (soft delete/archive content)
- [ ] T106 [US2] [content] GET /api/content endpoint (paginated content list with filtering per contracts/content.openapi.yml)
- [ ] T107 [US2] [content] Implement content filtering (by category, difficulty, technologies)
- [ ] T108 [US2] [content] Implement pagination (page, limit parameters)
- [ ] T109 [US2] [content] Create content service backend/src/services/content-service.ts
- [ ] T110 [US2] [content] Define tagging taxonomy backend/src/lib/content/taxonomy.ts (topics, difficulty levels, runtime needs)
- [ ] T111 [US2] [content] Validate content input backend/src/lib/validation/content-schema.ts (Zod schema per data-model.md)
- [ ] T112 [US2] [content] Update seed script backend/prisma/seed.ts with diverse content (Python, TypeScript, Rust examples; beginner to expert)
- [ ] T113 [US2] [content] Run seed script: npx prisma db seed
- [ ] T114 [US2] [content] Test admin CRUD operations (unit tests for service, integration tests for endpoints)
- [ ] T115 [US2] [content] Test content filtering and pagination (verify query parameters work correctly)

**Acceptance**: Admin can create/update/delete content; content catalog populated with seed data; filtering works

---

### T116-T125: Content Catalog UI (content)

- [ ] T116 [P] [US2] [content] Create content catalog page frontend/src/pages/content/index.tsx
- [ ] T117 [US2] [content] Implement content grid/list view (display title, description, tags)
- [ ] T118 [P] [US2] [content] Add filtering UI (dropdowns for category, difficulty; search input)
- [ ] T119 [US2] [content] Connect filters to GET /api/content query parameters
- [ ] T120 [US2] [content] Implement pagination controls (next/previous, page numbers)
- [ ] T121 [US2] [content] Add loading states (skeleton UI while fetching)
- [ ] T122 [US2] [content] Add empty state (when no content matches filters)
- [ ] T123 [US2] [content] Implement content click handler (navigate to contentUrl)
- [ ] T124 [US2] [content] Style content cards with technology badges (tags.technologies)
- [ ] T125 [US2] [content] Test content catalog UI (e2e test: load → filter → paginate → click content)

**Acceptance**: Users can view and filter content catalog; pagination works; UI responsive

---

### T126-T145: Recommendation Engine (recs)

- [ ] T126 [P] [US2] [recs] Create recommendations routes backend/src/modules/recommendations/routes.ts per contracts/content.openapi.yml
- [ ] T127 [US2] [recs] Create scoring service backend/src/modules/recommendations/scoring-service.ts per research.md R2
- [ ] T128 [US2] [recs] Implement calculateLanguageMatch function (exact match = 1.0, partial = 0.5)
- [ ] T129 [P] [US2] [recs] Implement calculateFrameworkMatch function (exact = 1.0, related = 0.6)
- [ ] T130 [P] [US2] [recs] Implement calculateDifficultyMatch function (skill level ± 1 level = 1.0, decay otherwise)
- [ ] T131 [P] [US2] [recs] Implement calculateHardwareMatch function (OS/GPU compatibility)
- [ ] T132 [US2] [recs] Implement calculateRelevanceScore function (weighted combination: 0.4 lang + 0.3 framework + 0.2 difficulty + 0.1 hardware)
- [ ] T133 [US2] [recs] Create recommendation generation service backend/src/services/recommendation-service.ts
- [ ] T134 [US2] [recs] Implement score computation for all content items (batch processing)
- [ ] T135 [US2] [recs] Store scores in RecommendationScore table (profileId, contentId, score, computedAt)
- [ ] T136 [US2] [recs] Implement score caching in Redis backend/src/lib/cache/redis-client.ts (top 100 per profile, 1-hour TTL)
- [ ] T137 [US2] [recs] GET /api/recommendations endpoint (retrieve top N recommendations for current user per contracts/content.openapi.yml)
- [ ] T138 [US2] [recs] Implement fallback logic (popularity ranking when profile incomplete per FR-010)
- [ ] T139 [US2] [recs] Generate "Why this?" explanation backend/src/modules/recommendations/explainer.ts (match profile attributes to content tags)
- [ ] T140 [US2] [recs] Implement cache invalidation on profile update (triggered by PATCH /api/profile)
- [ ] T141 [US2] [recs] POST /api/recommendations/feedback endpoint (record "Was this helpful?" responses)
- [ ] T142 [US2] [recs] Create recommendation feedback table/model (for future algorithm tuning)
- [ ] T143 [US2] [recs] Test scoring algorithm (unit tests: various profiles × content combinations)
- [ ] T144 [US2] [recs] Test recommendation endpoint (integration test: profile → recommendations → verify top-ranked items match profile)
- [ ] T145 [US2] [recs] Performance test: recommendation generation <2 seconds per SC-005

**Acceptance**: Recommendations ranked by relevance; caching works; <2 second response time; fallback to popularity when needed

---

### T146-T160: Recommendations UI (recs)

- [ ] T146 [P] [US2] [recs] Create recommendations page frontend/src/pages/recommendations/index.tsx
- [ ] T147 [P] [US2] [recs] Create recommendation card component frontend/src/components/recommendations/RecommendationCard.tsx
- [ ] T148 [US2] [recs] Display ranked recommendations (top N from GET /api/recommendations)
- [ ] T149 [US2] [recs] Show relevance score and rank (if useful for debugging; hide in production)
- [ ] T150 [US2] [recs] Display "Why this?" explanation (reason field from API response)
- [ ] T151 [US2] [recs] Add "Was this helpful?" feedback buttons (thumbs up/down or yes/no)
- [ ] T152 [US2] [recs] Connect feedback to POST /api/recommendations/feedback
- [ ] T153 [US2] [recs] Add refresh button (manual trigger to regenerate recommendations)
- [ ] T154 [US2] [recs] Show profile snapshot used for recommendations (languages, skill level from API response)
- [ ] T155 [US2] [recs] Handle empty state (no profile → prompt to complete questionnaire)
- [ ] T156 [US2] [recs] Handle no recommendations state (profile too niche → show popular content)
- [ ] T157 [US2] [recs] Integrate recommendations into dashboard frontend/src/pages/dashboard.tsx (widget with top 3-5 recommendations)
- [ ] T158 [US2] [recs] Test recommendations UI (e2e test: login → view recommendations → verify order)
- [ ] T159 [US2] [recs] Test feedback submission (click helpful → verify POST /api/recommendations/feedback)
- [ ] T160 [US2] [recs] Test profile update → recommendation refresh (edit profile → navigate to recommendations → verify updated)

**Acceptance**: Recommendations displayed with explanations; feedback submission works; refresh on profile update (next page load per FR-011)

---

### T161-T165: User Story 2 Integration & Validation

- [ ] T161 [US2] Run end-to-end test: login with profile → view recommendations → verify relevance (Playwright test frontend/tests/e2e/user-story-2.spec.ts)
- [ ] T162 [US2] Verify acceptance scenario 1: Python + ML profile → Python ML frameworks ranked high
- [ ] T163 [US2] Verify acceptance scenario 2: Windows + .NET profile → Windows/Azure resources prominent
- [ ] T164 [US2] Verify acceptance scenario 3: no profile preferences → generic recommendations (popularity fallback)
- [ ] T165 [US2] Validate success criteria SC-002: 70% relevance score from user feedback; SC-003: 40% increased content views for users with profiles

**Acceptance**: User Story 2 fully functional; recommendations match profile; independently testable

---

## Phase 5: User Story 3 - Context-Aware Starter Templates (M4 - Week 8)

**Goal**: Users see starter templates filtered by their profile

**Reference**: User Story 3 (Priority P3) from spec.md, FR-009

**Dependencies**: Requires User Story 1 (developer profiles)

**Independent Test**: Login with profile → browse templates → verify filtered by technologies

### T166-T180: Starter Templates Infrastructure (content)

- [ ] T166 [P] [US3] [content] Create template routes in backend/src/modules/templates/routes.ts
- [ ] T167 [P] [US3] [content] Create admin template routes in backend/src/modules/templates/admin-routes.ts
- [ ] T168 [US3] [content] POST /api/admin/templates endpoint (create template per data-model.md StarterTemplate)
- [ ] T169 [US3] [content] PATCH /api/admin/templates/:id endpoint (update template)
- [ ] T170 [US3] [content] DELETE /api/admin/templates/:id endpoint (delete template)
- [ ] T171 [US3] [content] GET /api/templates endpoint (list templates, personalized if authenticated per contracts/content.openapi.yml)
- [ ] T172 [US3] [content] Implement template filtering by profile (match languages, frameworks, OS from supportedTechnologies)
- [ ] T173 [US3] [content] Calculate match scores (count overlapping technologies, normalize 0-1)
- [ ] T174 [US3] [content] Sort templates by match score descending (personalized=true for authenticated users)
- [ ] T175 [US3] [content] GET /api/templates/:id/download endpoint (generate signed download URL or ZIP file)
- [ ] T176 [US3] [content] Create template service backend/src/services/template-service.ts
- [ ] T177 [US3] [content] Update seed script with starter templates backend/prisma/seed.ts (5+ templates: Node.js+Docker, Python+Django, React+TypeScript, etc.)
- [ ] T178 [US3] [content] Test template CRUD operations
- [ ] T179 [US3] [content] Test template filtering (various profiles → verify correct templates ranked first)
- [ ] T180 [US3] [content] Test download endpoint (generate URL, verify expiration)

**Acceptance**: Templates filtered by profile; match scores accurate; download endpoint works

---

### T181-T195: Starter Templates UI (content)

- [ ] T181 [P] [US3] [content] Create templates page frontend/src/pages/templates/index.tsx
- [ ] T182 [P] [US3] [content] Create template card component frontend/src/components/templates/TemplateCard.tsx
- [ ] T183 [US3] [content] Display template list (GET /api/templates)
- [ ] T184 [US3] [content] Show match score for authenticated users (highlight best matches)
- [ ] T185 [US3] [content] Show supported technologies badges (languages, frameworks, OS)
- [ ] T186 [US3] [content] Add "Download" button (calls GET /api/templates/:id/download)
- [ ] T187 [US3] [content] Handle download URL (open in new tab or trigger browser download)
- [ ] T188 [US3] [content] Add filtering UI (by language, framework)
- [ ] T189 [US3] [content] Implement sort options (best match, newest, popular)
- [ ] T190 [US3] [content] Show generic list for unauthenticated users (personalized=false)
- [ ] T191 [US3] [content] Add "Create Project" flow integration (if applicable, link to project creation wizard)
- [ ] T192 [US3] [content] Test templates page UI (e2e test: load → verify sorted by match score)
- [ ] T193 [US3] [content] Test download functionality (click download → verify URL generation)
- [ ] T194 [US3] [content] Test filtering and sorting
- [ ] T195 [US3] [content] Test unauthenticated access (verify generic list shown)

**Acceptance**: Templates displayed; match scores visible; download works; personalized for authenticated users

---

### T196-T200: User Story 3 Integration & Validation

- [ ] T196 [US3] Run end-to-end test: login with Node.js+Docker profile → browse templates → verify Node.js+Docker template ranked first (Playwright test frontend/tests/e2e/user-story-3.spec.ts)
- [ ] T197 [US3] Verify acceptance scenario 1: Node.js+Docker profile → Dockerfile-included templates prominent
- [ ] T198 [US3] Verify acceptance scenario 2: iOS/Swift profile → iOS templates ranked first, Android lower
- [ ] T199 [US3] Verify acceptance scenario 3: no profile → generic template list (all templates, no personalization)
- [ ] T200 [US3] Validate success criteria SC-004: 30% increase in template selection for users with profiles vs without

**Acceptance**: User Story 3 fully functional; templates filtered correctly; independently testable

---

## Phase 6: Privacy & Data Rights (M5 - Week 9)

**Purpose**: Privacy controls, data export, deletion flows per constitution Principle 2

**Reference**: Milestone M5 from plan.md, FR-006, FR-012, research.md R3

### T201-T215: Privacy Infrastructure (privacy)

- [ ] T201 [P] [privacy] Create privacy routes backend/src/modules/privacy/routes.ts
- [ ] T202 [privacy] Implement consent update endpoint PATCH /api/profile (already exists, verify privacy preferences handling)
- [ ] T203 [privacy] Create consent audit log backend/src/services/audit-service.ts (log all consent changes with timestamp)
- [ ] T204 [privacy] Implement data export service backend/src/services/export-service.ts
- [ ] T205 [privacy] GET /api/profile/export endpoint (generate JSON export per contracts/profiles.openapi.yml)
- [ ] T206 [privacy] Include profile data, analytics events (if opted in), recommendation history in export
- [ ] T207 [privacy] Implement data deletion service backend/src/services/deletion-service.ts
- [ ] T208 [privacy] DELETE /api/profile/delete endpoint (soft delete, schedule purge after 30 days per constitution)
- [ ] T209 [privacy] Create deletion request table/model (track deletion requests, status, scheduled purge date)
- [ ] T210 [privacy] Implement scheduled purge job backend/src/jobs/purge-deleted-profiles.ts (runs daily, purges expired soft-deletes)
- [ ] T211 [privacy, obs] Log all privacy-related actions (export, deletion requests) in audit log
- [ ] T212 [privacy] Test consent audit logging
- [ ] T213 [privacy] Test data export (verify all user data included, no PII leaks for other users)
- [ ] T214 [privacy] Test deletion flow (soft delete → verify data inaccessible → scheduled purge)
- [ ] T215 [privacy] Test analytics opt-out enforcement (verify events NOT logged when analyticsOptIn=false)

**Acceptance**: Consent changes logged; data export works; deletion request flow functional; 30-day grace period enforced

---

### T216-T230: Privacy UI (privacy)

- [ ] T216 [P] [privacy] Create privacy settings page frontend/src/pages/settings/privacy.tsx
- [ ] T217 [privacy] Display current consent settings (personalizationOptIn, analyticsOptIn)
- [ ] T218 [privacy] Add toggle switches for consent preferences (plain-language explanations per research.md R3)
- [ ] T219 [privacy] Show consent change confirmation dialog (ensure user understands impact)
- [ ] T220 [privacy] Implement save consent changes (PATCH /api/profile)
- [ ] T221 [privacy] Add "Download My Data" button (calls GET /api/profile/export)
- [ ] T222 [privacy] Handle download (trigger JSON file download in browser)
- [ ] T223 [privacy] Add "Delete My Account" button (with strong confirmation warning)
- [ ] T224 [privacy] Implement deletion confirmation modal (explain 30-day grace period, irreversibility)
- [ ] T225 [privacy] Call DELETE /api/profile/delete on confirmation
- [ ] T226 [privacy] Show deletion request status (pending deletion, scheduled purge date)
- [ ] T227 [privacy] Add "Cancel Deletion" option (within 30-day grace period)
- [ ] T228 [privacy] Test privacy settings UI (toggle consent → verify saved)
- [ ] T229 [privacy] Test data export UI (click download → verify JSON file)
- [ ] T230 [privacy] Test deletion UI (request deletion → verify confirmation → check status)

**Acceptance**: Privacy settings UI clear and accessible; data export downloads successfully; deletion request submitted with grace period

---

## Phase 7: Observability (M6 - Week 10)

**Purpose**: Event instrumentation, analytics, metrics dashboards

**Reference**: Milestone M6 from plan.md, FR-012, research.md R3

### T231-T245: Analytics Infrastructure (obs)

- [ ] T231 [P] [obs] Create analytics events table (already defined in data-model.md AnalyticsEvent, verify migration)
- [ ] T232 [obs] Create analytics service backend/src/services/analytics-service.ts
- [ ] T233 [obs] Implement logEvent function (check analyticsOptIn before logging)
- [ ] T234 [obs] Define event schemas backend/src/lib/analytics/event-schemas.ts (profile_created, profile_updated, recommendation_viewed, template_selected, content_clicked)
- [ ] T235 [obs] Instrument profile creation event (POST /api/profile → logEvent('profile_created'))
- [ ] T236 [P] [obs] Instrument profile update event (PATCH /api/profile → logEvent('profile_updated'))
- [ ] T237 [P] [obs] Instrument recommendation view event (GET /api/recommendations → logEvent('recommendation_viewed'))
- [ ] T238 [P] [obs] Instrument template selection event (GET /api/templates/:id/download → logEvent('template_selected'))
- [ ] T239 [P] [obs] Instrument content click event (frontend tracks clicks → POST /api/analytics/event)
- [ ] T240 [obs] Create analytics aggregation queries backend/src/services/metrics-service.ts (event counts, top content, user engagement)
- [ ] T241 [obs] Implement 90-day retention policy (delete old events, triggered by purge job)
- [ ] T242 [obs] Test event logging (verify events only logged when analyticsOptIn=true)
- [ ] T243 [obs] Test event schemas (validate eventData structure)
- [ ] T244 [obs] Test opt-out enforcement (user opts out → no events logged)
- [ ] T245 [obs] Test retention policy (old events deleted after 90 days)

**Acceptance**: Events logged with opt-in enforcement; schemas validated; retention policy works

---

### T246-T260: Metrics Dashboard (obs)

- [ ] T246 [P] [obs] Create admin dashboard page frontend/src/pages/admin/dashboard.tsx (maintainers only)
- [ ] T247 [obs] Create metrics API backend/src/modules/admin/metrics-routes.ts
- [ ] T248 [obs] GET /api/admin/metrics/overview endpoint (total users, profiles, content, events)
- [ ] T249 [P] [obs] GET /api/admin/metrics/events endpoint (event counts by type, date range)
- [ ] T250 [P] [obs] GET /api/admin/metrics/content endpoint (top content by views/clicks)
- [ ] T251 [obs] Display overview metrics in dashboard (cards with counts)
- [ ] T252 [obs] Display event charts (bar chart of event counts over time)
- [ ] T253 [obs] Display top content list (ranked by engagement)
- [ ] T254 [obs] Add date range filter (last 7 days, 30 days, 90 days)
- [ ] T255 [obs] Implement real-time updates (refresh metrics every 60 seconds)
- [ ] T256 [obs] Add export metrics button (download CSV)
- [ ] T257 [obs] Test dashboard UI (load → verify metrics displayed)
- [ ] T258 [obs] Test date range filtering
- [ ] T259 [obs] Test metrics accuracy (compare dashboard to database queries)
- [ ] T260 [obs] Validate success criteria SC-002, SC-003, SC-004 (calculate metrics from events)

**Acceptance**: Metrics dashboard shows accurate data; event counts visible; top content ranked; date filters work

---

## Phase 8: Quality & Hardening (M7 - Weeks 11-12)

**Purpose**: Accessibility, performance, security hardening for beta launch

**Reference**: Milestone M7 from plan.md, Constitution Principle 4 (Accessibility), Principle 3 (Security)

### T261-T275: Accessibility Audit (a11y)

- [ ] T261 [P] [a11y] Run axe-core audit on all pages (automated WCAG AA checks)
- [ ] T262 [a11y] Fix color contrast issues (4.5:1 minimum for text per WCAG AA)
- [ ] T263 [a11y] Add focus visible styles (keyboard navigation indicators)
- [ ] T264 [a11y] Fix ARIA label issues (missing labels, incorrect roles)
- [ ] T265 [a11y] Test screen reader compatibility (NVDA on Windows, VoiceOver on macOS)
- [ ] T266 [a11y] Test keyboard navigation (tab order, skip links, no keyboard traps)
- [ ] T267 [a11y] Fix form validation accessibility (associate errors with fields, announce validation)
- [ ] T268 [a11y] Add landmarks and headings hierarchy (proper semantic structure)
- [ ] T269 [a11y] Test with keyboard only (no mouse usage)
- [ ] T270 [a11y] Manual testing with screen reader users (recruit 2-3 testers)
- [ ] T271 [a11y] Document accessibility features in docs/accessibility.md
- [ ] T272 [a11y] Add accessibility statement to footer
- [ ] T273 [a11y] Configure CI to run axe-core on every PR (.github/workflows/a11y.yml)
- [ ] T274 [a11y] Fix any remaining WCAG AA violations
- [ ] T275 [a11y] Validate SC-007: 90% first-attempt signup completion (accessibility contributes to this)

**Acceptance**: WCAG AA checks passed; screen reader compatible; keyboard navigable; CI enforces accessibility

---

### T276-T290: Performance Optimization (perf)

- [ ] T276 [P] [perf] Set performance budget targets (<200ms p95 API response, <2s recommendation generation per plan.md constraints)
- [ ] T277 [perf] Run Lighthouse audits on all pages (performance score >= 90)
- [ ] T278 [perf] Optimize questionnaire load time (<90s to complete, frontend bundle size)
- [ ] T279 [perf] Optimize recommendation endpoint (database query optimization, Redis caching verified)
- [ ] T280 [perf] Implement database indexes (profileId, score DESC on RecommendationScore per data-model.md)
- [ ] T281 [perf] Optimize content list query (pagination efficiency, add indexes on tags)
- [ ] T282 [perf] Implement image lazy loading (if applicable for content thumbnails)
- [ ] T283 [perf] Minimize frontend bundle size (code splitting, tree shaking)
- [ ] T284 [perf] Add CDN for static assets (if deploying to production)
- [ ] T285 [perf] Load testing: 1,000 concurrent users (per SC-005)
- [ ] T286 [perf] Monitor p95 response times (set up APM if needed)
- [ ] T287 [perf] Fix performance bottlenecks identified in load testing
- [ ] T288 [perf] Validate SC-005: profile updates + recommendation regeneration handles 1,000 concurrent users
- [ ] T289 [perf] Document performance targets in docs/performance.md
- [ ] T290 [perf] Add performance monitoring to CI (fail if Lighthouse score <90)

**Acceptance**: P95 API response <200ms; recommendation generation <2s; 1,000 concurrent users supported; Lighthouse score >=90

---

### T291-T305: Security Review (infra)

- [ ] T291 [P] [infra] Conduct threat modeling session (identify attack vectors, data flows)
- [ ] T292 [infra] Review authentication security (session management, CSRF protection, cookie settings)
- [ ] T293 [infra] Review input validation (SQL injection, XSS prevention via Zod schemas and sanitization)
- [ ] T294 [infra] Review authorization (ensure protected routes enforce authentication)
- [ ] T295 [infra] Review data encryption (sensitive profile fields per data-model.md privacy considerations)
- [ ] T296 [infra] Implement rate limiting backend/src/middleware/rate-limit.ts (prevent brute-force, DDoS)
- [ ] T297 [infra] Implement CORS configuration (restrict origins to frontend domain)
- [ ] T298 [infra] Review secrets management (verify no secrets in code, use env vars)
- [ ] T299 [infra] Implement security headers (helmet.js: CSP, HSTS, X-Frame-Options)
- [ ] T300 [infra] Run security scanning (npm audit, Snyk, or equivalent)
- [ ] T301 [infra] Fix high/critical vulnerabilities
- [ ] T302 [infra] Document security measures in docs/security.md
- [ ] T303 [infra] Create security incident response plan
- [ ] T304 [infra] Add security review to PR checklist
- [ ] T305 [infra] Validate constitution Principle 3 (Security First) compliance

**Acceptance**: Threat model documented; vulnerabilities fixed; security headers configured; secrets managed securely

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Final polish, documentation, deployment readiness

### T306-T320: Documentation & Deployment (infra)

- [ ] T306 [P] [infra] Update README.md with project overview, setup instructions, architecture
- [ ] T307 [P] [infra] Update quickstart.md with any changes from implementation
- [ ] T308 [P] [infra] Document API contracts (ensure OpenAPI specs up-to-date in contracts/)
- [ ] T309 [P] [infra] Create deployment guide docs/deployment.md (environment setup, migrations, secrets)
- [ ] T310 [infra] Set up staging environment (if applicable)
- [ ] T311 [infra] Configure production environment variables
- [ ] T312 [infra] Run database migrations in production
- [ ] T313 [infra] Seed production content (curated, not test data)
- [ ] T314 [infra] Configure monitoring and alerting (error tracking, uptime monitoring)
- [ ] T315 [infra] Set up backup and disaster recovery (database backups, restore process)
- [ ] T316 [infra] Create runbook for common operations (restart services, rollback deployment, handle incidents)
- [ ] T317 [infra] Validate all constitution gates resolved (re-run constitution check from plan.md)
- [ ] T318 [infra] Final smoke test in production (signup → questionnaire → profile → recommendations → templates)
- [ ] T319 [infra] Update tasks.md with actual vs estimated effort (for future planning)
- [ ] T320 [infra] Create launch checklist and go/no-go decision criteria

**Acceptance**: Documentation complete; staging and production deployed; monitoring configured; ready for beta launch

---

## Dependencies & Execution Order

**Story Dependencies**:
1. Phase 1 (Setup) → MUST complete before all others
2. Phase 2 (Foundational) → MUST complete before User Stories
3. User Story 1 (Profile Creation) → MUST complete before User Story 2 and 3
4. User Story 2 (Recommendations) → Independent of User Story 3 (can run in parallel)
5. User Story 3 (Templates) → Independent of User Story 2 (can run in parallel)
6. Phase 6-8 (Privacy, Observability, Hardening) → Can run in parallel after User Stories complete

**Parallelization Opportunities**:
- Setup tasks T001-T020: Many can run in parallel (marked with [P])
- User Story 2 & 3: Can be developed in parallel after User Story 1 completes
- Phase 6-8: Privacy, Observability, Hardening can overlap

**Critical Path**: Setup → Foundational → User Story 1 → User Story 2 (longer) → Hardening → Launch

**MVP Scope**: User Story 1 only (Profile Creation) delivers foundational value

---

## Implementation Strategy

**Milestone-Driven Delivery**:
- M0 (Week 1): Complete Phase 1 (Setup) - T001-T020
- M1-M2 (Weeks 2-4): Complete Phase 2 (Foundational) + Phase 3 (User Story 1) - T021-T100
- M3-M4 (Weeks 5-8): Complete Phase 4 (User Story 2) + Phase 5 (User Story 3) - T101-T200
- M5 (Week 9): Complete Phase 6 (Privacy) - T201-T230
- M6 (Week 10): Complete Phase 7 (Observability) - T231-T260
- M7 (Weeks 11-12): Complete Phase 8 (Hardening) + Phase 9 (Polish) - T261-T320

**Feature Flags**:
- FEATURE_QUESTIONNAIRE_ENABLED (M1-M2)
- FEATURE_RECOMMENDATIONS_ENABLED (M4)
- FEATURE_TEMPLATES_ENABLED (M4)

**Testing Strategy**:
- Unit tests: Run continuously during development
- Integration tests: Run after each phase completion
- E2E tests: Run after each user story completion
- Accessibility tests: Run in M7 hardening phase
- Performance tests: Run in M7 hardening phase

---

## Task Summary

**Total Tasks**: 320
- Phase 1 (Setup): 20 tasks
- Phase 2 (Foundational): 35 tasks
- Phase 3 (User Story 1): 45 tasks
- Phase 4 (User Story 2): 65 tasks
- Phase 5 (User Story 3): 35 tasks
- Phase 6 (Privacy): 30 tasks
- Phase 7 (Observability): 30 tasks
- Phase 8 (Hardening): 45 tasks
- Phase 9 (Polish): 15 tasks

**Parallelizable Tasks**: ~90 tasks (marked with [P])

**Independent Test Criteria**:
- User Story 1: Signup → questionnaire → profile saved → settings edit
- User Story 2: Login with profile → recommendations match attributes
- User Story 3: Login with profile → templates filtered correctly

**Format Validation**: ✅ All tasks follow checklist format with ID, [P] where applicable, [Story] label, and file paths
