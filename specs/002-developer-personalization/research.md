# Research: Developer Personalization System

**Feature**: 002-developer-personalization  
**Date**: 2025-12-16  
**Status**: Complete

## Overview

This document consolidates research findings for implementing the developer personalization system, including authentication strategy, recommendation algorithms, privacy controls, and accessibility patterns.

## R1: Better Auth Integration Strategy

### Decision

Use Better Auth with email/password provider as the primary authentication method, with OAuth2 (GitHub, Google) as secondary options for developer convenience.

### Rationale

- Better Auth is explicitly required by PDCP Constitution (Principle 3: Security First)
- Better Auth provides built-in session management, CSRF protection, and secure cookie handling
- Email/password provider enables self-hosted control and reduces third-party dependencies
- OAuth2 providers (GitHub especially) align with developer audience expectations
- Better Auth supports TypeScript natively, matching our technical stack

### Alternatives Considered

- **NextAuth.js**: Popular but more complex configuration; Better Auth provides simpler API and better TypeScript support
- **Clerk**: Managed service with excellent UX but introduces vendor lock-in and ongoing costs; conflicts with self-hosted preference
- **Custom JWT implementation**: Maximum control but high security risk and maintenance burden; violates "Security First" principle

### Implementation Notes

- Install `better-auth` npm package
- Configure providers in `backend/src/modules/auth/config.ts`
- Use Better Auth React hooks for frontend authentication state
- Session stored in HTTP-only cookies (secure by default)
- CSRF tokens automatically managed

### References

- Better Auth Documentation: https://better-auth.com/docs
- OWASP Authentication Cheat Sheet: https://cheatsheetseries.owasp.org/cheatsheets/Authentication_Cheat_Sheet.html

---

## R2: Recommendation Algorithm Approach

### Decision

Implement a **rule-based scoring system** with weighted tag matching for MVP (M4), with architecture allowing future upgrade to collaborative filtering or ML-based recommendations.

### Rationale

- Rule-based approach is simple, deterministic, and explainable ("Why this?" requirement from milestones)
- Sufficient for initial 10,000 users and 500 content items
- Can be developed and tested within M4 (Weeks 7-8) timeline
- Transparent scoring aligns with privacy principle (users can understand why content is recommended)
- Lower computational cost than ML models (meets <2 second recommendation refresh requirement)

### Algorithm Design

```
RelevanceScore = (
  0.4 × LanguageMatch +
  0.3 × FrameworkMatch +
  0.2 × DifficultyMatch +
  0.1 × HardwareCompatibility
)

Where each component is normalized 0-1:
- LanguageMatch: Exact match = 1.0, partial match = 0.5, no match = 0
- FrameworkMatch: Exact match = 1.0, related framework = 0.6, no match = 0
- DifficultyMatch: Profile skill level ± 1 level = 1.0, otherwise decay
- HardwareCompatibility: OS/GPU match = 1.0, compatible = 0.7, incompatible = 0
```

### Alternatives Considered

- **Collaborative filtering**: Requires sufficient user interaction data (cold start problem); defer to post-MVP
- **ML-based embeddings**: Higher accuracy potential but adds complexity, training data requirements, and compute costs; violates simplicity for MVP
- **Pure popularity ranking**: Ignores personalization entirely; doesn't meet core feature requirements

### Implementation Notes

- Store recommendation scores in Redis with TTL (profile updates invalidate cache)
- Pre-compute scores for top 100 content items per user profile
- Fallback to popularity ranking when profile incomplete (FR-010)
- Log "Was this helpful?" feedback for future algorithm tuning

### References

- Content-based filtering overview: https://developers.google.com/machine-learning/recommendation/content-based/basics
- Tag-based recommendation systems: Research papers on weighted tag matching

---

## R3: Privacy-Respecting Analytics

### Decision

Implement **event-based analytics with explicit opt-in** using a simple internal event logging system (PostgreSQL table) with aggregation for dashboards, avoiding third-party analytics services for MVP.

### Rationale

- Aligns with Constitution Principle 2 (Privacy & Consent by Design)
- FR-012: Must respect opt-out preferences
- Self-hosted analytics provides full data control and avoids GDPR complexities with third-party processors
- Event table in PostgreSQL is simple, queryable, and sufficient for 10,000 user scale
- Can integrate with external analytics later if needed (e.g., Plausible, Umami)

### Event Schema

```sql
CREATE TABLE analytics_events (
  id UUID PRIMARY KEY,
  user_id UUID REFERENCES users(id) ON DELETE CASCADE,
  event_type VARCHAR(50) NOT NULL, -- 'profile_created', 'recommendation_viewed', 'template_selected'
  event_data JSONB,                -- Flexible payload for event-specific data
  timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  opted_in BOOLEAN NOT NULL DEFAULT false
);

-- Only log if user has opted in
INSERT INTO analytics_events (...) WHERE user.privacy_preferences.analytics_opted_in = true;
```

### Alternatives Considered

- **Google Analytics**: Easy integration but third-party data sharing, GDPR concerns, conflicts with privacy-first approach
- **Plausible/Umami**: Privacy-friendly but adds infrastructure dependency; defer to post-MVP
- **No analytics**: Violates Principle 5 (Observability & Learning) and prevents data-driven improvements

### Implementation Notes

- Create `analytics_events` table in Prisma schema
- Middleware checks `user.privacyPreferences.analyticsOptIn` before logging
- Dashboard queries aggregate events for metrics (M6: Observability)
- Retention policy: Delete events older than 90 days (configurable)

### References

- GDPR compliant analytics patterns: https://gdpr.eu/cookies/
- Privacy-first analytics tools: Plausible documentation

---

## R4: Accessibility Best Practices for Forms

### Decision

Implement **WCAG AA compliant form patterns** using semantic HTML, ARIA labels, keyboard navigation, and real-time validation feedback for the signup questionnaire.

### Rationale

- Constitution Principle 4: Accessibility & Inclusivity mandates WCAG AA compliance
- SC-007: 90% first-attempt signup completion requires clear, accessible form design
- Questionnaire is the critical path for user onboarding (P1 user story)
- Accessible forms improve usability for all users, not just those with disabilities

### Best Practices to Implement

1. **Semantic HTML**:
   - Use `<label>` elements explicitly associated with inputs (`for` attribute)
   - Use `<fieldset>` and `<legend>` for grouped questions
   - Use appropriate input types (`type="email"`, `type="text"`, `<select>`)

2. **Keyboard Navigation**:
   - Logical tab order (top-to-bottom, left-to-right)
   - Skip optional fields without confusion
   - Submit button accessible via Enter key

3. **ARIA Labels and Descriptions**:
   - `aria-describedby` for field help text
   - `aria-invalid` and `aria-errormessage` for validation errors
   - `aria-required` for required fields

4. **Validation Feedback**:
   - Real-time validation on blur (not on every keystroke)
   - Clear error messages adjacent to fields
   - Summary of errors at form top for screen readers

5. **Visual Design**:
   - Sufficient color contrast (4.5:1 minimum for text)
   - Error indicators not relying solely on color (use icons + text)
   - Focus visible styles for keyboard navigation

### Alternatives Considered

- **Form library (React Hook Form, Formik)**: These libraries help with state management but don't guarantee accessibility; still need manual ARIA implementation
- **Headless UI libraries (Radix, Headless UI)**: Provide accessible primitives; good option for complex components but may be overkill for questionnaire
- **Manual implementation with accessibility testing**: Chosen approach—gives full control and ensures we meet requirements

### Implementation Notes

- Use Vitest + Testing Library for unit tests with accessibility assertions
- Run axe-core accessibility linter in CI (M0 setup)
- Manual keyboard navigation testing during M1-M2 development
- Playwright e2e tests include accessibility checks

### References

- WCAG 2.1 Form Guidelines: https://www.w3.org/WAI/WCAG21/quickref/?tags=forms
- A11y Project Form Patterns: https://www.a11yproject.com/patterns/
- React Hook Form Accessibility: https://react-hook-form.com/advanced-usage#AccessibilityA11y

---

## R5: Database Schema Design for Profiles

### Decision

Use **PostgreSQL with Prisma ORM** for developer profiles, leveraging JSONB columns for flexible software/hardware context storage while maintaining relational integrity for user and content relationships.

### Rationale

- PostgreSQL chosen in Technical Context for relational data and JSONB support
- Prisma provides type-safe database access with TypeScript, reducing bugs
- JSONB allows flexible schema for diverse software stacks (languages, frameworks, tools) without rigid schema constraints
- Relational structure maintains data integrity for user-profile relationships
- Supports encryption at rest for sensitive fields (Principle 3: Security First)

### Schema Design

```prisma
model User {
  id              String   @id @default(uuid())
  email           String   @unique
  emailVerified   DateTime?
  createdAt       DateTime @default(now())
  updatedAt       DateTime @updatedAt
  profile         DeveloperProfile?
}

model DeveloperProfile {
  id                    String   @id @default(uuid())
  userId                String   @unique
  user                  User     @relation(fields: [userId], references: [id], onDelete: Cascade)
  
  // Software context (JSONB for flexibility)
  softwareStack         Json     // { languages: [], frameworks: [], tools: [] }
  
  // Hardware context (JSONB)
  hardwareEnvironment   Json     // { os: "", deviceType: "", gpuAvailable: boolean }
  
  // Learning goals (JSONB)
  learningGoals         Json     // { skillLevel: "", interests: [], projectTypes: [] }
  
  // Privacy preferences
  personalizationOptIn  Boolean  @default(true)
  analyticsOptIn        Boolean  @default(false)
  
  createdAt             DateTime @default(now())
  updatedAt             DateTime @updatedAt
  
  // Relationships
  recommendationScores  RecommendationScore[]
}

model ContentItem {
  id                String   @id @default(uuid())
  title             String
  description       String
  tags              Json     // { technologies: [], difficulty: "", category: "" }
  createdAt         DateTime @default(now())
  updatedAt         DateTime @updatedAt
  
  // Relationships
  recommendationScores RecommendationScore[]
}

model StarterTemplate {
  id                  String   @id @default(uuid())
  name                String
  description         String
  supportedTechnologies Json   // { languages: [], frameworks: [], os: [] }
  templateFiles       String   // URL or storage path
  createdAt           DateTime @default(now())
  updatedAt           DateTime @updatedAt
}

model RecommendationScore {
  id              String   @id @default(uuid())
  profileId       String
  profile         DeveloperProfile @relation(fields: [profileId], references: [id], onDelete: Cascade)
  contentId       String
  content         ContentItem @relation(fields: [contentId], references: [id], onDelete: Cascade)
  score           Float    // 0.0 - 1.0
  computedAt      DateTime @default(now())
  
  @@unique([profileId, contentId])
  @@index([profileId, score(sort: Desc)])
}
```

### Alternatives Considered

- **MongoDB**: NoSQL flexibility but loses relational integrity; Prisma supports Postgres better
- **Strict relational schema for software stack**: Too rigid; developer stacks vary widely (dozens of languages/frameworks)
- **Separate tables for languages/frameworks**: Overly normalized; JSONB provides better query performance for this use case

### Implementation Notes

- Encrypt `softwareStack` and `hardwareEnvironment` JSONB fields if they contain sensitive data (TBD based on threat modeling in M7)
- Use Prisma migrations to version schema changes
- Index `personalizationOptIn` for efficient query filtering
- Cache profile data in Redis after first load to reduce database hits

### References

- Prisma JSONB documentation: https://www.prisma.io/docs/concepts/components/prisma-client/working-with-fields/json-fields
- PostgreSQL JSONB indexing: https://www.postgresql.org/docs/current/datatype-json.html

---

## R6: Milestone-Driven Development Strategy

### Decision

Adopt the provided 12-week milestone plan (M0-M7) with **weekly status updates** and **feature flag-driven rollouts** to derisk delivery and enable incremental validation.

### Rationale

- Milestones align with P1→P2→P3 user story priorities (Profile → Recommendations → Templates)
- M0 establishes critical infrastructure (CI, Speckit, secrets management)
- M1-M2 deliver foundational value (auth + profile) before adding complexity
- Feature flags (mentioned in M1) enable testing in production without exposing incomplete features
- M7 hardening phase addresses deferred Constitution gates (accessibility, security, performance)

### Milestone Dependencies

```
M0 (Bootstrap) → enables all other milestones
  ↓
M1 (Auth & Signup Shell) → required for M2, M3, M4
  ↓
M2 (Profile & Consent) → required for M4
  ↓
M3 (Content Catalog) → required for M4
  ↓
M4 (Personalization Engine) → depends on M1, M2, M3
  ↓
M5 (Privacy & Data Rights) → can run parallel with M4
  ↓
M6 (Observability) → can run parallel with M5
  ↓
M7 (Beta Hardening) → final validation before launch
```

### Risk Mitigation Implementation

- **Auth provider changes**: Use Better Auth abstraction layer; configure multiple providers in M1
- **PII risk**: Implement encryption in M2; threat model in M7; minimal data collection (already in spec)
- **Cold-start recommendations**: Popularity fallback algorithm (FR-010) implemented in M4
- **Dependencies**: Documented in Technical Context; secrets management in M0; content seed in M3

### Implementation Notes

- Weekly status updates appended to `history/prompts/002-developer-personalization/status-YYYY-MM-DD.md`
- Milestone completion triggers `tasks.md` sync (via `/sp.tasks` command)
- Feature flags managed via environment variables and Prisma feature flag table
- Exit criteria must be met and verified before milestone completion

### References

- Feature flag best practices: https://martinfowler.com/articles/feature-toggles.html
- Agile milestone planning: https://www.atlassian.com/agile/project-management/milestones

---

## Summary

All research items resolved. Key decisions:
- **R1**: Better Auth with email/password + OAuth2
- **R2**: Rule-based recommendation scoring (weighted tag matching)
- **R3**: Self-hosted analytics with opt-in enforcement
- **R4**: WCAG AA form patterns with semantic HTML and ARIA
- **R5**: PostgreSQL + Prisma with JSONB for flexible profile schema
- **R6**: 12-week milestone plan with feature flags and weekly updates

**Status**: ✅ Phase 0 Complete — Ready for Phase 1 (Design & Contracts)
