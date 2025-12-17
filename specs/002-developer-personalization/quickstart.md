# Quickstart Guide: Developer Personalization System

**Feature**: 002-developer-personalization  
**Date**: 2025-12-16  
**Target Audience**: Developers implementing this feature

## Overview

This guide provides step-by-step instructions for implementing the developer personalization system, following the 12-week milestone plan (M0-M7).

## Prerequisites

- Node.js 20.x installed
- PostgreSQL 15+ running locally or accessible remotely
- Redis 7.x for caching (optional for M1-M3, required for M4)
- GitHub account for OAuth provider (optional)
- Basic familiarity with Next.js, Prisma, and TypeScript

## Development Environment Setup (M0)

### 1. Clone and Install Dependencies

```bash
# Clone repository
git clone <repository-url>
cd pdcp

# Install dependencies
npm install

# or in workspace structure:
cd backend && npm install
cd ../frontend && npm install
```

### 2. Configure Environment Variables

Create `.env` files in `backend/` and `frontend/`:

**backend/.env**:
```bash
# Database
DATABASE_URL="postgresql://user:password@localhost:5432/pdcp_dev"

# Redis (for M4+)
REDIS_URL="redis://localhost:6379"

# Better Auth
AUTH_SECRET="<generate-random-32-char-string>"
AUTH_URL="http://localhost:3000"

# OAuth Providers (optional for M1)
GITHUB_CLIENT_ID="<your-github-client-id>"
GITHUB_CLIENT_SECRET="<your-github-client-secret>"
GOOGLE_CLIENT_ID="<your-google-client-id>"
GOOGLE_CLIENT_SECRET="<your-google-client-secret>"

# Feature Flags
FEATURE_QUESTIONNAIRE_ENABLED="true"
FEATURE_RECOMMENDATIONS_ENABLED="false"  # Enable in M4
```

**frontend/.env**:
```bash
NEXT_PUBLIC_API_URL="http://localhost:3000/api"
```

### 3. Run Database Migrations

```bash
cd backend
npx prisma migrate dev --name init
npx prisma generate
```

### 4. Seed Development Data (M3+)

```bash
npm run seed
# Seeds 10 content items, 5 starter templates
```

### 5. Start Development Servers

```bash
# Terminal 1: Backend
cd backend
npm run dev  # Runs on http://localhost:3000

# Terminal 2: Frontend
cd frontend
npm run dev  # Runs on http://localhost:3001
```

## Implementation Path

### M1: Auth & Signup Shell (Weeks 2-3)

**Goal**: Users can sign up with Better Auth and see questionnaire UI

**Steps**:

1. **Install Better Auth**:
   ```bash
   cd backend
   npm install better-auth
   ```

2. **Configure Better Auth** (`backend/src/modules/auth/config.ts`):
   ```typescript
   import { createAuth } from 'better-auth';
   
   export const auth = createAuth({
     database: prisma, // Prisma client instance
     providers: {
       email: {
         enabled: true,
         // Email/password provider
       },
       github: {
         clientId: process.env.GITHUB_CLIENT_ID,
         clientSecret: process.env.GITHUB_CLIENT_SECRET,
       },
     },
     session: {
       cookieName: 'session',
       secure: process.env.NODE_ENV === 'production',
     },
   });
   ```

3. **Implement Signup API** (see `contracts/auth.openapi.yml`):
   - `POST /api/auth/signup`
   - `POST /api/auth/signin`
   - `GET /api/auth/session`

4. **Build Signup Questionnaire UI** (`frontend/src/components/profile/Questionnaire.tsx`):
   - Form with sections: Software Stack, Hardware, Learning Goals
   - Use React Hook Form for state management
   - Implement accessibility (ARIA labels, keyboard navigation)
   - Optional fields skippable without blocking

5. **Feature Flag Check**:
   ```typescript
   if (process.env.FEATURE_QUESTIONNAIRE_ENABLED !== 'true') {
     // Skip questionnaire, redirect to dashboard
   }
   ```

6. **Test**:
   ```bash
   npm test -- tests/auth.test.ts
   npm run test:e2e -- tests/e2e/signup.spec.ts
   ```

**Exit Criteria**:
- ✅ New users can sign up with email/password
- ✅ Questionnaire page renders with all fields
- ✅ Draft responses saved (even if incomplete)

---

### M2: Profile & Consent (Week 4)

**Goal**: Profile updates persist and privacy controls work

**Steps**:

1. **Implement Profile API** (see `contracts/profiles.openapi.yml`):
   - `POST /api/profile` (create from questionnaire)
   - `GET /api/profile` (retrieve current user profile)
   - `PATCH /api/profile` (update existing profile)

2. **Build Profile Settings Page** (`frontend/src/pages/settings/profile.tsx`):
   - Display current profile data
   - Edit form for software stack, hardware, learning goals
   - Privacy toggles: `personalizationOptIn`, `analyticsOptIn`

3. **Validation Logic** (`backend/src/lib/validation/profile-schema.ts`):
   ```typescript
   import { z } from 'zod';
   
   export const profileSchema = z.object({
     softwareStack: z.object({
       languages: z.array(z.string()).min(1, 'At least one language required'),
       frameworks: z.array(z.string()),
       tools: z.array(z.string()),
     }),
     hardwareEnvironment: z.object({
       os: z.enum(['Windows', 'macOS', 'Linux', 'Other']),
       deviceType: z.enum(['desktop', 'laptop', 'tablet', 'mobile']),
       gpuAvailable: z.boolean().optional(),
     }),
     learningGoals: z.object({
       skillLevel: z.enum(['beginner', 'intermediate', 'advanced', 'expert']),
       interests: z.array(z.string()),
       projectTypes: z.array(z.string()),
     }),
   });
   ```

4. **Test**:
   ```bash
   npm test -- tests/profile.test.ts
   npm run test:e2e -- tests/e2e/profile-settings.spec.ts
   ```

**Exit Criteria**:
- ✅ Profile updates persist in database
- ✅ Privacy toggles work (opt-out honored)
- ✅ Validation prevents invalid data

---

### M3: Content Catalog (Weeks 5-6)

**Goal**: Admin can create content; users can view catalog

**Steps**:

1. **Build Admin Content CRUD** (`backend/src/modules/content/admin.controller.ts`):
   - `POST /api/admin/content` (create)
   - `PATCH /api/admin/content/:id` (update)
   - `DELETE /api/admin/content/:id` (soft delete)

2. **Implement Content List API** (see `contracts/content.openapi.yml`):
   - `GET /api/content` (paginated, filterable)

3. **Build Content Catalog UI** (`frontend/src/pages/content/index.tsx`):
   - Grid/list view of content items
   - Filtering by category, difficulty
   - Search functionality

4. **Seed Script** (`backend/prisma/seed.ts`):
   ```typescript
   await prisma.contentItem.createMany({
     data: [
       {
         title: 'Python Data Science Essentials',
         description: 'Learn NumPy, Pandas, and Matplotlib...',
         contentUrl: 'https://example.com/python-ds',
         tags: {
           technologies: ['Python', 'NumPy', 'Pandas'],
           difficulty: 'intermediate',
           category: 'data-science',
         },
       },
       // ...more content items
     ],
   });
   ```

5. **Test**:
   ```bash
   npm test -- tests/content.test.ts
   npm run test:e2e -- tests/e2e/content-catalog.spec.ts
   ```

**Exit Criteria**:
- ✅ Admin can create content with tags
- ✅ Users can view and filter content catalog

---

### M4: Personalization Engine (Weeks 7-8)

**Goal**: Users receive personalized recommendations

**Steps**:

1. **Implement Recommendation Scoring** (`backend/src/modules/recommendations/scoring.service.ts`):
   ```typescript
   export function calculateRelevanceScore(
     profile: DeveloperProfile,
     content: ContentItem
   ): number {
     const languageMatch = calculateLanguageMatch(profile.softwareStack.languages, content.tags.technologies);
     const frameworkMatch = calculateFrameworkMatch(profile.softwareStack.frameworks, content.tags.technologies);
     const difficultyMatch = calculateDifficultyMatch(profile.learningGoals.skillLevel, content.tags.difficulty);
     const hardwareMatch = calculateHardwareMatch(profile.hardwareEnvironment.os, content.tags);
     
     return (
       0.4 * languageMatch +
       0.3 * frameworkMatch +
       0.2 * difficultyMatch +
       0.1 * hardwareMatch
     );
   }
   ```

2. **Implement Recommendations API** (see `contracts/content.openapi.yml`):
   - `GET /api/recommendations` (top N recommendations)
   - Pre-compute scores for top 100 content items per profile
   - Cache in Redis with 1-hour TTL

3. **Build Recommendations UI** (`frontend/src/components/recommendations/RecommendationList.tsx`):
   - Display ranked recommendations
   - Show "Why this?" explanation (reason field)
   - Feedback button ("Was this helpful?")

4. **Invalidation Logic** (profile updates):
   ```typescript
   // On profile update
   await redis.del(`recommendations:${userId}`);
   await recomputeScores(userId);
   ```

5. **Test**:
   ```bash
   npm test -- tests/recommendations.test.ts
   npm run test:e2e -- tests/e2e/recommendations.spec.ts
   ```

**Exit Criteria**:
- ✅ Users see personalized recommendations based on profile
- ✅ Recommendations include "Why this?" explanation
- ✅ Fallback to popularity ranking when profile incomplete

---

### M5-M7: Privacy, Observability, Hardening

See milestone definitions in user input for detailed exit criteria.

## Testing Strategy

### Unit Tests (Vitest)

```bash
npm test
```

### Integration Tests

```bash
npm run test:integration
```

### E2E Tests (Playwright)

```bash
npm run test:e2e
```

### Accessibility Tests

```bash
npm run test:a11y
```

## Troubleshooting

### Database Connection Issues

```bash
# Check PostgreSQL status
pg_isready -h localhost -p 5432

# Reset database
npx prisma migrate reset
```

### Better Auth Session Issues

- Clear cookies in browser
- Verify `AUTH_SECRET` is set in `.env`
- Check session expiry in Better Auth config

### Recommendation Scores Not Updating

- Verify Redis connection
- Check cache invalidation logic in profile update endpoint
- Manually clear Redis: `redis-cli FLUSHALL`

## Next Steps

1. Complete M0 setup ✅
2. Implement M1 (Auth & Signup Shell)
3. Implement M2 (Profile & Consent)
4. Implement M3 (Content Catalog)
5. Implement M4 (Personalization Engine)
6. Implement M5-M7 (Privacy, Observability, Hardening)
7. Run `/sp.tasks` to generate task breakdown

## Resources

- [Better Auth Documentation](https://better-auth.com/docs)
- [Prisma Documentation](https://www.prisma.io/docs)
- [WCAG AA Guidelines](https://www.w3.org/WAI/WCAG21/quickref/)
- [Constitution](../../.specify/memory/constitution.md)
- [Spec](spec.md)
- [Research](research.md)
- [Data Model](data-model.md)

## Status

✅ Phase 1 Quickstart Complete — Ready for Agent Context Update
