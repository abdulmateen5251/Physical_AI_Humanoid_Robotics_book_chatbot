# Data Model: Developer Personalization System

**Feature**: 002-developer-personalization  
**Date**: 2025-12-16  
**Status**: Complete  
**Prerequisites**: [research.md](research.md) - R5: Database Schema Design

## Overview

This document defines the data entities, relationships, validation rules, and state transitions for the developer personalization system. The model uses PostgreSQL with Prisma ORM, leveraging JSONB for flexible schema while maintaining relational integrity.

## Entities

### User

Represents an authenticated user account (managed by Better Auth).

**Attributes**:
- `id` (UUID, Primary Key): Unique identifier
- `email` (String, Unique, Required): User email address
- `emailVerified` (DateTime, Nullable): Email verification timestamp
- `createdAt` (DateTime, Required): Account creation timestamp
- `updatedAt` (DateTime, Required): Last update timestamp

**Relationships**:
- Has one `DeveloperProfile` (optional, created during/after signup)

**Validation Rules**:
- Email must be valid format (RFC 5322)
- Email must be unique across all users

**State Transitions**:
1. Created → Email sent for verification
2. EmailVerified set → Full account access enabled

---

### DeveloperProfile

Represents a user's technical context, learning goals, and privacy preferences.

**Attributes**:
- `id` (UUID, Primary Key): Unique identifier
- `userId` (UUID, Foreign Key → User, Required, Unique): Associated user
- `softwareStack` (JSONB, Required): Programming languages, frameworks, tools
  - Schema: `{ languages: string[], frameworks: string[], tools: string[] }`
  - Example: `{ "languages": ["Python", "TypeScript"], "frameworks": ["React", "FastAPI"], "tools": ["Docker", "Git"] }`
- `hardwareEnvironment` (JSONB, Required): OS, device type, hardware capabilities
  - Schema: `{ os: string, deviceType: string, gpuAvailable: boolean }`
  - Example: `{ "os": "Linux", "deviceType": "laptop", "gpuAvailable": true }`
- `learningGoals` (JSONB, Required): Skill level, interests, project types
  - Schema: `{ skillLevel: string, interests: string[], projectTypes: string[] }`
  - Example: `{ "skillLevel": "intermediate", "interests": ["machine-learning", "web-dev"], "projectTypes": ["personal", "open-source"] }`
- `personalizationOptIn` (Boolean, Required, Default: true): User consent for personalization
- `analyticsOptIn` (Boolean, Required, Default: false): User consent for analytics tracking
- `createdAt` (DateTime, Required): Profile creation timestamp
- `updatedAt` (DateTime, Required): Last update timestamp

**Relationships**:
- Belongs to one `User`
- Has many `RecommendationScore` entries

**Validation Rules**:
- `softwareStack.languages` must be non-empty array (at least one language required)
- `skillLevel` must be one of: `beginner`, `intermediate`, `advanced`, `expert`
- `os` must be one of: `Windows`, `macOS`, `Linux`, `Other`
- `deviceType` must be one of: `desktop`, `laptop`, `tablet`, `mobile`

**State Transitions**:
1. Created (during signup) → Initial questionnaire data saved
2. Updated (from profile settings) → `updatedAt` timestamp refreshed, recommendation cache invalidated

**Privacy Considerations**:
- Encrypt `softwareStack` and `hardwareEnvironment` if containing proprietary tool names (TBD: M7 threat modeling)
- Delete on user account deletion (Cascade)
- Export in JSON format for data portability (M5)

---

### ContentItem

Represents learning content (articles, tutorials, documentation) available for recommendation.

**Attributes**:
- `id` (UUID, Primary Key): Unique identifier
- `title` (String, Required): Content title
- `description` (String, Required): Content description
- `contentUrl` (String, Required): URL to content
- `tags` (JSONB, Required): Technologies, difficulty, category metadata
  - Schema: `{ technologies: string[], difficulty: string, category: string }`
  - Example: `{ "technologies": ["Python", "Django", "PostgreSQL"], "difficulty": "intermediate", "category": "web-development" }`
- `createdAt` (DateTime, Required): Content creation timestamp
- `updatedAt` (DateTime, Required): Last update timestamp

**Relationships**:
- Has many `RecommendationScore` entries

**Validation Rules**:
- `title` must be 10-200 characters
- `description` must be 50-1000 characters
- `contentUrl` must be valid URL format
- `tags.difficulty` must be one of: `beginner`, `intermediate`, `advanced`, `expert`
- `tags.technologies` must be non-empty array

**State Transitions**:
1. Created (by admin via content catalog) → Available for recommendations
2. Updated → Recommendation scores recalculated for affected profiles

**Admin Operations** (M3: Content Catalog):
- Create new content item
- Update existing content (tags, description, URL)
- Soft delete (archive) content no longer relevant

---

### StarterTemplate

Represents project boilerplate and scaffolding code.

**Attributes**:
- `id` (UUID, Primary Key): Unique identifier
- `name` (String, Required): Template name
- `description` (String, Required): Template description
- `supportedTechnologies` (JSONB, Required): Languages, frameworks, OS compatibility
  - Schema: `{ languages: string[], frameworks: string[], os: string[] }`
  - Example: `{ "languages": ["TypeScript"], "frameworks": ["Next.js"], "os": ["Windows", "macOS", "Linux"] }`
- `templateFiles` (String, Required): URL or storage path to template archive (ZIP/Git repo)
- `createdAt` (DateTime, Required): Template creation timestamp
- `updatedAt` (DateTime, Required): Last update timestamp

**Relationships**:
- None (standalone entity, filtered by profile matching)

**Validation Rules**:
- `name` must be 5-100 characters
- `description` must be 50-500 characters
- `supportedTechnologies.languages` must be non-empty array
- `templateFiles` must be valid URL or storage path

**State Transitions**:
1. Created (by admin) → Available for filtering based on user profiles
2. Updated → Compatibility checks re-run

**Filtering Logic** (FR-009):
- Match profile `softwareStack.languages` against `supportedTechnologies.languages`
- Match profile `hardwareEnvironment.os` against `supportedTechnologies.os`
- Rank matches by number of overlapping technologies

---

### RecommendationScore

Represents computed relevance of content to a developer profile.

**Attributes**:
- `id` (UUID, Primary Key): Unique identifier
- `profileId` (UUID, Foreign Key → DeveloperProfile, Required): Associated profile
- `contentId` (UUID, Foreign Key → ContentItem, Required): Associated content
- `score` (Float, Required): Relevance score (0.0 - 1.0)
- `computedAt` (DateTime, Required): Score computation timestamp

**Relationships**:
- Belongs to one `DeveloperProfile`
- Belongs to one `ContentItem`

**Validation Rules**:
- `score` must be between 0.0 and 1.0 (inclusive)
- Unique constraint on (`profileId`, `contentId`) — one score per profile-content pair

**Indexes**:
- Composite index on (`profileId`, `score DESC`) for efficient "top N recommendations" queries

**State Transitions**:
1. Computed (when profile created/updated) → Stored in database + Redis cache
2. Invalidated (when profile updated or content tags changed) → Deleted and recomputed

**Caching Strategy** (R2: Recommendation Algorithm):
- Store top 100 scores per profile in Redis with TTL (1 hour)
- Invalidate cache on profile update
- Fallback to database query if cache miss

---

### AnalyticsEvent

Represents user interaction events for observability and metrics.

**Attributes**:
- `id` (UUID, Primary Key): Unique identifier
- `userId` (UUID, Foreign Key → User, Required): Associated user
- `eventType` (String, Required): Event category
  - Values: `profile_created`, `profile_updated`, `recommendation_viewed`, `template_selected`, `content_clicked`
- `eventData` (JSONB, Nullable): Event-specific payload
  - Example for `recommendation_viewed`: `{ "contentId": "...", "score": 0.85, "rank": 3 }`
- `timestamp` (DateTime, Required): Event occurrence timestamp
- `optedIn` (Boolean, Required): Whether user had analytics opt-in at event time

**Relationships**:
- Belongs to one `User`

**Validation Rules**:
- `eventType` must be from predefined list (enforced in application layer)
- Event only logged if `DeveloperProfile.analyticsOptIn == true` at time of event

**Retention Policy**:
- Delete events older than 90 days (configurable)
- Aggregate metrics before deletion for historical dashboards

**Privacy Enforcement** (R3: Privacy-Respecting Analytics):
- Middleware checks `user.profile.analyticsOptIn` before logging
- Events deleted on user account deletion (Cascade)
- No PII in `eventData` (content IDs and scores only)

---

## Relationships Diagram

```
User (1) ──────────────── (1) DeveloperProfile
                                │
                                │ (1)
                                ↓
                           (many) RecommendationScore (many)
                                ↓
                                (1)
                            ContentItem

User (1) ──────────────── (many) AnalyticsEvent

StarterTemplate (standalone, filtered by profile attributes)
```

## Prisma Schema Summary

```prisma
model User {
  id              String            @id @default(uuid())
  email           String            @unique
  emailVerified   DateTime?
  createdAt       DateTime          @default(now())
  updatedAt       DateTime          @updatedAt
  profile         DeveloperProfile?
  analyticsEvents AnalyticsEvent[]
}

model DeveloperProfile {
  id                    String   @id @default(uuid())
  userId                String   @unique
  user                  User     @relation(fields: [userId], references: [id], onDelete: Cascade)
  softwareStack         Json
  hardwareEnvironment   Json
  learningGoals         Json
  personalizationOptIn  Boolean  @default(true)
  analyticsOptIn        Boolean  @default(false)
  createdAt             DateTime @default(now())
  updatedAt             DateTime @updatedAt
  recommendationScores  RecommendationScore[]
}

model ContentItem {
  id                   String   @id @default(uuid())
  title                String
  description          String
  contentUrl           String
  tags                 Json
  createdAt            DateTime @default(now())
  updatedAt            DateTime @updatedAt
  recommendationScores RecommendationScore[]
}

model StarterTemplate {
  id                    String   @id @default(uuid())
  name                  String
  description           String
  supportedTechnologies Json
  templateFiles         String
  createdAt             DateTime @default(now())
  updatedAt             DateTime @updatedAt
}

model RecommendationScore {
  id         String            @id @default(uuid())
  profileId  String
  profile    DeveloperProfile  @relation(fields: [profileId], references: [id], onDelete: Cascade)
  contentId  String
  content    ContentItem       @relation(fields: [contentId], references: [id], onDelete: Cascade)
  score      Float
  computedAt DateTime          @default(now())
  
  @@unique([profileId, contentId])
  @@index([profileId, score(sort: Desc)])
}

model AnalyticsEvent {
  id        String   @id @default(uuid())
  userId    String
  user      User     @relation(fields: [userId], references: [id], onDelete: Cascade)
  eventType String
  eventData Json?
  timestamp DateTime @default(now())
  optedIn   Boolean
}
```

## Migration Strategy

1. **M0-M1**: Create `User` table (Better Auth integration)
2. **M2**: Add `DeveloperProfile` table with privacy preferences
3. **M3**: Add `ContentItem` and `StarterTemplate` tables
4. **M4**: Add `RecommendationScore` table with indexes
5. **M6**: Add `AnalyticsEvent` table

Each migration must include:
- Up migration (create tables, indexes)
- Down migration (rollback capability)
- Seed data for testing (at least 10 content items, 5 templates)

## Status

✅ Phase 1 Data Model Complete — Ready for API Contract Definition
