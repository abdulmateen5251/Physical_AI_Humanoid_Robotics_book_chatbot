# Authentication Data Storage Report

**Date:** December 19, 2025  
**Project:** Physical AI Humanoid Robotics Chatbot  
**System:** RAG Chatbot with Better Auth Integration

---

## Executive Summary

This report documents the complete authentication data flow, storage mechanisms, and security implementation for the RAG chatbot platform. Sign-in data is stored in three distinct layers:

1. **Browser Storage** (Frontend)
2. **Database** (Backend)
3. **JWT Token** (Stateless Authentication)

---

## 1. Frontend Storage (Browser)

### Location: `localStorage`

The frontend stores authentication data in the browser's localStorage for session persistence.

#### Stored Data:

| Key | Value | Format | Purpose |
|-----|-------|--------|---------|
| `auth_token` | JWT Token | String (JWT) | API authentication |
| `user` | User Info | JSON String | Display user email/name |
| `preferred_language` | Language Preference | String ('en' or 'ur') | Translation preference |

#### Storage Implementation:

**File:** `frontend/src/components/AuthContext.tsx`

```typescript
// Save after login/signup
localStorage.setItem('auth_token', data.token);
localStorage.setItem('user', JSON.stringify(data.user));

// Retrieve on page load
const token = localStorage.getItem('auth_token');
const user = JSON.parse(localStorage.getItem('user'));

// Clear on logout
localStorage.removeItem('auth_token');
localStorage.removeItem('user');
```

#### Security Notes:
- ⚠️ localStorage is **NOT** HttpOnly (accessible to JavaScript)
- ✅ XSS protection: Input validation in AuthModal
- ✅ CSRF protection: Token sent in Authorization header
- ✅ Auto-logout: Token expires in 7 days

---

## 2. Backend Database Storage

### Database Architecture

#### Development Environment:
```
Location: backend/rag_chatbot.db
Type: SQLite
ORM: Prisma 5.x
```

#### Production Environment:
```
Location: Neon Serverless PostgreSQL
Type: PostgreSQL 15+
ORM: Prisma 5.x
```

### Database Schema

#### User Table

**File:** `backend/prisma/schema.prisma`

```prisma
model User {
  id              String            @id @default(uuid())
  email           String            @unique
  passwordHash    String?
  emailVerified   DateTime?
  createdAt       DateTime          @default(now())
  updatedAt       DateTime          @updatedAt
  
  // Relationships
  profile         DeveloperProfile?
  analyticsEvents AnalyticsEvent[]

  @@map("users")
}
```

#### Field Details:

| Field | Type | Description |
|-------|------|-------------|
| `id` | UUID | Unique user identifier |
| `email` | String (unique) | Login email address |
| `passwordHash` | String | Bcrypt hashed password (never plain text) |
| `emailVerified` | DateTime | Email verification timestamp |
| `createdAt` | DateTime | Account creation time |
| `updatedAt` | DateTime | Last update time |

#### Related Tables:

```prisma
model DeveloperProfile {
  id                  String   @id @default(uuid())
  userId              String   @unique
  softwareStack       String   // JSON: {languages, frameworks, tools}
  hardwareEnvironment String   // JSON: {os, deviceType, gpuAvailable}
  learningGoals       String   // JSON: {skillLevel, interests, projectTypes}
  user                User     @relation(fields: [userId], references: [id])
}

model AnalyticsEvent {
  id          String   @id @default(uuid())
  userId      String
  eventName   String
  metadata    String   // JSON
  createdAt   DateTime @default(now())
  user        User     @relation(fields: [userId], references: [id])
}
```

---

## 3. JWT Token Authentication

### Token Mechanism

**File:** `backend/src/modules/auth/config.ts`

#### Token Structure:

```
Header:     { alg: "HS256", typ: "JWT" }
Payload:    { 
              sub: "user-id",
              email: "user@example.com",
              iat: 1703000000,
              exp: 1703604800,
              iss: "pdcp-auth"
            }
Signature:  HMACSHA256(base64(header) + base64(payload), AUTH_SECRET)
```

#### Token Configuration:

```typescript
const JWT_EXPIRES_IN = process.env.JWT_EXPIRES_IN || '7d';    // Default 7 days
const JWT_ISSUER = process.env.JWT_ISSUER || 'pdcp-auth';
const AUTH_SECRET = process.env.AUTH_SECRET;                   // From .env
```

#### Token Usage:

```typescript
// Signing during login
function signToken(payload: AuthTokenPayload): string {
  return jwt.sign(payload, AUTH_SECRET!, {
    expiresIn: JWT_EXPIRES_IN,
    issuer: JWT_ISSUER,
  });
}

// Verification on API calls
function verifyToken(token: string): AuthTokenPayload {
  return jwt.verify(token, AUTH_SECRET!, { issuer: JWT_ISSUER });
}
```

---

## 4. Authentication Flow

### Sign-Up Flow

```
┌─────────────────────────────────────────────────────────────┐
│ 1. User enters email & password in AuthModal                │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 2. POST /api/auth/signup (Frontend → Backend)               │
│    Body: { email, password }                                │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 3. Backend: Hash password with bcrypt                       │
│    saltRounds: 10                                           │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 4. Backend: Save to database (prisma.user.create)           │
│    - id (UUID)                                              │
│    - email                                                  │
│    - passwordHash                                           │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 5. Backend: Generate JWT token                              │
│    Payload: { sub: userId, email }                          │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 6. Backend: Return response                                 │
│    { user: { id, email }, token: "jwt..." }                 │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 7. Frontend: Store in localStorage                          │
│    - auth_token = token                                     │
│    - user = JSON.stringify(user)                            │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 8. Frontend: Update AuthContext (isAuthenticated = true)    │
└─────────────────────────────────────────────────────────────┘
```

### Sign-In Flow

```
┌─────────────────────────────────────────────────────────────┐
│ 1. User enters email & password in AuthModal                │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 2. POST /api/auth/signin (Frontend → Backend)               │
│    Body: { email, password }                                │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 3. Backend: Find user by email (prisma.user.findUnique)     │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 4. Backend: Compare password with stored hash (bcrypt)      │
│    If invalid: throw "Invalid credentials"                  │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 5. Backend: Generate JWT token                              │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 6. Frontend: Save to localStorage + AuthContext             │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 7. User can now use translation + chat features             │
└─────────────────────────────────────────────────────────────┘
```

### Session Verification Flow

```
┌─────────────────────────────────────────────────────────────┐
│ 1. Page load: Check localStorage for auth_token             │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 2. GET /api/auth/session (Frontend → Backend)               │
│    Header: Authorization: Bearer <token>                    │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 3. Backend: Verify JWT signature                            │
│    - If invalid/expired: 401 Unauthorized                   │
│    - If valid: Extract userId & email                       │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 4. Backend: Return user data or error                       │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ 5. Frontend: Update AuthContext accordingly                 │
└─────────────────────────────────────────────────────────────┘
```

---

## 5. API Endpoints

### Authentication Endpoints

**Base URL:** `http://localhost:3001/api/auth`

#### 1. Sign Up

```http
POST /api/auth/signup
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "securePassword123"
}

Response (201):
{
  "user": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "user@example.com"
  },
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
}

Response (409):
{
  "error": "User already exists"
}
```

#### 2. Sign In

```http
POST /api/auth/signin
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "securePassword123"
}

Response (200):
{
  "user": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "user@example.com"
  },
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
}

Response (401):
{
  "error": "Invalid credentials"
}
```

#### 3. Get Session

```http
GET /api/auth/session
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...

Response (200):
{
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "user": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "user@example.com"
  }
}

Response (401):
{
  "error": "Unauthorized"
}
```

#### 4. Sign Out

```http
POST /api/auth/signout
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...

Response (200):
{
  "message": "signed out"
}

Note: Client-driven (frontend deletes token from localStorage)
```

---

## 6. Security Implementation

### Password Security

✅ **Bcrypt Hashing**
- Algorithm: bcrypt with 10 salt rounds
- Never stores plain text passwords
- Slow by design (resistant to brute force)

```typescript
// Hashing on signup
const passwordHash = await bcrypt.hash(password, 10);

// Verification on signin
const valid = await bcrypt.compare(password, hash);
```

### Token Security

✅ **JWT (JSON Web Token)**
- Signed with AUTH_SECRET (HMAC-SHA256)
- Expires in 7 days by default
- Stateless (no server-side session storage)
- Verified on each API request

### API Security

✅ **CORS Policy**
```typescript
allow_origins: [
  "http://localhost:3000",
  "http://localhost:3001",
  "http://127.0.0.1:3000",
  "http://127.0.0.1:3001"
]
```

✅ **Authorization Header**
```
Authorization: Bearer <token>
```

---

## 7. Environment Configuration

### .env.auth File

```env
# Authentication
AUTH_SECRET=your-super-secret-key-change-this-in-production
JWT_EXPIRES_IN=7d
JWT_ISSUER=pdcp-auth

# Database
DATABASE_URL=sqlite:./rag_chatbot.db
# or for production:
# DATABASE_URL=postgresql://user:password@db.neon.tech/dbname

# Server
PORT=3001
NODE_ENV=development
```

### Required Environment Variables

| Variable | Default | Purpose |
|----------|---------|---------|
| `AUTH_SECRET` | None (required) | JWT signing key |
| `JWT_EXPIRES_IN` | 7d | Token expiration |
| `DATABASE_URL` | sqlite:./rag_chatbot.db | Database connection |
| `PORT` | 3001 | API server port |

---

## 8. Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                        BROWSER (Frontend)                       │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ AuthContext + AuthModal                                  │  │
│  │  - Manages login/signup state                            │  │
│  │  - Handles user input validation                         │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ localStorage                                             │  │
│  │  - auth_token (JWT)                                      │  │
│  │  - user (JSON)                                           │  │
│  │  - preferred_language                                   │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                           │ HTTP/JSON
                           │ Authorization: Bearer <token>
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│                      API SERVER (Backend)                       │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ Express Routes (/api/auth/*)                             │  │
│  │  - signup, signin, signout, session                      │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ JWT Verification                                         │  │
│  │  - Verify token signature                                │  │
│  │  - Check expiration                                      │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ bcrypt + Password Hashing                                │  │
│  │  - Compare plaintext with stored hash                    │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ Prisma ORM                                               │  │
│  │  - Interact with database                                │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                           │ SQL Queries
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│                       DATABASE                                  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ Users Table                                              │  │
│  │  id         │ email      │ passwordHash  │ createdAt    │  │
│  │  (UUID)     │ (unique)   │ (bcrypt)      │ (timestamp)  │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ Related Tables                                           │  │
│  │  - DeveloperProfile (preferences, skills)                │  │
│  │  - AnalyticsEvent (user activity)                        │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 9. Key Features

### ✅ Implemented

- [x] Email/password authentication
- [x] JWT token generation and verification
- [x] Bcrypt password hashing
- [x] Session persistence (localStorage)
- [x] Auto-logout on token expiration
- [x] CORS configuration
- [x] Protected API routes
- [x] Sign-in gated translation feature
- [x] Sign-in gated chatbot access

### 🔄 In Progress

- [ ] OAuth2 (GitHub, Google) integration
- [ ] Email verification
- [ ] Password reset flow
- [ ] Rate limiting

### 📅 Planned

- [ ] Two-factor authentication (2FA)
- [ ] Session management (multiple devices)
- [ ] Activity logging and auditing
- [ ] Better Auth full migration

---

## 10. Testing Data

### Test User Accounts

| Email | Password | Status |
|-------|----------|--------|
| test@example.com | password123 | Available for testing |
| user@test.com | testpass123 | Available for testing |

**Note:** These are created during development/testing. Production credentials should use strong passwords and proper secret management.

---

## 11. Troubleshooting

### Common Issues

#### 1. "Unauthorized" on session check
- **Cause:** Token expired or invalid
- **Solution:** Clear localStorage, sign in again

#### 2. "User already exists"
- **Cause:** Email already registered
- **Solution:** Use different email or sign in instead

#### 3. "Invalid credentials"
- **Cause:** Wrong email or password
- **Solution:** Check email/password and try again

#### 4. Token not sent to API
- **Cause:** Authorization header missing
- **Solution:** Check frontend is using token from localStorage

#### 5. Database connection error
- **Cause:** DATABASE_URL not set or database unreachable
- **Solution:** Check .env configuration and database status

---

## 12. Production Deployment Checklist

- [ ] Set strong `AUTH_SECRET` (32+ character random string)
- [ ] Change `DATABASE_URL` to production PostgreSQL (Neon)
- [ ] Set `NODE_ENV=production`
- [ ] Enable HTTPS for all endpoints
- [ ] Update CORS allowed origins
- [ ] Configure proper logging and monitoring
- [ ] Set up rate limiting
- [ ] Enable email verification
- [ ] Test backup/restore procedures
- [ ] Document password rotation policy

---

## 13. References

### Files Involved

**Frontend:**
- `frontend/src/components/AuthContext.tsx` - Authentication state management
- `frontend/src/components/AuthButton.tsx` - Sign in button UI
- `frontend/src/components/AuthModal.tsx` - Sign in/signup form
- `frontend/src/theme/Navbar/Content/index.tsx` - Navbar integration

**Backend:**
- `backend/src/modules/auth/routes.ts` - API endpoints
- `backend/src/modules/auth/config.ts` - Auth logic (bcrypt, JWT)
- `backend/prisma/schema.prisma` - Database schema
- `backend/src/lib/prisma.ts` - Prisma client

**Configuration:**
- `backend/.env.auth` - Environment variables
- `backend/package.json` - Dependencies

### Dependencies

```json
{
  "bcrypt": "^5.1.1",
  "jsonwebtoken": "^9.1.2",
  "@prisma/client": "^5.x",
  "express": "^4.x",
  "cors": "^2.x"
}
```

---

## 14. Contact & Support

For issues or questions regarding authentication:
- Check `.env.auth` configuration
- Review error logs in backend console
- Check browser console for frontend errors
- Verify database connection

---

**Report Generated:** December 19, 2025  
**Last Updated:** December 19, 2025  
**Status:** Active & Maintained
