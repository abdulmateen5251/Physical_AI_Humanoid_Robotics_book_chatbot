# Docker Error - Quick Fix

## Problem
The Docker container is failing because the required environment variables are not set. The error message shows:
```
sqlalchemy.exc.ArgumentError: Could not parse SQLAlchemy URL from string ''
```

This means the `DATABASE_URL` (and possibly other environment variables) are empty.

## Solution

### Step 1: Create the .env file
Copy the example file to create your .env:

```powershell
# In PowerShell
Copy-Item .env.example .env
```

Or manually create a `.env` file in the project root directory.

### Step 2: Fill in your credentials
Edit the `.env` file with your actual credentials:

```env
# OPENAI CONFIGURATION
OPENAI_API_KEY=sk-proj-your_actual_openai_api_key_here
OPENAI_MODEL=gpt-4o-mini
OPENAI_EMBEDDING_MODEL=text-embedding-3-small

# QDRANT CLOUD CONFIGURATION
QDRANT_URL=https://your-actual-qdrant-instance.qdrant.io
QDRANT_API_KEY=your_actual_qdrant_api_key_here
QDRANT_COLLECTION=book_chunks

# NEON POSTGRES CONFIGURATION
DATABASE_URL=postgresql://user:password@host.neon.tech/database?sslmode=require
```

### Step 3: Rebuild and restart Docker containers

```powershell
# Stop and remove existing containers
docker-compose down

# Rebuild with the new environment variables
docker-compose up --build -d

# Check the logs
docker-compose logs -f backend
```

## Where to get your credentials:

1. **OpenAI API Key**: https://platform.openai.com/api-keys
2. **Qdrant Cloud**: https://qdrant.io (Free Tier available)
3. **Neon Postgres**: https://neon.tech (Free Tier available)

## Verify it's working

Once the containers are running, check:
```powershell
# Health check
curl http://localhost:8090/health

# Should return: {"status":"healthy"}
```

## What was fixed:

1. ✅ Added `env_file: .env` to docker-compose.yml
2. ✅ Added validation in config.py to show clear error messages
3. ✅ Made environment variables non-required with defaults to allow startup
4. ✅ Added helpful error messages when required vars are missing
