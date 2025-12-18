# Docker Compose Quick Start

## Services Overview

Your Docker Compose setup runs **3 services**:

1. **Auth Backend** (Node.js/TypeScript) - Port 3001
   - Better Auth authentication service
   - TypeScript with hot reload
   - Prisma ORM for database

2. **FastAPI Backend** (Python) - Port 8090
   - RAG chatbot API
   - OpenAI integration
   - Qdrant vector search

3. **Frontend** (Docusaurus) - Port 3000
   - Documentation website
   - Chat interface
   - Book content display

## Quick Start

### 1. Setup Environment Variables

```powershell
# Copy the example file
cp .env.example .env

# Edit .env with your credentials
notepad .env
```

Required credentials:
- `OPENAI_API_KEY` - Get from [OpenAI Platform](https://platform.openai.com/api-keys)
- `QDRANT_URL` and `QDRANT_API_KEY` - Get from [Qdrant Cloud](https://qdrant.io)
- `DATABASE_URL` - Get from [Neon](https://neon.tech)
- `BETTER_AUTH_SECRET` - Generate a random 32+ character string

### 2. Start All Services

**PowerShell:**
```powershell
.\start-docker.ps1
```

**Command Prompt:**
```cmd
start-docker.bat
```

**Or directly:**
```bash
docker-compose up --build
```

### 3. Access Services

- 🔐 **Auth Backend**: http://localhost:3001
- 🤖 **FastAPI Backend**: http://localhost:8090/docs (Swagger UI)
- 🌐 **Frontend**: http://localhost:3000

## Common Commands

### Start services (detached mode)
```bash
docker-compose up -d
```

### View logs
```bash
# All services
docker-compose logs -f

# Specific service
docker-compose logs -f auth-backend
docker-compose logs -f backend
docker-compose logs -f frontend
```

### Stop services
```bash
docker-compose down
```

### Rebuild specific service
```bash
docker-compose up --build auth-backend
docker-compose up --build backend
docker-compose up --build frontend
```

### Restart a service
```bash
docker-compose restart auth-backend
```

### Execute commands in a container
```bash
# Auth backend
docker-compose exec auth-backend npm run prisma:migrate
docker-compose exec auth-backend npm run db:seed

# FastAPI backend
docker-compose exec backend python scripts/ingest_book.py

# Frontend
docker-compose exec frontend npm run build
```

### Clean up everything
```bash
# Stop and remove containers, networks, volumes
docker-compose down -v

# Remove all images
docker-compose down --rmi all
```

## Development Workflow

### Hot Reload

All services support hot reload:
- **Auth Backend**: Edit files in `backend/src/` - TypeScript will recompile automatically
- **FastAPI Backend**: Edit files in `backend/app/` - Uvicorn will reload automatically  
- **Frontend**: Edit files in `frontend/src/` or `frontend/docs/` - Docusaurus will rebuild

### Database Migrations

```bash
# Run Prisma migrations for auth backend
docker-compose exec auth-backend npm run prisma:migrate

# Generate Prisma client
docker-compose exec auth-backend npm run prisma:generate

# Open Prisma Studio
docker-compose exec auth-backend npm run prisma:studio
```

### Ingest Book Content

```bash
# Run the ingestion script
docker-compose exec backend python scripts/ingest_book.py
```

## Troubleshooting

### Services won't start

1. Check if ports are already in use:
   ```powershell
   netstat -ano | findstr "3000 3001 8090"
   ```

2. Check Docker is running:
   ```powershell
   docker ps
   ```

3. View service logs:
   ```bash
   docker-compose logs -f
   ```

### Auth backend connection issues

- Verify `DATABASE_URL` in `.env` is correct
- Check `BETTER_AUTH_SECRET` is at least 32 characters
- Ensure Neon database is accessible

### FastAPI backend connection issues

- Verify `OPENAI_API_KEY` is valid
- Check `QDRANT_URL` and `QDRANT_API_KEY` are correct
- Ensure Qdrant collection exists

### Frontend can't connect to backends

- Check all services are healthy: `docker-compose ps`
- Verify CORS settings in backends
- Check browser console for errors

### Rebuild from scratch

```bash
# Stop everything
docker-compose down -v

# Remove all cached images
docker system prune -a

# Rebuild and start
docker-compose up --build
```

## Network Configuration

All services communicate on the `rag-network` Docker network:

- Frontend → Auth Backend: `http://auth-backend:3001`
- Frontend → FastAPI Backend: `http://backend:8000`
- FastAPI Backend → Auth Backend: `http://auth-backend:3001`

External access (from your host machine):
- Auth Backend: `http://localhost:3001`
- FastAPI Backend: `http://localhost:8090`
- Frontend: `http://localhost:3000`

## Volume Mounts

- `backend-cache`: Python package cache
- `auth-node-modules`: Node.js dependencies for auth backend
- Source code directories are mounted for hot reload

## Health Checks

All services include health checks:
- Services won't be marked as "healthy" until their health check passes
- Frontend waits for both backends to be healthy before starting
- Check status: `docker-compose ps`
