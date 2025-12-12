# Docker Setup Guide

This guide explains how to run the Physical AI Humanoid Robotics Book RAG Chatbot using Docker and Docker Compose.

## Prerequisites

- Docker (version 20.10 or higher)
- Docker Compose (version 2.0 or higher)
- OpenAI API key
- Qdrant Cloud account (Free tier available)
- Neon Postgres database (Serverless)

## Quick Start

### 1. Configure Environment Variables

Copy the example environment file and fill in your credentials:

```bash
cp .env.example .env
```

Edit `.env` and add your actual credentials:

```env
OPENAI_API_KEY=sk-proj-your_actual_key_here
QDRANT_URL=https://your-actual-cluster.qdrant.io
QDRANT_API_KEY=ey_your_actual_key_here
DATABASE_URL=postgresql://user:password@host.neon.tech/database?sslmode=require
```

### 2. Build and Start Services

Build and start all services in detached mode:

```bash
docker-compose up -d --build
```

This will:
- Build the backend FastAPI service
- Build the frontend Docusaurus service
- Start both services with proper networking
- Set up health checks and auto-restart policies

### 3. Verify Services

Check that all services are running:

```bash
docker-compose ps
```

You should see both `rag-chatbot-backend` and `rag-chatbot-frontend` with status "Up".

### 4. Access the Application

- **Frontend**: http://localhost:3000
- **Backend API**: http://localhost:8090
- **API Docs**: http://localhost:8090/docs
- **Health Check**: http://localhost:8090/health

## Docker Commands

### View Logs

View logs from all services:
```bash
docker-compose logs -f
```

View logs from a specific service:
```bash
docker-compose logs -f backend
docker-compose logs -f frontend
```

### Stop Services

Stop all services (containers remain):
```bash
docker-compose stop
```

### Start Stopped Services

Start previously stopped services:
```bash
docker-compose start
```

### Restart Services

Restart all services:
```bash
docker-compose restart
```

Restart a specific service:
```bash
docker-compose restart backend
```

### Stop and Remove Containers

Stop and remove all containers:
```bash
docker-compose down
```

Stop and remove containers, volumes, and networks:
```bash
docker-compose down -v
```

### Rebuild Services

Rebuild services after code changes:
```bash
docker-compose up -d --build
```

Rebuild a specific service:
```bash
docker-compose up -d --build backend
```

### Execute Commands in Containers

Run a command in the backend container:
```bash
docker-compose exec backend python scripts/ingest_book.py
```

Open a shell in the backend container:
```bash
docker-compose exec backend bash
```

Open a shell in the frontend container:
```bash
docker-compose exec frontend sh
```

## Initial Data Setup

After starting the services for the first time, you need to ingest the book content:

```bash
docker-compose exec backend python scripts/ingest_book.py
```

This will:
- Parse the book content from the `docs/` directory
- Create chunks with proper metadata
- Generate embeddings using OpenAI
- Upload to Qdrant vector database
- Initialize the Postgres database schema

## Development Mode

For development with hot-reload, you can modify the docker-compose.yml or create a `docker-compose.dev.yml`:

```yaml
version: '3.8'

services:
  backend:
    build:
      context: ./backend
    command: uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
    volumes:
      - ./backend:/app
    ports:
      - "8090:8000"
    environment:
      - ENVIRONMENT=development

  frontend:
    build:
      context: ./frontend
    command: npm start
    volumes:
      - ./frontend:/app
      - /app/node_modules
    ports:
      - "3000:3000"
```

Run development mode:
```bash
docker-compose -f docker-compose.dev.yml up
```

## Troubleshooting

### Service Won't Start

Check logs for errors:
```bash
docker-compose logs backend
docker-compose logs frontend
```

### Port Already in Use

If ports 3000 or 8090 are already in use, modify the port mapping in `docker-compose.yml`:

```yaml
ports:
  - "8091:8000"  # Change host port (left side)
```

### Environment Variables Not Loading

Ensure `.env` file exists in the root directory and contains valid values.

### Database Connection Issues

Verify your `DATABASE_URL` is correct and Neon database is accessible.

### Vector Store Connection Issues

Verify your `QDRANT_URL` and `QDRANT_API_KEY` are correct.

### Container Health Check Failing

Check if the service is responding:
```bash
docker-compose exec backend curl http://localhost:8000/health
docker-compose exec frontend wget -O- http://localhost:3000/
```

## Production Deployment

For production deployment:

1. **Update CORS settings** in `backend/app/main.py` to include your production domain
2. **Use production environment variables** with secure secrets management
3. **Set up reverse proxy** (nginx/traefik) for SSL/TLS termination
4. **Enable resource limits** in docker-compose.yml:

```yaml
services:
  backend:
    deploy:
      resources:
        limits:
          cpus: '1'
          memory: 1G
        reservations:
          cpus: '0.5'
          memory: 512M
```

5. **Use Docker secrets** for sensitive data instead of environment variables
6. **Set up logging aggregation** (ELK stack, Loki, etc.)
7. **Monitor with health checks** and alerting

## Clean Up

Remove all containers, volumes, and images:

```bash
docker-compose down -v
docker system prune -a
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Docker Network (rag-network)            │
│                                                             │
│  ┌──────────────────┐           ┌──────────────────┐      │
│  │   Frontend       │           │    Backend       │      │
│  │   (Docusaurus)   │──────────▶│    (FastAPI)     │      │
│  │   Port: 3000     │           │    Port: 8000    │      │
│  └──────────────────┘           └──────────────────┘      │
│                                           │                 │
└───────────────────────────────────────────┼─────────────────┘
                                            │
                        ┌───────────────────┼───────────────┐
                        │                   │               │
                        ▼                   ▼               ▼
                 ┌──────────┐       ┌──────────┐   ┌──────────┐
                 │  OpenAI  │       │  Qdrant  │   │   Neon   │
                 │   API    │       │  Cloud   │   │ Postgres │
                 └──────────┘       └──────────┘   └──────────┘
```

## Support

For issues or questions:
- Check the main [README.md](./README.md)
- Review [SETUP_GUIDE.md](./SETUP_GUIDE.md)
- Check [RAG_SETUP_GUIDE.md](./RAG_SETUP_GUIDE.md)
