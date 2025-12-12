# Physical AI & Humanoid Robotics Learning Platform

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Status: Production Ready](https://img.shields.io/badge/Status-Production%20Ready-brightgreen.svg)]()
[![Built with Docusaurus](https://img.shields.io/badge/Built%20with-Docusaurus-white.svg)](https://docusaurus.io/)
[![Powered by FastAPI](https://img.shields.io/badge/Backend-FastAPI-009688.svg)](https://fastapi.tiangolo.com/)
[![Vector DB Qdrant](https://img.shields.io/badge/Vector%20DB-Qdrant-b93b5e.svg)](https://qdrant.tech/)

A **free, open-source documentation and learning platform** for Physical AI and Humanoid Robotics engineering, now featuring an **Integrated RAG Chatbot** for interactive learning.

## 📚 What This Is

A comprehensive learning platform with **20+ chapters** covering 4 robotics modules, enhanced with an AI assistant that answers questions strictly from the book's content.

```
✅ Module 1: ROS 2 Fundamentals
✅ Module 2: Digital Twin & Gazebo Simulation  
✅ Module 3: NVIDIA Isaac Sim
✅ Module 4: Vision-Language-Action (VLA) Systems
```

## 🤖 AI Chatbot Features

- **Context-Aware Q&A**: Ask questions about any topic in the book
- **Grounded Answers**: Responses are strictly based on the documentation (RAG)
- **Citations**: Every answer includes links to specific chapters and sections
- **Selection Mode**: Highlight text to ask questions about that specific section
- **Dark Mode Support**: Fully integrated UI that adapts to your theme

## 🚀 Quick Start (Docker - Recommended)

### Prerequisites
- Docker & Docker Compose
- OpenAI API Key
- Qdrant Cloud URL & API Key (Free Tier available)

### Steps

```bash
# 1. Clone repository
git clone https://github.com/abdulmateen5251/Physical_AI_Humanoid_Robotics_book.git
cd Physical_AI_Humanoid_Robotics_book

# 2. Configure environment
cp .env.example .env
# Edit .env with your API keys

# 3. Run with Docker Compose
docker-compose up --build -d

# 4. Access the platform
# Frontend: http://localhost:3000
# Backend API: http://localhost:8090/docs
```

### Alternative: Frontend Only (Static Site)

```bash
# For documentation-only mode (no chatbot)
cd frontend
npm install
npm start
# Open browser to http://localhost:3001
```

## 🛠️ Tech Stack

### Frontend
- **Docusaurus 3** - Static site generator
- **React 18** - UI components
- **TypeScript** - Type safety
- **Custom Chat Widget** - React-based floating chat interface

### Backend
- **FastAPI** - High-performance Python API
- **OpenAI** - LLM for chat responses
- **Qdrant** - Vector database for semantic search
- **SQLite/PostgreSQL** - Chat history and session storage
- **Docker** - Containerization

## 📖 Features

- ✅ **AI-Powered Chatbot** - Ask questions, get instant answers from the book
- ✅ **RAG Architecture** - Grounded responses with citations
- ✅ **Selection Mode** - Query specific highlighted text
- ✅ **Full-text search** across all content
- ✅ **Dark mode** support with modern UI
- ✅ **Responsive design** (mobile-friendly)
- ✅ **Fast performance** (static HTML + API)
## 📁 Project Structure

```
Physical_AI_Humanoid_Robotics_book/
├── backend/                     # FastAPI RAG Server
│   ├── app/                     # Application code
│   │   ├── api/                 # API endpoints
│   │   ├── models/              # Database models
│   │   └── services/            # Business logic
│   ├── scripts/                 # Ingestion scripts
│   └── Dockerfile               # Backend container
│
├── frontend/                    # Docusaurus site
│   ├── docs/                    # Course content
│   │   ├── module-01-ros2/      # Module 1
│   │   ├── module-02-gazebo/    # Module 2
│   │   ├── module-03-isaac/     # Module 3
│   │   └── module-04-vla/       # Module 4
│   ├── src/
│   │   ├── components/          # ChatWidget component
│   │   └── theme/               # Custom theme
│   └── Dockerfile               # Frontend container
│
├── docker-compose.yml           # Orchestration
├── .env.example                 # Environment template
└── specs/                       # Original specifications
``` └── docusaurus.config.js     # Configuration
│
└── specs/                       # Original specifications
```

## 🌐 Deploy

### Vercel (Recommended - Free)
```bash
# Push to GitHub, connect to Vercel
# Auto-deploys on every push
```

### Netlify
## 📝 Edit Content

Edit markdown files in `frontend/docs/`:

```bash
# Example: Add new chapter
frontend/docs/module-01-ros2/03-new-chapter.md

# Update sidebar in frontend/sidebars.js

# Ingest into chatbot knowledge base
docker-compose exec backend python scripts/ingest_book.py
```
Edit markdown files in `frontend/docs/`:

```bash
# Example: Add new chapter
frontend/docs/module-01-ros2/03-new-chapter.md

# Update sidebar in frontend/sidebars.js

# Changes appear instantly with npm start
```

## 🤝 Contribute

1. Fork repository
2. Create branch: `git checkout -b feat/new-content`
3. Add content to `frontend/docs/`
4. Test: `npm run build`
5. Push: `git push origin feat/new-content`
6. Open Pull Request

## ⚡ Commands

### Docker Commands
```bash
docker-compose up --build -d    # Build and start all services
docker-compose down             # Stop all services
docker-compose logs -f          # View logs
docker-compose restart frontend # Restart frontend only
```

### Frontend Only (Development)
```bash
cd frontend
npm start         # Dev server (http://localhost:3001)
npm run build     # Production build
npm run serve     # Serve production build
npm run clean     # Clear build cache
```

### Backend Commands
```bash
docker-compose exec backend python scripts/ingest_book.py  # Ingest content
```*Edit this page**: See "Edit this page" link on every page

## ⚡ Commands

```bash
cd frontend
---

**Status**: ✅ **Production Ready** (with RAG Chatbot)  
**Last Updated**: December 13, 2025  
**Maintainer**: Abdul Mateen (@abdulmateen5251)
```

## 🎯 Next Steps

1. **Read**: Explore the course modules
2. **Contribute**: Add more chapters or fix issues
3. **Share**: Deploy and share with others
4. **Learn**: Use as learning resource

---

**Status**: ✅ **Production Ready**  
**Last Updated**: December 12, 2025  
**Maintainer**: Abdul Mateen (@abdulmateen5251)


