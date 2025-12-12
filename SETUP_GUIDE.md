# 📋 Setup Guide - Physical AI & Humanoid Robotics Learning Platform

**Last Updated**: December 7, 2025  
**Setup Time**: 5-10 minutes

---

## 🎯 Overview

This guide will help you set up and run the **Docusaurus documentation site** locally. The project is a static documentation platform without complex dependencies.

## ✅ Prerequisites

### Required
- **Node.js 18+** - [Download here](https://nodejs.org/)
- **npm 10+** - Comes with Node.js
- **Git** - [Download here](https://git-scm.com/downloads)

### Optional (Not Required for Frontend)
- Python 3.11+ - Only if you want to run backend
- Docker - Only for advanced setup

---

## 🚀 Quick Start (5 minutes)

### Step 1: Clone Repository

```bash
# Via HTTPS
git clone https://github.com/abdulmateen5251/Physical_AI_Humanoid_Robotics_book.git

# Or via SSH (if you have SSH keys configured)
git clone git@github.com:abdulmateen5251/Physical_AI_Humanoid_Robotics_book.git

# Navigate to project
cd Physical_AI_Humanoid_Robotics_book
```

### Step 2: Install Frontend Dependencies

```bash
cd frontend
npm install

# Wait for npm to download all packages (~2-3 minutes)
```

**Windows Users** (PowerShell):
```powershell
cd frontend
npm install
```

### Step 3: Start Development Server

```bash
npm start

# Expected output:
# ℹ ️  [docusaurus:start] Starting the development server...
# ✔ [docusaurus:server] Server started on http://localhost:3001
```

### Step 4: Open in Browser

Visit: **http://localhost:3001**

You should see the documentation site with all course modules!

---

## 📖 Using the Site

### Navigation
- **Sidebar**: Click module names to expand/collapse
- **Search**: Use search bar at top to find content
- **Dark Mode**: Toggle in top-right corner
- **Previous/Next**: Navigate between chapters

### Course Structure
1. **Module 1**: ROS 2 Fundamentals
2. **Module 2**: Digital Twin & Simulation (Gazebo)
3. **Module 3**: NVIDIA Isaac Sim
4. **Module 4**: Vision-Language-Action (VLA)

---

## 🏗️ Project Structure

```
Physical_AI_Humanoid_Robotics_book/
├── frontend/                    # Main documentation site
│   ├── docs/                    # Course content
│   │   ├── index.md             # Home page
│   │   ├── module-01-ros2/      # Module 1 chapters
│   │   ├── module-02-gazebo/    # Module 2 chapters
│   │   ├── module-03-isaac/     # Module 3 chapters
│   │   └── module-04-vla/       # Module 4 chapters
│   ├── src/
│   │   ├── components/          # React components
│   │   ├── css/                 # Styling
│   │   ├── pages/               # Custom pages
│   │   └── theme/               # Theme customization
│   ├── static/                  # Images, logos, assets
│   ├── package.json             # Dependencies & scripts
│   ├── docusaurus.config.js     # Site configuration
│   └── sidebars.js              # Navigation sidebar
│
├── backend/                     # FastAPI backend (optional)
│   ├── app/                     # Application code
│   ├── requirements.txt         # Python dependencies
│   └── alembic/                 # Database migrations
│
├── specs/                       # Original specifications
└── README.md                    # Project overview

```

---

## 🛠️ Common Commands

### Development

```bash
cd frontend

# Start dev server (auto-reloads on changes)
npm start

# Build for production
npm run build

# Serve production build locally
npm run serve

# Clean build directory
npm run clean
```

### Code Quality

```bash
# Format code
npm run format

# Lint check
npm run lint

# Type check
npm run typecheck
```

### Testing

```bash
# Build validation (ensures build works)
npm run build
```

---

## 🌐 Deployment

### Deploy to Vercel (Recommended)

1. Push code to GitHub
2. Go to [vercel.com](https://vercel.com)
3. Click "New Project"
4. Select your repository
5. Vercel auto-detects Docusaurus
6. Click "Deploy"
7. Site live in ~5 minutes!

**Automatic deploys**: Every push to main branch deploys automatically

### Deploy to Netlify

```bash
cd frontend
npm run build
# Upload frontend/build/ folder to Netlify
```

Or connect GitHub and Netlify will auto-deploy:
1. Go to [netlify.com](https://netlify.com)
2. Click "New site from Git"
3. Select repository
4. Build command: `cd frontend && npm run build`
5. Publish directory: `frontend/build`

### Deploy to GitHub Pages

```bash
cd frontend
npm run build
npm run deploy
# Your site will be at: yourname.github.io/repo-name
```

---

## 🔧 Customization

### Change Site Title

Edit `frontend/docusaurus.config.js`:
```javascript
const config = {
  title: 'My Custom Title',
  tagline: 'My custom tagline',
  // ... rest of config
};
```

### Add New Chapter

1. Create `.md` file in appropriate module folder:
   ```
   frontend/docs/module-01-ros2/03-your-chapter.md
   ```

2. Update sidebar in `frontend/sidebars.js`:
   ```javascript
   ros2: [
     'module-01-ros2/introduction',
     'module-01-ros2/nodes-topics-services',
     'module-01-ros2/your-chapter',  // Add this
   ],
   ```

3. Restart dev server - changes appear automatically

### Customize Colors

Edit `frontend/src/css/custom.css`:
```css
:root {
  --ifm-color-primary: #2e8555;      /* Primary color */
  --ifm-color-primary-dark: #29784c;
  --ifm-color-primary-darker: #277148;
  --ifm-color-primary-darkest: #205540;
  --ifm-color-primary-light: #33a566;
  --ifm-color-primary-lighter: #37b26d;
  --ifm-color-primary-lightest: #46cf9f;
}
```

---

## 🐛 Troubleshooting

### Problem: `npm install` fails

**Solution**:
```bash
# Clear npm cache
npm cache clean --force

# Delete node_modules and package-lock.json
rm -rf node_modules package-lock.json

# Reinstall
npm install
```

**Windows PowerShell**:
```powershell
npm cache clean --force
Remove-Item -Recurse node_modules
Remove-Item package-lock.json
npm install
```

### Problem: Port 3001 already in use

**Solution**:
```bash
# Use different port
npm start -- --port 3002
```

### Problem: Changes not appearing

**Solution**:
```bash
# Clear Docusaurus cache
npm run clean

# Restart dev server
npm start
```

### Problem: Build fails

**Solution**:
```bash
# Check for broken links
npm run build

# If still fails, check:
1. All markdown files are valid
2. No broken image references
3. No dead links in content
```

---

## 📊 Development Workflow

### Making Content Changes

1. Edit `.md` files in `frontend/docs/`
2. Changes auto-reload in browser
3. Commit and push:
   ```bash
   git add .
   git commit -m "docs: update chapter content"
   git push origin main
   ```
4. Vercel/Netlify auto-deploys

### Making Code Changes

1. Edit React components in `frontend/src/`
2. Changes auto-reload in browser
3. Test: `npm run build`
4. Commit and push
5. Auto-deploy

---

## 🚀 Backend Setup (Optional)

If you want to run the backend (not required for documentation):

### Prerequisites
- Python 3.11+
- PostgreSQL (or Docker)

### Setup

```bash
# Create virtual environment
python -m venv .venv

# Activate (Mac/Linux)
source .venv/bin/activate

# Activate (Windows PowerShell)
.\.venv\Scripts\Activate.ps1

# Install dependencies
pip install -r backend/requirements.txt

# Start server
cd backend
uvicorn app.main:app --reload --port 8000
```

**Note**: Backend is optional - the documentation site works without it.

---

## 📝 Editing Content

### Add a New Module

1. Create folder: `frontend/docs/module-05-name/`
2. Create chapters as `.md` files
3. Update `frontend/sidebars.js`:
   ```javascript
   {
     type: 'category',
     label: 'Module 5: Your Module',
     items: [
       'module-05-name/chapter-1',
       'module-05-name/chapter-2',
     ],
   }
   ```

### Markdown Features Supported

```markdown
# Heading 1
## Heading 2
### Heading 3

**Bold text**
*Italic text*

- Bullet point
- Another point

1. Numbered list
2. Another item

[Link text](https://example.com)

![Image alt](./image.png)

> Blockquote

\`\`\`python
# Code block
def hello():
    print("Hello, World!")
\`\`\`

<div className="custom-class">
Custom HTML/JSX
</div>
```

### Admonitions (Info Boxes)

```markdown
:::tip
This is a helpful tip!
:::

:::note
Important information
:::

:::warning
Be careful about this!
:::

:::danger
This is dangerous!
:::
```

---

## 🤝 Contributing

### Contribute Content

1. Fork repository on GitHub
2. Create branch: `git checkout -b feat/new-content`
3. Add or edit content in `frontend/docs/`
4. Test: `npm run build`
5. Commit: `git commit -m "docs: add new chapter"`
6. Push: `git push origin feat/new-content`
7. Open Pull Request on GitHub

### Contribute Code

1. Fork repository
2. Create branch: `git checkout -b feat/new-feature`
3. Edit code in `frontend/src/` or `backend/`
4. Test thoroughly
5. Submit Pull Request

---

## 📚 Documentation Standards

### Writing Guidelines
- Use clear, technical English
- Include code examples
- Add exercises at chapter end
- Keep lines ~100 characters
- Use proper Markdown formatting
- Add images when helpful

### Code Example Format
```markdown
### Example: Title of Example

\`\`\`python
# Language specified
# Code here
\`\`\`

**Output**:
\`\`\`
expected output
\`\`\`

**Explanation**: What the code does
```

---

## 🔒 Environment Variables

For backend (optional):

Create `backend/.env`:
```bash
# Backend Configuration
OPENAI_API_KEY=sk-...
QDRANT_URL=http://localhost:6333
DATABASE_URL=postgresql://...
ENVIRONMENT=development
LOG_LEVEL=INFO
```

**Important**: Never commit `.env` file!

---

## 📞 Getting Help

### Resources
- [Docusaurus Docs](https://docusaurus.io/)
- [Markdown Guide](https://www.markdownguide.org/)
- [React Documentation](https://react.dev/)

### Community
- Open an [GitHub Issue](https://github.com/abdulmateen5251/Physical_AI_Humanoid_Robotics_book/issues)
- Check [GitHub Discussions](https://github.com/abdulmateen5251/Physical_AI_Humanoid_Robotics_book/discussions)
- Email: abdulmateen5251@gmail.com

---

## ✅ Verification Checklist

After setup, verify everything works:

- [ ] `npm install` completes without errors
- [ ] `npm start` opens site at http://localhost:3001
- [ ] All 4 modules visible in sidebar
- [ ] Search functionality works
- [ ] Dark mode toggle works
- [ ] Navigation between chapters works
- [ ] Images load correctly
- [ ] Code blocks display properly
- [ ] Site responsive on mobile
- [ ] `npm run build` succeeds

---

## 🎉 You're Ready!

Your development environment is now set up. Start exploring and contributing!

### Next Steps
1. Read through course modules
2. Make a test change to content
3. Try deploying to production
4. Start contributing!

---

**Happy Learning! 🚀**

Last Updated: December 7, 2025

### Step 4: Start the Server

```powershell
# From backend directory
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

### Step 5: Test the API

Open another terminal:

```powershell
# Test health endpoint
curl http://localhost:8000/health

# Open interactive API docs
Start-Process "http://localhost:8000/docs"
```

---

## Full Setup (With Docker Desktop)

### Step 1: Install Docker Desktop

1. Download from: https://www.docker.com/products/docker-desktop/
2. Install and restart your computer
3. Start Docker Desktop
4. Verify installation:
   ```powershell
   docker --version
   docker compose version  # Note: "docker compose" not "docker-compose"
   ```

### Step 2: Update docker-compose Command

Modern Docker Desktop uses `docker compose` (with space) instead of `docker-compose`:

```powershell
# Old command (doesn't work):
docker-compose up -d

# New command (works with Docker Desktop):
docker compose up -d postgres qdrant redis
```

### Step 3: Start Services

```powershell
# From project root
docker compose up -d postgres qdrant redis

# Check services are running
docker compose ps
```

### Step 4: Configure and Run Backend

```powershell
# Configure environment (edit .env with your keys)
cd backend
notepad .env  # Edit with your API keys

# Run migrations
alembic upgrade head

# Start server
uvicorn app.main:app --reload
```

---

## Testing Without Docker (Recommended for Quick Start)

You can skip Docker entirely and use cloud services:

### 1. Qdrant Cloud (Vector Database)

1. Go to https://cloud.qdrant.io
2. Sign up (free tier available)
3. Create a cluster
4. Get your cluster URL and API key
5. Add to `.env`:
   ```
   QDRANT_URL=https://your-cluster.cloud.qdrant.io:6333
   QDRANT_API_KEY=your-api-key
   ```

### 2. Neon (Postgres Database)

1. Go to https://neon.tech
2. Sign up (free tier available)
3. Create a project
4. Copy connection string
5. Add to `.env`:
   ```
   DATABASE_URL=postgresql+asyncpg://user:pass@host.neon.tech/dbname
   ```

### 3. Redis (Optional - Skip for Phase 3)

Redis is only needed for Phase 6 (Personalization). You can skip it for now.

---

## Troubleshooting

### Issue: "docker-compose not recognized"

**Solution**: Use `docker compose` (with space) or install Docker Desktop.

```powershell
# Instead of:
docker-compose up -d

# Use:
docker compose up -d
```

### Issue: "Alembic JSONDecodeError"

**Cause**: Missing or invalid `.env` file.

**Solution**: 
1. Copy `.env.example` to `.env`
2. Edit `.env` with valid values
3. For CORS_ORIGINS, use proper JSON array:
   ```
   CORS_ORIGINS=["http://localhost:3000","http://localhost:8000"]
   ```

### Issue: "Cannot connect to Qdrant"

**Solution**: 
- If using Docker: Make sure `docker compose up -d qdrant` succeeded
- If using cloud: Check your QDRANT_URL and QDRANT_API_KEY
- Test connection: http://localhost:6333 (Docker) or your cloud URL

### Issue: "Cannot connect to Postgres"

**Solution**:
- If using Docker: Make sure `docker compose up -d postgres` succeeded
- If using cloud (Neon): Check your DATABASE_URL
- Verify format: `postgresql+asyncpg://user:pass@host:5432/dbname`

---

## Minimal Setup for Testing Phase 3

**What you actually need right now:**

1. ✅ Python environment (you have this)
2. ✅ Backend dependencies installed (you have this)
3. ⚠️ `.env` file with valid API keys (create this)
4. ⚠️ Qdrant instance (use cloud.qdrant.io - free)
5. ⚠️ Postgres instance (use neon.tech - free)
6. ❌ Redis (NOT needed until Phase 6)
7. ❌ Frontend (NOT needed until T039-T047)

**Minimal start command:**

```powershell
# 1. Edit .env
cd backend
notepad .env  # Add your API keys

# 2. Run migrations
alembic upgrade head

# 3. Start server
uvicorn app.main:app --reload

# 4. Test in another terminal
curl http://localhost:8000/health
curl http://localhost:8000/docs
```

---

## Next Steps After Setup

1. **Create sample content** (T048-T049)
2. **Index content** to Qdrant:
   ```powershell
   python scripts/ingest_to_qdrant.py --docs ../docs --collection physical_ai_humanoid_robotics_course
   ```
3. **Test RAG pipeline**:
   ```powershell
   curl -X POST http://localhost:8000/api/answer `
     -H "Content-Type: application/json" `
     -d '{"question": "What is ROS 2?", "scope": "fullbook"}'
   ```

---

## Getting API Keys (Free Tiers)

### OpenAI API Key
- Sign up: https://platform.openai.com/signup
- Add payment method (required, but you get $5 free credit)
- Create API key: https://platform.openai.com/api-keys
- Cost: ~$0.02 per 1000 questions (very cheap for testing)

### Qdrant Cloud
- Sign up: https://cloud.qdrant.io
- Free tier: 1GB storage, 100k vectors
- Create cluster → Get API key
- Sufficient for the entire textbook

### Neon Postgres
- Sign up: https://neon.tech
- Free tier: 0.5GB storage, 1 database
- Create project → Copy connection string
- More than enough for user data

---

## Support

If you encounter issues:
1. Check logs: `uvicorn app.main:app --reload --log-level debug`
2. Verify `.env` file: `cat .env` (PowerShell: `Get-Content .env`)
3. Test services individually:
   - Qdrant: Visit your cluster URL in browser
   - Postgres: `psql <DATABASE_URL>`
   - API: `curl http://localhost:8000/health`
