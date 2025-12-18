@echo off
REM Start all services with Docker Compose

echo Starting RAG Chatbot with Docker Compose...
echo Services:
echo   - Auth Backend (Node.js): http://localhost:3001
echo   - FastAPI Backend: http://localhost:8090
echo   - Frontend (Docusaurus): http://localhost:3000
echo.

REM Check if .env exists
if not exist .env (
    echo Warning: .env file not found!
    echo Copying .env.example to .env...
    copy .env.example .env
    echo Please edit .env file with your actual credentials before continuing.
    pause
)

echo Building and starting containers...
docker-compose up --build

REM Note: Use Ctrl+C to stop all services
