#!/usr/bin/env pwsh
# Start all services with Docker Compose

Write-Host "Starting RAG Chatbot with Docker Compose..." -ForegroundColor Green
Write-Host "Services:" -ForegroundColor Cyan
Write-Host "  - Auth Backend (Node.js): http://localhost:3001" -ForegroundColor Yellow
Write-Host "  - FastAPI Backend: http://localhost:8090" -ForegroundColor Yellow
Write-Host "  - Frontend (Docusaurus): http://localhost:3000" -ForegroundColor Yellow
Write-Host ""

# Check if .env exists
if (-not (Test-Path ".env")) {
    Write-Host "Warning: .env file not found!" -ForegroundColor Red
    Write-Host "Copying .env.example to .env..." -ForegroundColor Yellow
    Copy-Item ".env.example" ".env"
    Write-Host "Please edit .env file with your actual credentials before continuing." -ForegroundColor Yellow
    Write-Host "Press any key to continue or Ctrl+C to exit..."
    $null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
}

# Build and start services
Write-Host "Building and starting containers..." -ForegroundColor Green
docker-compose up --build

# Note: Use Ctrl+C to stop all services
