#!/usr/bin/env pwsh
Set-Location "$PSScriptRoot\backend"
Write-Host "Starting Authentication Backend on Port 3001..." -ForegroundColor Green
npm run dev
