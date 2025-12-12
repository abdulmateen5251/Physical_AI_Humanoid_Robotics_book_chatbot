from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api import chat, history
from app.models.database import init_db
from app.config import get_settings

settings = get_settings()

app = FastAPI(title="Book RAG Chatbot API", version="1.0.0")

# CORS - Allow frontend to connect
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://localhost:3001",
        "http://127.0.0.1:3000",
        "http://127.0.0.1:3001"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize database
@app.on_event("startup")
async def startup_event():
    init_db()

# Health check
@app.get("/health")
async def health():
    return {"status": "healthy"}

# Include routers
app.include_router(chat.router)
app.include_router(history.router)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
