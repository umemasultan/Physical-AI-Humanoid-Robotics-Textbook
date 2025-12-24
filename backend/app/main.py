"""FastAPI application entry point."""

from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.errors import RateLimitExceeded
from slowapi.util import get_remote_address

from app.config import get_settings

settings = get_settings()

# Rate limiter
limiter = Limiter(key_func=get_remote_address)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan handler for startup and shutdown."""
    # Startup
    print(f"Starting Physical AI Textbook Backend v1.0.0 ({settings.app_env})")
    yield
    # Shutdown
    print("Shutting down...")


app = FastAPI(
    title="Physical AI Textbook API",
    description="RAG-powered chatbot for the Physical AI & Humanoid Robotics textbook",
    version="1.0.0",
    lifespan=lifespan,
)

# Add rate limiting
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)


@app.get("/api/health")
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "version": "1.0.0",
        "environment": settings.app_env,
    }


@app.get("/")
async def root():
    """Root endpoint with API information."""
    return {
        "name": "Physical AI Textbook API",
        "version": "1.0.0",
        "docs": "/docs",
        "health": "/api/health",
    }
