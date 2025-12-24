"""Pydantic schemas for API request/response models."""

from pydantic import BaseModel, Field


class ChatRequest(BaseModel):
    """Request model for chat endpoint."""

    question: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="The user's question about the textbook content",
    )


class ChatResponse(BaseModel):
    """Response model for chat endpoint."""

    answer: str = Field(..., description="The AI-generated answer")
    sources: list[str] = Field(
        default_factory=list,
        description="List of source documents used to generate the answer",
    )


class HealthResponse(BaseModel):
    """Response model for health check endpoint."""

    status: str
    version: str
    environment: str
