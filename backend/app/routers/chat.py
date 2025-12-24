"""Chat router for RAG-based Q&A."""

from fastapi import APIRouter, HTTPException
from app.models.schemas import ChatRequest, ChatResponse
from app.services.rag_service import get_rag_service

router = APIRouter(prefix="/api", tags=["chat"])


@router.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Ask a question about the Physical AI textbook.

    Returns an AI-generated answer based on the textbook content.
    """
    try:
        rag_service = get_rag_service()
        answer, sources = await rag_service.ask(request.question)
        return ChatResponse(answer=answer, sources=sources)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
