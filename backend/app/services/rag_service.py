"""RAG service for question answering."""

import httpx
from qdrant_client import QdrantClient
from qdrant_client.http.exceptions import ResponseHandlingException

from app.config import get_settings
from app.services.embedding_service import get_embedding_service


class RAGService:
    """Service for RAG-based question answering."""

    COLLECTION_NAME = "textbook_chunks"

    def __init__(self):
        self.settings = get_settings()
        self.embedding_service = get_embedding_service()
        self._qdrant = None

    @property
    def qdrant(self) -> QdrantClient:
        """Lazy load Qdrant client."""
        if self._qdrant is None:
            if self.settings.qdrant_api_key:
                self._qdrant = QdrantClient(
                    url=self.settings.qdrant_url,
                    api_key=self.settings.qdrant_api_key,
                )
            else:
                self._qdrant = QdrantClient(url=self.settings.qdrant_url)
        return self._qdrant

    async def search_similar(self, query: str, top_k: int = 3) -> list[dict]:
        """Search for similar chunks."""
        try:
            query_embedding = self.embedding_service.embed_text(query)
            results = self.qdrant.search(
                collection_name=self.COLLECTION_NAME,
                query_vector=query_embedding,
                limit=top_k,
            )
            return [
                {
                    "content": hit.payload.get("content", ""),
                    "source": hit.payload.get("source", "Unknown"),
                    "score": hit.score,
                }
                for hit in results
            ]
        except (ResponseHandlingException, Exception):
            return []

    async def generate_answer(self, question: str, context_chunks: list[dict]) -> str:
        """Generate answer using LLM."""
        if not self.settings.groq_api_key:
            return self._fallback_answer(question, context_chunks)

        context = "\n\n".join(
            f"[{chunk['source']}]: {chunk['content']}"
            for chunk in context_chunks
        )

        system_prompt = """You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
Answer questions based on the provided context. Be concise and accurate.
If the context doesn't have the answer, say so honestly."""

        user_prompt = f"""Context:
{context}

Question: {question}

Answer:"""

        try:
            async with httpx.AsyncClient() as client:
                response = await client.post(
                    "https://api.groq.com/openai/v1/chat/completions",
                    headers={
                        "Authorization": f"Bearer {self.settings.groq_api_key}",
                        "Content-Type": "application/json",
                    },
                    json={
                        "model": "llama-3.1-8b-instant",
                        "messages": [
                            {"role": "system", "content": system_prompt},
                            {"role": "user", "content": user_prompt},
                        ],
                        "max_tokens": 500,
                        "temperature": 0.7,
                    },
                    timeout=30.0,
                )
                response.raise_for_status()
                data = response.json()
                return data["choices"][0]["message"]["content"]
        except Exception as e:
            return f"Error: {str(e)}"

    def _fallback_answer(self, question: str, chunks: list[dict]) -> str:
        """Fallback when no LLM configured."""
        if not chunks:
            return "I couldn't find relevant information. Please try a different question."
        return f"Based on the textbook:\n\n{chunks[0]['content']}\n\n(Source: {chunks[0]['source']})"

    async def ask(self, question: str) -> tuple[str, list[str]]:
        """Process question and return answer with sources."""
        chunks = await self.search_similar(question, top_k=3)
        answer = await self.generate_answer(question, chunks)
        sources = list(set(c["source"] for c in chunks if c.get("source")))
        return answer, sources


def get_rag_service() -> RAGService:
    """Get RAG service instance."""
    return RAGService()
