"""Embedding service for text vectorization."""

from functools import lru_cache
from sentence_transformers import SentenceTransformer
from app.config import get_settings


class EmbeddingService:
    """Service for generating text embeddings."""

    def __init__(self, model_name: str = None):
        settings = get_settings()
        self.model_name = model_name or settings.embedding_model
        self._model = None

    @property
    def model(self) -> SentenceTransformer:
        """Lazy load the model."""
        if self._model is None:
            self._model = SentenceTransformer(self.model_name)
        return self._model

    def embed_text(self, text: str) -> list[float]:
        """Generate embedding for text."""
        embedding = self.model.encode(text, convert_to_numpy=True)
        return embedding.tolist()

    def embed_texts(self, texts: list[str]) -> list[list[float]]:
        """Generate embeddings for multiple texts."""
        embeddings = self.model.encode(texts, convert_to_numpy=True)
        return embeddings.tolist()


@lru_cache
def get_embedding_service() -> EmbeddingService:
    """Get cached embedding service."""
    return EmbeddingService()
