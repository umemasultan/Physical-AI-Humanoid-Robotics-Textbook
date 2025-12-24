"""Application configuration using Pydantic Settings."""

from functools import lru_cache

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
    )

    # Qdrant Configuration
    qdrant_url: str = "http://localhost:6333"
    qdrant_api_key: str | None = None

    # LLM Configuration
    groq_api_key: str | None = None

    # Embedding Configuration
    embedding_model: str = "all-MiniLM-L6-v2"

    # CORS Configuration
    cors_origins: str = "http://localhost:3000"

    # Rate Limiting
    rate_limit_per_minute: int = 10

    # Application
    app_env: str = "development"
    log_level: str = "INFO"

    @property
    def cors_origins_list(self) -> list[str]:
        """Parse CORS origins from comma-separated string."""
        return [origin.strip() for origin in self.cors_origins.split(",")]

    @property
    def is_production(self) -> bool:
        """Check if running in production environment."""
        return self.app_env.lower() == "production"


@lru_cache
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()
