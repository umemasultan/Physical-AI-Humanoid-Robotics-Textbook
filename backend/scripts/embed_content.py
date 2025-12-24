"""Script to embed textbook content into Qdrant vector database."""

import os
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams, PointStruct
from sentence_transformers import SentenceTransformer
import hashlib
import re


def load_markdown_files(docs_path: str) -> list[dict]:
    """Load all markdown files from docs directory."""
    documents = []
    docs_dir = Path(docs_path)

    for md_file in docs_dir.rglob("*.md"):
        with open(md_file, "r", encoding="utf-8") as f:
            content = f.read()

        # Remove frontmatter
        content = re.sub(r"^---\n.*?\n---\n", "", content, flags=re.DOTALL)

        # Get source name
        source = str(md_file.relative_to(docs_dir))

        documents.append({
            "content": content,
            "source": source,
            "path": str(md_file)
        })

    return documents


def chunk_document(doc: dict, chunk_size: int = 500, overlap: int = 100) -> list[dict]:
    """Split document into chunks."""
    content = doc["content"]
    chunks = []

    # Split by sections first
    sections = re.split(r"\n## ", content)

    for i, section in enumerate(sections):
        if i > 0:
            section = "## " + section

        # If section is too long, split further
        if len(section) > chunk_size:
            words = section.split()
            current_chunk = []
            current_length = 0

            for word in words:
                current_chunk.append(word)
                current_length += len(word) + 1

                if current_length >= chunk_size:
                    chunk_text = " ".join(current_chunk)
                    chunks.append({
                        "content": chunk_text,
                        "source": doc["source"],
                    })
                    # Keep overlap
                    current_chunk = current_chunk[-overlap // 5:]
                    current_length = sum(len(w) + 1 for w in current_chunk)

            if current_chunk:
                chunks.append({
                    "content": " ".join(current_chunk),
                    "source": doc["source"],
                })
        else:
            chunks.append({
                "content": section,
                "source": doc["source"],
            })

    return chunks


def generate_id(text: str) -> str:
    """Generate unique ID for chunk."""
    return hashlib.md5(text.encode()).hexdigest()


def main():
    # Configuration
    DOCS_PATH = Path(__file__).parent.parent.parent / "frontend" / "docs"
    QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    COLLECTION_NAME = "textbook_chunks"
    EMBEDDING_MODEL = "all-MiniLM-L6-v2"

    print(f"Loading documents from: {DOCS_PATH}")

    # Load documents
    documents = load_markdown_files(str(DOCS_PATH))
    print(f"Loaded {len(documents)} documents")

    # Chunk documents
    all_chunks = []
    for doc in documents:
        chunks = chunk_document(doc)
        all_chunks.extend(chunks)
    print(f"Created {len(all_chunks)} chunks")

    # Initialize embedding model
    print(f"Loading embedding model: {EMBEDDING_MODEL}")
    model = SentenceTransformer(EMBEDDING_MODEL)

    # Generate embeddings
    print("Generating embeddings...")
    texts = [chunk["content"] for chunk in all_chunks]
    embeddings = model.encode(texts, show_progress_bar=True)
    print(f"Generated {len(embeddings)} embeddings")

    # Connect to Qdrant
    print(f"Connecting to Qdrant: {QDRANT_URL}")
    if QDRANT_API_KEY:
        client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    else:
        client = QdrantClient(url=QDRANT_URL)

    # Create collection
    vector_size = embeddings[0].shape[0]
    print(f"Creating collection with vector size: {vector_size}")

    try:
        client.delete_collection(COLLECTION_NAME)
    except Exception:
        pass

    client.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE),
    )

    # Upload points
    print("Uploading to Qdrant...")
    points = [
        PointStruct(
            id=i,
            vector=embeddings[i].tolist(),
            payload={
                "content": all_chunks[i]["content"],
                "source": all_chunks[i]["source"],
            }
        )
        for i in range(len(all_chunks))
    ]

    client.upsert(collection_name=COLLECTION_NAME, points=points)
    print(f"Uploaded {len(points)} points to Qdrant")

    print("Done!")


if __name__ == "__main__":
    main()
