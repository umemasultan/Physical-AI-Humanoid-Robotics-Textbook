# Physical AI & Humanoid Robotics — Essentials Constitution

## Project Vision

A short, clean, professional AI-Native textbook serving as a fast, simple, high-quality learning resource for Physical AI and Humanoid Robotics. Built with modern Docusaurus UI and integrated free-tier RAG chatbot.

## Core Principles

### I. Simplicity First

Every design decision prioritizes simplicity over feature richness.
- Minimal dependencies; prefer native solutions
- No unnecessary abstractions or over-engineering
- Clear, readable code over clever code
- If it can be removed without breaking core functionality, remove it

### II. Free-Tier Architecture

All infrastructure must work within free-tier limits.
- GitHub Pages for static hosting
- Qdrant Cloud free tier for vector storage
- Neon free tier for PostgreSQL
- No heavy GPU usage; CPU-friendly embeddings only
- Lightweight models (e.g., all-MiniLM-L6-v2 or similar)
- Cost: $0/month target

### III. Content Accuracy

Educational content must be technically accurate and up-to-date.
- All code examples must be tested and runnable
- ROS 2 content targets Humble/Iron LTS releases
- Simulation examples work with both Gazebo and Isaac Sim
- No outdated APIs or deprecated practices
- Citations required for technical claims

### IV. RAG Fidelity

The chatbot answers ONLY from book text — no hallucination allowed.
- Strict retrieval from embedded book chunks
- Clear "I don't know" responses when content not found
- Source citations for every answer
- No external knowledge injection
- Chunk overlap strategy for context preservation

### V. Clean UI/UX

Modern, professional interface with minimal cognitive load.
- Docusaurus default theme with light customization
- Mobile-responsive design
- Fast page loads (<2s target)
- Accessible (WCAG 2.1 AA minimum)
- Select-text → Ask AI interaction pattern

### VI. Minimalist Content

Six focused chapters; no bloat.
1. Introduction to Physical AI
2. Basics of Humanoid Robotics
3. ROS 2 Fundamentals
4. Digital Twin Simulation (Gazebo + Isaac)
5. Vision-Language-Action Systems
6. Capstone: Simple AI-Robot Pipeline

Each chapter: ~10-15 pages, clear learning objectives, practical examples.

## Technical Standards

### Stack Requirements

| Layer | Technology | Constraint |
|-------|------------|------------|
| Frontend | Docusaurus 3.x | Static export only |
| Hosting | GitHub Pages | Free tier |
| Vector DB | Qdrant Cloud | Free tier (1GB) |
| SQL DB | Neon PostgreSQL | Free tier (0.5GB) |
| Backend | FastAPI | Serverless-ready |
| Embeddings | Sentence Transformers | CPU-only, <500MB model |
| LLM | OpenRouter / Groq | Free tier or pay-per-token |

### Code Quality

- TypeScript for all frontend code
- Python 3.11+ for backend
- Type hints mandatory
- Linting: ESLint (frontend), Ruff (backend)
- Formatting: Prettier (frontend), Black (backend)
- No `any` types; no `# type: ignore` without justification

### Testing Requirements

- Unit tests for all utility functions
- Integration tests for RAG pipeline
- E2E tests for critical user flows (search, chat)
- Minimum 80% coverage for backend
- All tests must pass before merge

### Performance Budgets

| Metric | Target |
|--------|--------|
| First Contentful Paint | <1.5s |
| Largest Contentful Paint | <2.5s |
| Bundle size (JS) | <200KB gzipped |
| Embedding latency | <100ms per chunk |
| RAG response time | <3s end-to-end |

### Security

- No secrets in repository; use environment variables
- API keys via `.env` (never committed)
- Input sanitization on all user inputs
- Rate limiting on API endpoints
- CORS properly configured

## Content Standards

### Writing Style

- Active voice preferred
- Short paragraphs (3-5 sentences max)
- Code examples annotated with comments
- Diagrams for complex concepts
- Glossary terms linked on first use

### Chapter Structure

Each chapter follows this template:
1. Learning Objectives (3-5 bullet points)
2. Prerequisites (what reader should know)
3. Core Content (sections with examples)
4. Hands-On Exercise (practical task)
5. Summary (key takeaways)
6. Further Reading (optional resources)

### Code Examples

- Must be copy-paste runnable
- Include required imports
- Show expected output in comments
- Error handling demonstrated
- ROS 2 examples use standard message types

## RAG Architecture

### Embedding Strategy

- Chunk size: 512 tokens with 50-token overlap
- Metadata: chapter, section, page number
- Re-embed on content change only
- Batch processing for initial load

### Retrieval Pipeline

```
User Query → Embed → Vector Search (top-5) → Rerank → Context Assembly → LLM → Response
```

### Answer Generation Rules

1. If no relevant chunks found: "I couldn't find information about that in the book."
2. Always cite source: "According to Chapter X, Section Y..."
3. Never extrapolate beyond retrieved content
4. Confidence threshold: 0.7 similarity minimum

## Development Workflow

### Branch Strategy

- `main`: Production-ready, deployed to GitHub Pages
- `develop`: Integration branch
- `feature/*`: New features
- `fix/*`: Bug fixes
- `docs/*`: Documentation updates

### Commit Convention

```
<type>(<scope>): <description>

Types: feat, fix, docs, style, refactor, test, chore
Scope: frontend, backend, content, rag, infra
```

### PR Requirements

- Descriptive title and description
- All tests passing
- No linting errors
- Preview deployment for frontend changes
- At least one approval (if team project)

## Deployment

### GitHub Pages

- Automatic deployment on `main` push
- Build artifacts cached
- Custom domain support ready
- 404 page configured

### Backend (Serverless)

- FastAPI deployed to Vercel/Railway/Render free tier
- Environment variables in platform secrets
- Health check endpoint required
- Graceful degradation if RAG unavailable

## Constraints (Hard Limits)

- **NO** heavy GPU models or inference
- **NO** paid-only services in core path
- **NO** more than 6 chapters
- **NO** external knowledge in RAG answers
- **NO** breaking changes without migration path
- **NO** secrets in codebase

## Success Criteria

| Criterion | Measurement |
|-----------|-------------|
| Build Success | `npm run build` exits 0 |
| Accurate Chatbot | 90%+ answers cite correct source |
| Clean UI | Lighthouse score >90 |
| Deployment | GitHub Pages live and accessible |
| Free Tier | $0/month infrastructure cost |
| Performance | All budgets met |

## Optional Features (Phase 2)

These are explicitly out of scope for v1.0:
- Urdu translation toggle
- Personalization/bookmarks
- Progress tracking
- Quiz/assessment system
- PDF export

## Governance

This constitution is the authoritative source for project decisions.

- All PRs must verify compliance with these principles
- Amendments require documented justification
- Simplicity principle takes precedence in conflicts
- When in doubt, choose the simpler option

**Version**: 1.0.0 | **Ratified**: 2025-12-24 | **Last Amended**: 2025-12-24
