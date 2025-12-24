---
id: 002
title: Textbook Generation Implementation Plan
stage: plan
date: 2025-12-24
surface: agent
model: claude-sonnet-4-5-20250929
feature: textbook-generation
branch: master
user: user
command: /sp.plan
labels: ["planning", "implementation", "architecture", "phases"]
links:
  spec: specs/textbook-generation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/textbook-generation/plan.md
tests:
  - null
---

## Prompt

/sp.plan

Project: Physical AI & Humanoid Robotics — Essentials

Context:
A short, AI-native textbook with an integrated free-tier RAG chatbot is to be built.
The constitution and full specification are already finalized.

Objective:
Generate a clear, step-by-step execution plan that transforms the specification into a working product.

Plan Requirements:
- Divide the work into clear phases
- Each phase must include:
  - Purpose
  - Key tasks
  - Expected outputs
- Keep the plan minimal, fast, and free-tier friendly
- Prioritize correctness, simplicity, and hackathon delivery speed

Scope to Cover:
1. Repository & folder structure
2. Textbook content creation (6 chapters)
3. Docusaurus setup & UI
4. RAG chatbot backend (Qdrant + Neon + FastAPI)
5. Embedding & ingestion pipeline
6. Text-selection → Ask AI flow
7. Deployment (GitHub Pages + free backend)
8. Optional enhancements (Urdu / personalization)

Constraints:
- No heavy GPU usage
- Lightweight embeddings only
- RAG must answer strictly from book content
- Clean GitHub Pages deployment

Output Format:
- Phase-based execution plan
- Bullet points only
- Clear ordering
- No filler or explanations outside the plan

Success Criteria:
- Fully buildable project
- Accurate, source-grounded chatbot
- Clean UI
- Free-tier compliant architecture

## Response snapshot

Created comprehensive 9-phase implementation plan at `specs/textbook-generation/plan.md`:

**Phase 0**: Project Setup - folder structure, dependencies
**Phase 1**: Docusaurus Frontend Shell - config, sidebar, search
**Phase 2**: Chapter Content Creation - 6 chapters, diagrams, glossary (CRITICAL PATH)
**Phase 3**: RAG Backend Core - FastAPI, embeddings, Qdrant, Groq
**Phase 4**: Content Embedding Pipeline - chunking, indexing script
**Phase 5**: Chatbot UI Integration - ChatPanel, useChat hook
**Phase 6**: Select-to-Ask Feature - text selection tooltip
**Phase 7**: Deployment Pipeline - GitHub Pages + Vercel
**Phase 8**: Testing & Polish - Lighthouse, query validation
**Phase 9**: Optional Enhancements - Urdu, personalization

**Key Decisions:**
- Phases 2 & 3 can run in parallel
- Phase 2 (Content) is the critical path bottleneck
- Backend deploys to Vercel (simplest free option)
- all-MiniLM-L6-v2 for CPU-only embeddings

## Outcome

- Impact: Clear execution roadmap from spec to deployed product
- Tests: N/A (planning document)
- Files: 1 created (specs/textbook-generation/plan.md)
- Next prompts: /sp.tasks to generate detailed task breakdown
- Reflection: Plan identifies content creation as the bottleneck; backend work can proceed independently

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): Standard planning workflow
- Next experiment: Validate Qdrant free tier limits during Phase 4
