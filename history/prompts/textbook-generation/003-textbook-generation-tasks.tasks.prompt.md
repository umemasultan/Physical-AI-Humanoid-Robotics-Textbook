---
id: 003
title: Textbook Generation Task Breakdown
stage: tasks
date: 2025-12-24
surface: agent
model: claude-sonnet-4-5-20250929
feature: textbook-generation
branch: master
user: user
command: /sp.tasks
labels: ["tasks", "breakdown", "implementation", "hackathon"]
links:
  spec: specs/textbook-generation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/textbook-generation/tasks.md
tests:
  - null
---

## Prompt

/sp.task

Project: Physical AI & Humanoid Robotics — Essentials

Context:
The project constitution, specification, and execution plan are finalized.
The goal now is to break the plan into precise, actionable tasks.

Objective:
Generate a complete task breakdown that can be executed sequentially or in parallel
by a small team or a solo developer during a hackathon.

Task Requirements:
- Tasks must be atomic, concrete, and unambiguous
- Each task must have:
  - Task ID
  - Task title
  - Short description
  - Expected output / deliverable
- Tasks should be ordered logically
- Clearly mark which tasks are:
  - Core (must-have)
  - Optional (nice-to-have)

Scope to Cover:
1. Project initialization & repository setup
2. Docusaurus installation & configuration
3. Textbook content writing (all 6 chapters)
4. Markdown normalization for RAG
5. Embedding generation & ingestion
6. Qdrant setup & vector indexing
7. FastAPI RAG backend
8. RAG guardrails (book-only answers)
9. Select-text → Ask AI integration
10. Frontend–backend wiring
11. Environment & secrets management
12. Deployment (frontend + backend)
13. Smoke testing & validation
14. Documentation & README

Constraints:
- Free-tier services only
- Lightweight embeddings
- No GPU dependency
- No hallucinated answers

Output Format:
- Task list only
- Bullet or table format
- Clear ordering
- No explanations outside tasks

Success Criteria:
- Tasks can be followed step-by-step without clarification
- All tasks together fully realize the specified system

## Response snapshot

Created comprehensive task breakdown at `specs/textbook-generation/tasks.md` with **260 total tasks**:

**By Phase:**
- Phase 0 (Setup): 10 tasks
- Phase 1 (Frontend Shell): 16 tasks
- Phase 2 (Content): 69 tasks (6 chapters × ~11 tasks each)
- Phase 3 (Backend): 41 tasks
- Phase 4 (Embedding): 17 tasks
- Phase 5 (Chat UI): 32 tasks
- Phase 6 (Select-to-Ask): 12 tasks
- Phase 7 (Deployment): 21 tasks
- Phase 8 (Testing): 29 tasks
- Phase 9 (Optional): 13 tasks

**Task Classification:**
- Core (must-have): 239 tasks
- Optional (nice-to-have): 21 tasks

**Key Features:**
- [P] markers for parallelizable tasks
- [CORE]/[OPT] classification
- Exact file paths for each task
- Verification checkpoints per phase
- Dependency diagram included

## Outcome

- Impact: Complete actionable roadmap for hackathon execution
- Tests: N/A (planning document)
- Files: 1 created (specs/textbook-generation/tasks.md)
- Next prompts: Begin Phase 0 implementation
- Reflection: 260 tasks is comprehensive but achievable; content (Phase 2) remains the bottleneck

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): Standard task generation
- Next experiment: Track actual vs estimated effort per phase
