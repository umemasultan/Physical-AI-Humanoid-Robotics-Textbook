---
id: 001
title: Textbook Generation Feature Specification
stage: spec
date: 2025-12-24
surface: agent
model: claude-sonnet-4-5-20250929
feature: textbook-generation
branch: master
user: user
command: /sp.specify
labels: ["specification", "textbook", "rag", "docusaurus", "physical-ai"]
links:
  spec: specs/textbook-generation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/textbook-generation/spec.md
tests:
  - null
---

## Prompt

/sp.specify

Feature: textbook-generation

Objective:
Define a complete, unambiguous specification for building the AI-native textbook with RAG chatbot.

Book Structure:
1. Introduction to Physical AI
2. Basics of Humanoid Robotics
3. ROS 2 Fundamentals
4. Digital Twin Simulation (Gazebo + Isaac)
5. Vision-Language-Action Systems
6. Capstone

Technical Requirements:
- Docusaurus
- Auto sidebar
- RAG backend (Qdrant + Neon)
- Free-tier embeddings

Optional:
- Urdu translation
- Personalize chapter

Output:
Full specification.

## Response snapshot

Created comprehensive specification at `specs/textbook-generation/spec.md` with:

**7 User Stories (Prioritized):**
- P1: Read Textbook Content, Ask AI Chatbot
- P2: Select Text to Ask AI, Search Textbook, Mobile View
- P3: Urdu Translation, Personalized Chapter (Optional)

**25 Functional Requirements covering:**
- Frontend: Docusaurus rendering, sidebar, syntax highlighting, chat UI
- Backend: RAG pipeline, 512-token chunks, 0.7 similarity threshold
- Content: Chapter structure, code examples, glossary

**Key Technical Decisions:**
- Stack: Docusaurus 3.x + FastAPI + Qdrant + Groq/OpenRouter
- Embeddings: all-MiniLM-L6-v2 (22M params, 80MB)
- API: POST /api/chat with source citations

**Success Criteria:**
- Lighthouse >90, RAG <3s, $0/month infra
- 90% correct citations, 100% "not found" for out-of-scope

## Outcome

- Impact: Complete specification ready for planning phase
- Tests: N/A (specification document)
- Files: 1 created (specs/textbook-generation/spec.md)
- Next prompts: /sp.plan to create implementation plan
- Reflection: Spec covers all user journeys with testable acceptance criteria

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): Standard specification generation
- Next experiment: Validate RAG architecture assumptions during planning
