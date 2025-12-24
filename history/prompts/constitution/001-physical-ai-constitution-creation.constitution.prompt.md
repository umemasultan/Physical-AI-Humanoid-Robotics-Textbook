---
id: 001
title: Physical AI Constitution Creation
stage: constitution
date: 2025-12-24
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "project-setup", "physical-ai", "robotics"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - null
---

## Prompt

/sp.constitution

Project Name: Physical AI & Humanoid Robotics — Essentials

Purpose:
Create a short, clean, professional AI-Native textbook based on the Physical AI & Humanoid Robotics course.
The book must serve as a fast, simple, high-quality learning resource built with a modern Docusaurus UI
and a fully integrated free-tier RAG chatbot.

Scope:
- 6 short chapters:
  1. Introduction to Physical AI
  2. Basics of Humanoid Robotics
  3. ROS 2 Fundamentals
  4. Digital Twin Simulation (Gazebo + Isaac)
  5. Vision-Language-Action Systems
  6. Capstone: Simple AI-Robot Pipeline
- Clean UI
- Free-tier friendly
- Lightweight embeddings

Core Principles:
- Simplicity
- Accuracy
- Minimalism
- Fast builds
- Free-tier architecture
- RAG answers ONLY from book text

Key Features:
- Docusaurus textbook
- RAG chatbot (Qdrant + Neon + FastAPI)
- Select-text → Ask AI
- Optional Urdu / Personalize features

Constraints:
- No heavy GPU usage
- Minimal embeddings

Success Criteria:
- Build success
- Accurate chatbot
- Clean UI
- Smooth GitHub Pages deployment

Generate full constitution.

## Response snapshot

Created comprehensive constitution at `.specify/memory/constitution.md` with:

**6 Core Principles:**
1. Simplicity First - minimal dependencies, no over-engineering
2. Free-Tier Architecture - $0/month target with Qdrant, Neon, GitHub Pages
3. Content Accuracy - tested code examples, current ROS 2 LTS
4. RAG Fidelity - answers ONLY from book text, no hallucination
5. Clean UI/UX - Docusaurus, mobile-responsive, accessible
6. Minimalist Content - 6 focused chapters, 10-15 pages each

**Technical Standards:**
- Stack: Docusaurus 3.x, FastAPI, Qdrant Cloud, Neon PostgreSQL
- Performance budgets: <2s page load, <3s RAG response
- Code quality: TypeScript/Python, 80% test coverage

**RAG Architecture:**
- 512-token chunks with 50-token overlap
- 0.7 similarity threshold
- Always cite sources, never extrapolate

**Success Criteria:**
- Build success, 90%+ accurate citations, Lighthouse >90, $0 cost

## Outcome

- Impact: Established project foundation and governing principles
- Tests: N/A (documentation)
- Files: 1 created (.specify/memory/constitution.md)
- Next prompts: /sp.specify for feature spec, project scaffolding
- Reflection: Constitution provides clear technical constraints and success metrics for free-tier RAG textbook

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): Standard constitution generation
- Next experiment: Validate free-tier limits with actual implementation
