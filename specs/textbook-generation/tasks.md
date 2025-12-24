# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/textbook-generation/`
**Prerequisites**: plan.md (required), spec.md (required)

**Format**: `[ID] [P?] [Type] Description`
- **[P]**: Can run in parallel (different files, no dependencies)
- **[Type]**: CORE (must-have) or OPT (optional/nice-to-have)

---

## Phase 0: Project Setup

**Purpose**: Initialize repository structure and development environment

- [ ] T001 [CORE] Create root folder structure: `frontend/`, `backend/`, `content/`, `.github/workflows/`
- [ ] T002 [CORE] Create `.gitignore` with Node, Python, IDE, and `.env` exclusions
- [ ] T003 [CORE] Initialize Docusaurus 3.x project in `frontend/` via `npx create-docusaurus@latest`
- [ ] T004 [CORE] Create `frontend/package.json` scripts: `start`, `build`, `serve`
- [ ] T005 [P] [CORE] Initialize Python project in `backend/` with `pyproject.toml` or `setup.py`
- [ ] T006 [P] [CORE] Create `backend/requirements.txt` with: fastapi, uvicorn, sentence-transformers, qdrant-client, httpx, python-dotenv, slowapi
- [ ] T007 [CORE] Create `backend/.env.example` with placeholder variables: QDRANT_URL, QDRANT_API_KEY, GROQ_API_KEY, CORS_ORIGINS
- [ ] T008 [CORE] Create minimal `backend/app/main.py` with FastAPI app and `/health` endpoint
- [ ] T009 [CORE] Verify: `cd frontend && npm install && npm run start` opens localhost:3000
- [ ] T010 [CORE] Verify: `cd backend && pip install -r requirements.txt && uvicorn app.main:app --reload` runs

**Checkpoint**: Development environment ready

---

## Phase 1: Docusaurus Frontend Shell

**Purpose**: Build static textbook UI with placeholder content

- [ ] T011 [CORE] Configure `frontend/docusaurus.config.ts`: site title, tagline, URL, base path, navbar
- [ ] T012 [CORE] Configure `frontend/sidebars.ts` for auto-generated sidebar from `docs/` folder
- [ ] T013 [P] [CORE] Create `frontend/docs/01-intro-physical-ai/index.mdx` with placeholder content
- [ ] T014 [P] [CORE] Create `frontend/docs/02-humanoid-basics/index.mdx` with placeholder content
- [ ] T015 [P] [CORE] Create `frontend/docs/03-ros2-fundamentals/index.mdx` with placeholder content
- [ ] T016 [P] [CORE] Create `frontend/docs/04-digital-twin/index.mdx` with placeholder content
- [ ] T017 [P] [CORE] Create `frontend/docs/05-vla-systems/index.mdx` with placeholder content
- [ ] T018 [P] [CORE] Create `frontend/docs/06-capstone/index.mdx` with placeholder content
- [ ] T019 [CORE] Create chapter MDX template with sections: Learning Objectives, Prerequisites, Content, Exercise, Summary
- [ ] T020 [CORE] Install local search plugin: `npm install @easyops-cn/docusaurus-search-local`
- [ ] T021 [CORE] Configure search plugin in `docusaurus.config.ts` themes array
- [ ] T022 [CORE] Create `frontend/src/css/custom.css` with typography and spacing styles
- [ ] T023 [CORE] Verify: `npm run build` succeeds with 0 errors and warnings
- [ ] T024 [CORE] Verify: All 6 chapters accessible via sidebar navigation
- [ ] T025 [CORE] Verify: Search (Ctrl+K) finds placeholder content
- [ ] T026 [CORE] Verify: Site renders correctly at 375px mobile width

**Checkpoint**: Static textbook shell complete

---

## Phase 2: Chapter Content Creation

**Purpose**: Write all 6 chapters with educational content

### Chapter 1: Introduction to Physical AI (~3500 words)
- [ ] T027 [P] [CORE] Write section: What is Physical AI? (definition, examples)
- [ ] T028 [P] [CORE] Write section: Embodied Intelligence vs. Digital AI (comparison table)
- [ ] T029 [P] [CORE] Write section: Applications (Manufacturing, Healthcare, Service Robots)
- [ ] T030 [P] [CORE] Write section: Key Challenges (Perception, Planning, Control)
- [ ] T031 [P] [CORE] Write section: Role of Simulation in Physical AI
- [ ] T032 [CORE] Create diagram: Physical AI system architecture (SVG)
- [ ] T033 [CORE] Create diagram: Embodied vs Digital AI comparison (SVG)
- [ ] T034 [CORE] Write exercise: Identify 3 Physical AI systems in daily life
- [ ] T035 [CORE] Write chapter summary and learning objectives

### Chapter 2: Basics of Humanoid Robotics (~4000 words)
- [ ] T036 [P] [CORE] Write section: Anatomy of a humanoid robot (overview)
- [ ] T037 [P] [CORE] Write section: Degrees of Freedom (DoF) and kinematics (with formulas)
- [ ] T038 [P] [CORE] Write section: Actuators (Motors, servos, hydraulics comparison)
- [ ] T039 [P] [CORE] Write section: Sensors (IMU, force/torque, cameras)
- [ ] T040 [P] [CORE] Write section: URDF - Unified Robot Description Format (with XML example)
- [ ] T041 [CORE] Create diagram: Humanoid robot anatomy labeled (SVG)
- [ ] T042 [CORE] Create diagram: 6-DoF arm joint configuration (SVG)
- [ ] T043 [CORE] Add code example: Simple URDF robot arm definition (XML)
- [ ] T044 [CORE] Write exercise: Sketch a 6-DoF arm and label joints
- [ ] T045 [CORE] Write chapter summary and learning objectives

### Chapter 3: ROS 2 Fundamentals (~4500 words)
- [ ] T046 [P] [CORE] Write section: Why ROS 2? Architecture overview
- [ ] T047 [P] [CORE] Write section: Nodes, Topics, Services, Actions (with diagrams)
- [ ] T048 [P] [CORE] Write section: Launch files and parameters (with examples)
- [ ] T049 [P] [CORE] Write section: tf2 - Coordinate transforms
- [ ] T050 [P] [CORE] Write section: Visualization with RViz2
- [ ] T051 [CORE] Create diagram: ROS 2 communication patterns (SVG)
- [ ] T052 [CORE] Create diagram: tf2 coordinate frame tree (SVG)
- [ ] T053 [CORE] Add code example: Minimal publisher node (Python)
- [ ] T054 [CORE] Add code example: Minimal subscriber node (Python)
- [ ] T055 [CORE] Add code example: Launch file with parameters (Python)
- [ ] T056 [CORE] Write exercise: Create a simple publisher-subscriber pair
- [ ] T057 [CORE] Write chapter summary and learning objectives

### Chapter 4: Digital Twin Simulation (~4000 words)
- [ ] T058 [P] [CORE] Write section: What is a Digital Twin? (definition, benefits)
- [ ] T059 [P] [CORE] Write section: Gazebo setup and first simulation
- [ ] T060 [P] [CORE] Write section: Isaac Sim - NVIDIA's simulation platform
- [ ] T061 [P] [CORE] Write section: Importing URDF into simulators
- [ ] T062 [P] [CORE] Write section: Sensor simulation (Camera, LiDAR, IMU)
- [ ] T063 [CORE] Create diagram: Digital Twin architecture (SVG)
- [ ] T064 [CORE] Create diagram: Gazebo/Isaac Sim comparison table (SVG or MDX table)
- [ ] T065 [CORE] Add code example: Gazebo world file (SDF/XML)
- [ ] T066 [CORE] Add code example: Spawning robot in Gazebo (launch file)
- [ ] T067 [CORE] Write exercise: Spawn a robot in Gazebo and move it
- [ ] T068 [CORE] Write chapter summary and learning objectives

### Chapter 5: Vision-Language-Action Systems (~3500 words)
- [ ] T069 [P] [CORE] Write section: Multimodal AI for robotics (overview)
- [ ] T070 [P] [CORE] Write section: Vision - Object detection, segmentation
- [ ] T071 [P] [CORE] Write section: Language - Instruction following, grounding
- [ ] T072 [P] [CORE] Write section: Action - Policy learning basics
- [ ] T073 [P] [CORE] Write section: Open-source VLA models overview (RT-2, OpenVLA)
- [ ] T074 [CORE] Create diagram: VLA pipeline architecture (SVG)
- [ ] T075 [CORE] Create diagram: Vision-Language grounding example (SVG)
- [ ] T076 [CORE] Add code example: Running inference on pre-trained model (Python)
- [ ] T077 [CORE] Write exercise: Run inference on a pre-trained VLA model
- [ ] T078 [CORE] Write chapter summary and learning objectives

### Chapter 6: Capstone - Simple AI-Robot Pipeline (~4000 words)
- [ ] T079 [P] [CORE] Write section: Project overview - Voice-commanded pick-and-place
- [ ] T080 [P] [CORE] Write section: System architecture diagram and explanation
- [ ] T081 [P] [CORE] Write section: Step 1 - Speech-to-text input (Whisper API)
- [ ] T082 [P] [CORE] Write section: Step 2 - LLM for intent parsing
- [ ] T083 [P] [CORE] Write section: Step 3 - Motion planning (MoveIt2 basics)
- [ ] T084 [P] [CORE] Write section: Step 4 - Execution in simulation
- [ ] T085 [CORE] Create diagram: Full pipeline architecture (SVG)
- [ ] T086 [CORE] Create diagram: State machine for pick-and-place (SVG)
- [ ] T087 [CORE] Add code example: Speech-to-text integration (Python)
- [ ] T088 [CORE] Add code example: LLM intent parser (Python)
- [ ] T089 [CORE] Add code example: MoveIt2 basic motion (Python)
- [ ] T090 [CORE] Write exercise: Build and run the complete pipeline
- [ ] T091 [CORE] Write chapter summary and learning objectives

### Glossary & Cross-References
- [ ] T092 [CORE] Create `frontend/docs/glossary.mdx` with 30+ terms
- [ ] T093 [CORE] Add internal links from chapters to glossary terms
- [ ] T094 [CORE] Verify all code examples have syntax highlighting and copy button
- [ ] T095 [CORE] Verify all diagrams render correctly on desktop and mobile

**Checkpoint**: All content complete and verified

---

## Phase 3: RAG Backend Core

**Purpose**: Build FastAPI backend with embedding and retrieval

### Project Structure
- [ ] T096 [CORE] Create `backend/app/__init__.py`
- [ ] T097 [CORE] Create `backend/app/config.py` with Pydantic Settings for env vars
- [ ] T098 [CORE] Create `backend/app/models/__init__.py`
- [ ] T099 [CORE] Create `backend/app/routers/__init__.py`
- [ ] T100 [CORE] Create `backend/app/services/__init__.py`

### Pydantic Schemas
- [ ] T101 [CORE] Create `backend/app/models/schemas.py` with ChatRequest model (message, session_id, context)
- [ ] T102 [CORE] Add ChatResponse model (response, sources, session_id)
- [ ] T103 [CORE] Add Source model (chapter, section, snippet)
- [ ] T104 [CORE] Add ErrorResponse model (error, retry_after)
- [ ] T105 [CORE] Add HealthResponse model (status, qdrant, llm, version)

### Embedding Service
- [ ] T106 [CORE] Create `backend/app/services/embeddings.py`
- [ ] T107 [CORE] Implement `EmbeddingService` class with sentence-transformers (all-MiniLM-L6-v2)
- [ ] T108 [CORE] Add `embed_text(text: str) -> list[float]` method
- [ ] T109 [CORE] Add `embed_batch(texts: list[str]) -> list[list[float]]` method
- [ ] T110 [CORE] Add model lazy loading to avoid startup delay

### Retrieval Service
- [ ] T111 [CORE] Create `backend/app/services/retrieval.py`
- [ ] T112 [CORE] Implement `RetrievalService` class with Qdrant client
- [ ] T113 [CORE] Add `search(query_embedding: list[float], top_k: int = 5) -> list[dict]` method
- [ ] T114 [CORE] Add similarity threshold filtering (0.7 minimum)
- [ ] T115 [CORE] Add metadata extraction (chapter, section, heading_path)

### Generation Service
- [ ] T116 [CORE] Create `backend/app/services/generation.py`
- [ ] T117 [CORE] Implement `GenerationService` class with Groq API client
- [ ] T118 [CORE] Add `generate_response(query: str, context: list[dict]) -> str` method
- [ ] T119 [CORE] Implement RAG prompt template with strict book-only instructions
- [ ] T120 [CORE] Add "not found" response when no relevant context
- [ ] T121 [CORE] Add source citation formatting in response

### Chat Router
- [ ] T122 [CORE] Create `backend/app/routers/chat.py`
- [ ] T123 [CORE] Implement `POST /api/chat` endpoint
- [ ] T124 [CORE] Wire up: embed query → retrieve chunks → generate response
- [ ] T125 [CORE] Return sources array with chapter, section, snippet
- [ ] T126 [CORE] Handle empty retrieval (return "not found" message)

### Main App Configuration
- [ ] T127 [CORE] Update `backend/app/main.py` with router includes
- [ ] T128 [CORE] Add CORS middleware with configurable origins
- [ ] T129 [CORE] Add rate limiting with SlowAPI (10 req/min per IP)
- [ ] T130 [CORE] Add startup event to verify Qdrant connection
- [ ] T131 [CORE] Add `/api/health` endpoint with service status checks

### Backend Tests
- [ ] T132 [P] [CORE] Create `backend/tests/test_embeddings.py` - test embedding generation
- [ ] T133 [P] [CORE] Create `backend/tests/test_retrieval.py` - test Qdrant search (mock)
- [ ] T134 [P] [CORE] Create `backend/tests/test_chat.py` - test full RAG pipeline (integration)
- [ ] T135 [CORE] Verify: `pytest` passes all tests
- [ ] T136 [CORE] Verify: `/api/health` returns `{"status": "healthy"}`

**Checkpoint**: Backend RAG pipeline functional

---

## Phase 4: Content Embedding Pipeline

**Purpose**: Embed textbook content into Qdrant for retrieval

### Embedding Script
- [ ] T137 [CORE] Create `backend/scripts/embed_content.py`
- [ ] T138 [CORE] Implement MDX parser to extract plain text (strip frontmatter, JSX, imports)
- [ ] T139 [CORE] Implement text chunker: 512 tokens, 50-token overlap
- [ ] T140 [CORE] Extract metadata per chunk: chapter_id, section_heading, heading_path
- [ ] T141 [CORE] Generate embeddings using EmbeddingService
- [ ] T142 [CORE] Upload chunks to Qdrant with metadata payload
- [ ] T143 [CORE] Add progress logging (chunks processed, time elapsed)
- [ ] T144 [CORE] Add idempotency: clear collection before re-embedding

### Qdrant Setup
- [ ] T145 [CORE] Create Qdrant Cloud account (free tier)
- [ ] T146 [CORE] Create collection `textbook_chunks` with vector size 384 (MiniLM)
- [ ] T147 [CORE] Configure collection with cosine similarity metric
- [ ] T148 [CORE] Add QDRANT_URL and QDRANT_API_KEY to `.env`

### Run & Validate
- [ ] T149 [CORE] Run `python scripts/embed_content.py` on all chapters
- [ ] T150 [CORE] Verify: Collection contains ~200-300 chunks
- [ ] T151 [CORE] Create `backend/scripts/test_queries.py` with 10 sample queries
- [ ] T152 [CORE] Verify: 9/10 queries return relevant chunks
- [ ] T153 [CORE] Document embedding process in `backend/README.md`

**Checkpoint**: Content indexed and retrievable

---

## Phase 5: Chatbot UI Integration

**Purpose**: Build chat panel component and integrate with backend

### Chat Components
- [ ] T154 [CORE] Create `frontend/src/components/ChatPanel/index.tsx` - main container
- [ ] T155 [CORE] Create `frontend/src/components/ChatPanel/ChatMessage.tsx` - message bubble
- [ ] T156 [CORE] Create `frontend/src/components/ChatPanel/ChatInput.tsx` - input with send button
- [ ] T157 [CORE] Create `frontend/src/components/ChatPanel/SourceCitation.tsx` - clickable source link
- [ ] T158 [CORE] Create `frontend/src/components/ChatPanel/TypingIndicator.tsx` - loading dots

### Chat State Management
- [ ] T159 [CORE] Create `frontend/src/hooks/useChat.ts`
- [ ] T160 [CORE] Implement state: messages array, isLoading, error
- [ ] T161 [CORE] Implement `sendMessage(text: string)` function
- [ ] T162 [CORE] Add API call to backend `/api/chat` endpoint
- [ ] T163 [CORE] Add sessionStorage persistence for chat history
- [ ] T164 [CORE] Add session_id generation (UUID v4)

### Theme Integration
- [ ] T165 [CORE] Create `frontend/src/theme/Root.tsx` to wrap app with ChatPanel
- [ ] T166 [CORE] Add floating chat toggle button (bottom-right corner)
- [ ] T167 [CORE] Implement slide-in animation for desktop (right side panel)
- [ ] T168 [CORE] Implement full-screen overlay for mobile (<768px)
- [ ] T169 [CORE] Add keyboard shortcut: Cmd/Ctrl+Shift+C to toggle chat

### Styling
- [ ] T170 [CORE] Add ChatPanel styles to `frontend/src/css/custom.css`
- [ ] T171 [CORE] Style for light mode compatibility
- [ ] T172 [CORE] Style for dark mode compatibility
- [ ] T173 [CORE] Add responsive breakpoints for mobile

### Error Handling
- [ ] T174 [CORE] Display error message on API failure
- [ ] T175 [CORE] Display "Backend unavailable" message with retry button
- [ ] T176 [CORE] Display rate limit message with countdown
- [ ] T177 [CORE] Add timeout handling (10 second limit)

### Environment Configuration
- [ ] T178 [CORE] Add `REACT_APP_API_URL` env variable support
- [ ] T179 [CORE] Create `frontend/.env.example` with API URL placeholder
- [ ] T180 [CORE] Configure env variable in `docusaurus.config.ts`

### Verification
- [ ] T181 [CORE] Verify: Chat opens/closes smoothly on desktop
- [ ] T182 [CORE] Verify: Chat opens/closes smoothly on mobile
- [ ] T183 [CORE] Verify: Questions receive answers within 3 seconds
- [ ] T184 [CORE] Verify: Source citations display and link correctly
- [ ] T185 [CORE] Verify: Chat history persists on page navigation

**Checkpoint**: Chatbot UI fully functional

---

## Phase 6: Select-to-Ask Feature

**Purpose**: Enable text selection to ask AI for explanation

- [ ] T186 [CORE] Create `frontend/src/components/SelectToAsk/index.tsx`
- [ ] T187 [CORE] Add `mouseup` event listener to detect text selection
- [ ] T188 [CORE] Calculate tooltip position near selection
- [ ] T189 [CORE] Show "Ask AI" button tooltip on valid selection (>10 chars)
- [ ] T190 [CORE] Hide tooltip on click outside or new selection
- [ ] T191 [CORE] On "Ask AI" click: open ChatPanel with pre-filled query "Explain: [text]"
- [ ] T192 [CORE] Truncate selected text at 500 chars with "..." suffix
- [ ] T193 [CORE] Add SelectToAsk to Root.tsx wrapper
- [ ] T194 [CORE] Style tooltip for light/dark mode
- [ ] T195 [OPT] Add mobile long-press support (touchstart/touchend)
- [ ] T196 [CORE] Verify: Selecting text shows tooltip
- [ ] T197 [CORE] Verify: Clicking "Ask AI" opens chat with correct query

**Checkpoint**: Select-to-ask feature complete

---

## Phase 7: Deployment Pipeline

**Purpose**: Deploy frontend to GitHub Pages, backend to free tier

### GitHub Pages (Frontend)
- [ ] T198 [CORE] Create `.github/workflows/deploy-frontend.yml`
- [ ] T199 [CORE] Configure workflow trigger on push to `main`
- [ ] T200 [CORE] Add steps: checkout, setup Node, install deps, build, deploy
- [ ] T201 [CORE] Configure GitHub Pages in repo settings (gh-pages branch)
- [ ] T202 [CORE] Update `docusaurus.config.ts` with correct baseUrl for GitHub Pages
- [ ] T203 [CORE] Add production API URL to build environment

### Backend Deployment (Vercel)
- [ ] T204 [CORE] Create `backend/vercel.json` with Python runtime config
- [ ] T205 [CORE] Create Vercel account and link repository
- [ ] T206 [CORE] Configure environment variables in Vercel dashboard
- [ ] T207 [CORE] Deploy backend to Vercel
- [ ] T208 [CORE] Update CORS_ORIGINS with GitHub Pages domain

### Alternative: Railway Deployment
- [ ] T209 [OPT] Create `backend/Procfile` for Railway
- [ ] T210 [OPT] Create Railway account and link repository
- [ ] T211 [OPT] Configure environment variables in Railway dashboard

### Alternative: Render Deployment
- [ ] T212 [OPT] Create `backend/render.yaml` for Render
- [ ] T213 [OPT] Create Render account and link repository
- [ ] T214 [OPT] Configure environment variables in Render dashboard

### Verification
- [ ] T215 [CORE] Verify: Push to `main` triggers frontend deployment
- [ ] T216 [CORE] Verify: Frontend loads at GitHub Pages URL
- [ ] T217 [CORE] Verify: Backend `/api/health` returns healthy at production URL
- [ ] T218 [CORE] Verify: Chat works end-to-end on production site

**Checkpoint**: Production deployment complete

---

## Phase 8: Testing & Polish

**Purpose**: Ensure quality and fix edge cases

### Lighthouse Audit
- [ ] T219 [CORE] Run Lighthouse on homepage
- [ ] T220 [CORE] Run Lighthouse on Chapter 1 page
- [ ] T221 [CORE] Run Lighthouse on Chapter 3 page (code-heavy)
- [ ] T222 [CORE] Fix any accessibility issues (target: score >90)
- [ ] T223 [CORE] Fix any performance issues (target: score >90)

### RAG Validation
- [ ] T224 [CORE] Test query: "What is Physical AI?" → expect Chapter 1 citation
- [ ] T225 [CORE] Test query: "What is ROS 2?" → expect Chapter 3 citation
- [ ] T226 [CORE] Test query: "Explain URDF" → expect Chapter 2 citation
- [ ] T227 [CORE] Test query: "How do I use Gazebo?" → expect Chapter 4 citation
- [ ] T228 [CORE] Test query: "What is VLA?" → expect Chapter 5 citation
- [ ] T229 [CORE] Test query: "How does the capstone work?" → expect Chapter 6 citation
- [ ] T230 [CORE] Test query: "What is tf2?" → expect Chapter 3 citation
- [ ] T231 [CORE] Test query: "Explain degrees of freedom" → expect Chapter 2 citation
- [ ] T232 [CORE] Test query: "What is Isaac Sim?" → expect Chapter 4 citation
- [ ] T233 [CORE] Test query: "Explain motion planning" → expect Chapter 6 citation
- [ ] T234 [CORE] Test out-of-scope: "What is quantum computing?" → expect "not found"
- [ ] T235 [CORE] Test out-of-scope: "Who is the president?" → expect "not found"
- [ ] T236 [CORE] Test out-of-scope: "Write me Python code" → expect "not found"
- [ ] T237 [CORE] Test out-of-scope: "What is machine learning?" → expect "not found"
- [ ] T238 [CORE] Test out-of-scope: "Tell me a joke" → expect "not found"

### Mobile Testing
- [ ] T239 [CORE] Test on mobile viewport (375px width)
- [ ] T240 [CORE] Test sidebar navigation on mobile
- [ ] T241 [CORE] Test chat panel on mobile (full-screen overlay)
- [ ] T242 [CORE] Test code block scrolling on mobile

### Documentation
- [ ] T243 [CORE] Update root `README.md` with project overview
- [ ] T244 [CORE] Add local development setup instructions to README
- [ ] T245 [CORE] Add deployment instructions to README
- [ ] T246 [CORE] Add environment variable documentation
- [ ] T247 [OPT] Create `CONTRIBUTING.md` with contribution guidelines

**Checkpoint**: Production-ready quality

---

## Phase 9: Optional Enhancements

**Purpose**: Add P3 features if time permits

### Urdu Translation (P3)
- [ ] T248 [OPT] Configure Docusaurus i18n in `docusaurus.config.ts`
- [ ] T249 [OPT] Create `frontend/i18n/ur/` directory structure
- [ ] T250 [OPT] Translate Chapter 1 to Urdu
- [ ] T251 [OPT] Add language toggle component to navbar
- [ ] T252 [OPT] Configure RTL layout for Urdu
- [ ] T253 [OPT] Embed Urdu content separately in Qdrant
- [ ] T254 [OPT] Add language parameter to chat API

### Personalization (P3)
- [ ] T255 [OPT] Create `frontend/src/hooks/useBookmarks.ts` with localStorage
- [ ] T256 [OPT] Add bookmark button to each section heading
- [ ] T257 [OPT] Create "My Bookmarks" page listing saved sections
- [ ] T258 [OPT] Create `frontend/src/hooks/useNotes.ts` with localStorage
- [ ] T259 [OPT] Add inline note editor component
- [ ] T260 [OPT] Create "My Notes" page with all saved notes

**Checkpoint**: Optional features complete

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 0 (Setup)
    │
    ▼
Phase 1 (Frontend Shell)
    │
    ├────────────────────────┐
    │                        │
    ▼                        ▼
Phase 2 (Content)      Phase 3 (Backend)
    │                        │
    └───────────┬────────────┘
                │
                ▼
          Phase 4 (Embedding)
                │
                ▼
          Phase 5 (Chat UI)
                │
                ▼
          Phase 6 (Select-to-Ask)
                │
                ▼
          Phase 7 (Deployment)
                │
                ▼
          Phase 8 (Testing)
                │
                ▼
          Phase 9 (Optional)
```

### Parallel Opportunities

| Tasks | Can Run In Parallel |
|-------|---------------------|
| T005, T006 | Backend init while frontend inits |
| T013-T018 | All 6 chapter placeholders |
| T027-T031 | Chapter 1 sections |
| T036-T040 | Chapter 2 sections |
| T046-T050 | Chapter 3 sections |
| T058-T062 | Chapter 4 sections |
| T069-T073 | Chapter 5 sections |
| T079-T084 | Chapter 6 sections |
| T132-T134 | Backend tests |
| Phase 2 & Phase 3 | Content & Backend (different directories) |

### Critical Path

1. Phase 0 → Phase 1 → Phase 2 (Content is the bottleneck)
2. Phase 0 → Phase 1 → Phase 3 (Backend can parallel with content)
3. Phase 4 requires both Phase 2 and Phase 3
4. Phases 5-8 are sequential

---

## Task Summary

| Phase | Total Tasks | Core | Optional |
|-------|-------------|------|----------|
| Phase 0: Setup | 10 | 10 | 0 |
| Phase 1: Frontend Shell | 16 | 16 | 0 |
| Phase 2: Content | 69 | 69 | 0 |
| Phase 3: Backend | 41 | 41 | 0 |
| Phase 4: Embedding | 17 | 17 | 0 |
| Phase 5: Chat UI | 32 | 32 | 0 |
| Phase 6: Select-to-Ask | 12 | 11 | 1 |
| Phase 7: Deployment | 21 | 15 | 6 |
| Phase 8: Testing | 29 | 28 | 1 |
| Phase 9: Optional | 13 | 0 | 13 |
| **Total** | **260** | **239** | **21** |

---

## Quick Reference: File Paths

### Frontend
- `frontend/docusaurus.config.ts` - Main configuration
- `frontend/sidebars.ts` - Sidebar configuration
- `frontend/docs/` - Chapter MDX files
- `frontend/src/components/` - React components
- `frontend/src/hooks/` - Custom hooks
- `frontend/src/css/custom.css` - Global styles
- `frontend/src/theme/Root.tsx` - Theme wrapper

### Backend
- `backend/app/main.py` - FastAPI entry point
- `backend/app/config.py` - Environment configuration
- `backend/app/models/schemas.py` - Pydantic models
- `backend/app/routers/chat.py` - Chat endpoint
- `backend/app/services/embeddings.py` - Embedding service
- `backend/app/services/retrieval.py` - Qdrant retrieval
- `backend/app/services/generation.py` - LLM generation
- `backend/scripts/embed_content.py` - Embedding pipeline
- `backend/scripts/test_queries.py` - RAG validation

### Deployment
- `.github/workflows/deploy-frontend.yml` - GitHub Pages workflow
- `backend/vercel.json` - Vercel configuration
