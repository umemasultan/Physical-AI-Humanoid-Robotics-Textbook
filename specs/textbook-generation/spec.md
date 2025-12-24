# Feature Specification: Textbook Generation

**Feature Branch**: `feature/textbook-generation`
**Created**: 2025-12-24
**Status**: Draft
**Input**: AI-native textbook with RAG chatbot for Physical AI & Humanoid Robotics course

---

## Overview

Build a complete AI-native textbook using Docusaurus with an integrated RAG chatbot. The textbook covers Physical AI and Humanoid Robotics in 6 focused chapters. The chatbot answers questions strictly from book content using free-tier infrastructure.

---

## User Scenarios & Testing

### User Story 1 - Read Textbook Content (Priority: P1)

A learner visits the textbook website to read educational content about Physical AI and Humanoid Robotics. They navigate through chapters using the sidebar, read content with code examples, and view diagrams.

**Why this priority**: Core value proposition—without readable content, nothing else matters.

**Independent Test**: Deploy static site with all 6 chapters; user can navigate and read all content without any backend.

**Acceptance Scenarios**:

1. **Given** a learner on the homepage, **When** they click a chapter in the sidebar, **Then** the chapter content loads in <2 seconds
2. **Given** a learner reading Chapter 3 (ROS 2), **When** they view a code example, **Then** syntax highlighting is applied and code is copy-able
3. **Given** a learner on mobile device, **When** they access any chapter, **Then** content is responsive and readable without horizontal scrolling
4. **Given** a learner on Chapter 4, **When** they click "Next" at bottom, **Then** they navigate to Chapter 5

---

### User Story 2 - Ask AI Chatbot (Priority: P1)

A learner has a question about content they just read. They open the chatbot panel, type their question, and receive an answer sourced from the book with citations.

**Why this priority**: Core differentiator—the RAG chatbot is the "AI-native" feature that sets this apart from static textbooks.

**Independent Test**: With backend running, ask "What is Physical AI?" and receive answer citing Chapter 1.

**Acceptance Scenarios**:

1. **Given** a learner on any page, **When** they click the chat icon, **Then** a chat panel opens within 500ms
2. **Given** a learner asks "What is ROS 2?", **When** the RAG pipeline processes, **Then** response appears in <3 seconds with citation "Chapter 3, Section X"
3. **Given** a learner asks "What is quantum computing?", **When** content is not in book, **Then** response is "I couldn't find information about that in the book"
4. **Given** a learner asks a follow-up question, **When** submitted, **Then** context from previous Q&A is preserved in the session

---

### User Story 3 - Select Text to Ask AI (Priority: P2)

A learner reads a complex paragraph and wants clarification. They select the text, click "Ask AI", and the chatbot explains the selected content in simpler terms.

**Why this priority**: Enhanced UX feature that improves learning but not essential for MVP.

**Independent Test**: Select text in Chapter 2, click "Ask AI", receive explanation of selected content.

**Acceptance Scenarios**:

1. **Given** a learner selects text on any page, **When** selection completes, **Then** a tooltip with "Ask AI" button appears near selection
2. **Given** a learner clicks "Ask AI" on selected text, **When** processed, **Then** chatbot opens with pre-filled context "Explain: [selected text]"
3. **Given** selected text is >500 characters, **When** "Ask AI" clicked, **Then** text is truncated with "..." and full context sent to backend

---

### User Story 4 - Search Textbook (Priority: P2)

A learner wants to find all mentions of "URDF" across the textbook. They use the search bar to find relevant sections quickly.

**Why this priority**: Standard textbook feature; Docusaurus provides this out-of-box.

**Independent Test**: Type "URDF" in search, see results from Chapter 2 and Chapter 4.

**Acceptance Scenarios**:

1. **Given** a learner on any page, **When** they press Ctrl+K or click search, **Then** search modal opens
2. **Given** a learner types "Gazebo", **When** results load, **Then** matching sections from Chapter 4 appear with highlighted snippets
3. **Given** a learner clicks a search result, **When** navigated, **Then** page scrolls to the matching section with highlight

---

### User Story 5 - View on Mobile (Priority: P2)

A learner accesses the textbook on their phone during commute. All features work on mobile with touch-friendly interactions.

**Why this priority**: Significant user segment; Docusaurus handles this but chat UI needs attention.

**Independent Test**: Access site on mobile viewport (375px width), navigate chapters, use chatbot.

**Acceptance Scenarios**:

1. **Given** a learner on mobile, **When** they tap hamburger menu, **Then** sidebar slides in with all chapters
2. **Given** a learner on mobile, **When** they open chatbot, **Then** chat panel is full-screen overlay
3. **Given** a learner on mobile, **When** they long-press text, **Then** "Ask AI" option appears in context menu

---

### User Story 6 - Urdu Translation Toggle (Priority: P3 - Optional)

A learner prefers reading in Urdu. They toggle language to Urdu and content displays in Urdu.

**Why this priority**: Nice-to-have for accessibility; requires significant content effort.

**Independent Test**: Toggle to Urdu, Chapter 1 displays in Urdu.

**Acceptance Scenarios**:

1. **Given** a learner on any page, **When** they click language toggle, **Then** options show "English" and "اردو"
2. **Given** a learner selects Urdu, **When** page reloads, **Then** content displays in Urdu with RTL layout
3. **Given** a learner asks chatbot in Urdu, **When** processed, **Then** response is in Urdu citing translated content

---

### User Story 7 - Personalized Chapter (Priority: P3 - Optional)

A learner can add personal notes to chapters and bookmark sections for later review.

**Why this priority**: Enhancement feature; requires user accounts or local storage.

**Independent Test**: Add note to Chapter 1, refresh page, note persists.

**Acceptance Scenarios**:

1. **Given** a learner on any section, **When** they click "Add Note", **Then** a note editor appears inline
2. **Given** a learner saves a note, **When** they revisit the section, **Then** note is visible with timestamp
3. **Given** a learner clicks "My Bookmarks", **When** panel opens, **Then** all bookmarked sections are listed with links

---

### Edge Cases

- **Empty search results**: Display "No results found for '[query]'" with suggestion to try different keywords
- **Chatbot backend unavailable**: Show graceful error "AI assistant is temporarily unavailable. Please try again later." with retry button
- **Very long chat response**: Truncate at 2000 characters with "Show more" expansion
- **Concurrent chat requests**: Queue requests; show typing indicator; process sequentially
- **Network timeout on chat**: After 10 seconds, show "Request timed out. Please try again."
- **Malformed user input**: Sanitize all inputs; reject if >1000 characters
- **Code block copy fails**: Fallback to browser clipboard API; show success/error toast

---

## Requirements

### Functional Requirements - Frontend

- **FR-001**: System MUST render 6 chapters as static HTML pages via Docusaurus
- **FR-002**: System MUST auto-generate sidebar navigation from folder structure
- **FR-003**: System MUST provide syntax highlighting for code blocks (Python, YAML, XML, Bash)
- **FR-004**: System MUST include copy button on all code blocks
- **FR-005**: System MUST be fully responsive (mobile, tablet, desktop)
- **FR-006**: System MUST provide full-text search via Docusaurus local search plugin
- **FR-007**: System MUST display chatbot as slide-in panel (desktop) or full-screen (mobile)
- **FR-008**: System MUST show typing indicator while waiting for chatbot response
- **FR-009**: System MUST persist chat history in session storage (cleared on tab close)
- **FR-010**: System MUST provide "Select text → Ask AI" tooltip functionality

### Functional Requirements - Backend (RAG)

- **FR-011**: System MUST embed all textbook content into vector database on build
- **FR-012**: System MUST chunk content at 512 tokens with 50-token overlap
- **FR-013**: System MUST store chunk metadata (chapter, section, heading hierarchy)
- **FR-014**: System MUST retrieve top-5 relevant chunks for each query
- **FR-015**: System MUST apply 0.7 similarity threshold; return "not found" if below
- **FR-016**: System MUST include source citation in every response
- **FR-017**: System MUST NOT answer questions outside book content
- **FR-018**: System MUST rate-limit API to 10 requests/minute per IP
- **FR-019**: System MUST log all queries (anonymized) for analytics
- **FR-020**: System MUST respond within 3 seconds for 95th percentile

### Functional Requirements - Content

- **FR-021**: Each chapter MUST have: learning objectives, prerequisites, content, exercise, summary
- **FR-022**: All code examples MUST be tested and runnable
- **FR-023**: Each chapter MUST be 10-15 pages (approximately 3000-5000 words)
- **FR-024**: System MUST include glossary with linked terms
- **FR-025**: System MUST include diagrams for complex concepts (minimum 2 per chapter)

### Non-Functional Requirements

- **NFR-001**: Page load time MUST be <2 seconds (First Contentful Paint)
- **NFR-002**: Lighthouse accessibility score MUST be >90
- **NFR-003**: Bundle size MUST be <200KB gzipped (excluding images)
- **NFR-004**: System MUST work on latest Chrome, Firefox, Safari, Edge
- **NFR-005**: System MUST deploy to GitHub Pages with zero configuration
- **NFR-006**: Infrastructure cost MUST be $0/month (free tiers only)
- **NFR-007**: System MUST handle 100 concurrent users without degradation

---

## Key Entities

### Chapter
- **Attributes**: id, title, slug, order, content (MDX), learning_objectives[], prerequisites[], exercises[], summary
- **Relationships**: Contains multiple Sections; belongs to Book

### Section
- **Attributes**: id, heading, level (h2/h3), content, code_examples[], diagrams[]
- **Relationships**: Belongs to Chapter; indexed for search and RAG

### ChatMessage
- **Attributes**: id, session_id, role (user/assistant), content, timestamp, sources[]
- **Relationships**: Belongs to ChatSession

### ContentChunk (for RAG)
- **Attributes**: id, text, embedding[], chapter_id, section_id, heading_path, token_count
- **Relationships**: References Chapter and Section; stored in Qdrant

### UserQuery (for analytics)
- **Attributes**: id, query_text, timestamp, response_time_ms, chunks_retrieved, ip_hash
- **Relationships**: None (anonymized logs)

---

## Book Structure

### Chapter 1: Introduction to Physical AI
- What is Physical AI?
- Embodied Intelligence vs. Digital AI
- Applications: Manufacturing, Healthcare, Service Robots
- Key challenges: Perception, Planning, Control
- The role of simulation in Physical AI
- **Exercise**: Identify 3 Physical AI systems in daily life

### Chapter 2: Basics of Humanoid Robotics
- Anatomy of a humanoid robot
- Degrees of Freedom (DoF) and kinematics
- Actuators: Motors, servos, hydraulics
- Sensors: IMU, force/torque, cameras
- URDF: Unified Robot Description Format
- **Exercise**: Sketch a 6-DoF arm and label joints

### Chapter 3: ROS 2 Fundamentals
- Why ROS 2? Architecture overview
- Nodes, Topics, Services, Actions
- Launch files and parameters
- tf2: Coordinate transforms
- Visualization with RViz2
- **Exercise**: Create a simple publisher-subscriber pair

### Chapter 4: Digital Twin Simulation
- What is a Digital Twin?
- Gazebo: Setup and first simulation
- Isaac Sim: NVIDIA's simulation platform
- Importing URDF into simulators
- Sensor simulation: Camera, LiDAR, IMU
- **Exercise**: Spawn a robot in Gazebo and move it

### Chapter 5: Vision-Language-Action (VLA) Systems
- Multimodal AI for robotics
- Vision: Object detection, segmentation
- Language: Instruction following, grounding
- Action: Policy learning basics
- Open-source VLA models overview
- **Exercise**: Run inference on a pre-trained VLA model

### Chapter 6: Capstone - Simple AI-Robot Pipeline
- Project overview: Voice-commanded pick-and-place
- System architecture diagram
- Step 1: Speech-to-text input
- Step 2: LLM for intent parsing
- Step 3: Motion planning
- Step 4: Execution in simulation
- **Exercise**: Build and run the complete pipeline

---

## Technical Architecture

### Frontend Stack
```
Docusaurus 3.x
├── docs/           # MDX content (6 chapters)
├── src/
│   ├── components/
│   │   ├── ChatPanel.tsx      # Chatbot UI
│   │   ├── ChatMessage.tsx    # Message bubble
│   │   ├── SelectToAsk.tsx    # Text selection tooltip
│   │   └── CodeBlock.tsx      # Enhanced code block
│   ├── hooks/
│   │   └── useChat.ts         # Chat state management
│   └── theme/                 # Docusaurus theme overrides
├── static/         # Images, diagrams
└── docusaurus.config.ts
```

### Backend Stack
```
FastAPI
├── app/
│   ├── main.py              # FastAPI app entry
│   ├── routers/
│   │   └── chat.py          # /api/chat endpoint
│   ├── services/
│   │   ├── embeddings.py    # Sentence Transformers
│   │   ├── retrieval.py     # Qdrant search
│   │   └── generation.py    # LLM response
│   ├── models/
│   │   └── schemas.py       # Pydantic models
│   └── config.py            # Environment config
├── scripts/
│   └── embed_content.py     # Build-time embedding
└── requirements.txt
```

### Infrastructure
```
┌─────────────────────────────────────────────────────────────┐
│                      GitHub Pages                           │
│                   (Static Docusaurus)                       │
└─────────────────────┬───────────────────────────────────────┘
                      │ HTTPS
                      ▼
┌─────────────────────────────────────────────────────────────┐
│              Vercel/Railway (Free Tier)                     │
│                    FastAPI Backend                          │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │ /api/chat   │→ │ Embeddings  │→ │ Qdrant Cloud (Free) │ │
│  └─────────────┘  └─────────────┘  └─────────────────────┘ │
│         │                                                   │
│         ▼                                                   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │        LLM API (Groq/OpenRouter Free Tier)          │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

---

## API Contract

### POST /api/chat

**Request**:
```json
{
  "message": "What is ROS 2?",
  "session_id": "uuid-v4",
  "context": []  // Previous messages for multi-turn
}
```

**Response (Success)**:
```json
{
  "response": "ROS 2 (Robot Operating System 2) is...",
  "sources": [
    {
      "chapter": 3,
      "section": "Why ROS 2?",
      "snippet": "ROS 2 is the next generation..."
    }
  ],
  "session_id": "uuid-v4"
}
```

**Response (Not Found)**:
```json
{
  "response": "I couldn't find information about that in the book.",
  "sources": [],
  "session_id": "uuid-v4"
}
```

**Response (Error)**:
```json
{
  "error": "Rate limit exceeded. Please try again in 60 seconds.",
  "retry_after": 60
}
```

### GET /api/health

**Response**:
```json
{
  "status": "healthy",
  "qdrant": "connected",
  "llm": "available",
  "version": "1.0.0"
}
```

---

## Success Criteria

### Measurable Outcomes

- **SC-001**: All 6 chapters render correctly with `npm run build` exit code 0
- **SC-002**: Lighthouse performance score >90 on all pages
- **SC-003**: Lighthouse accessibility score >90 on all pages
- **SC-004**: Chatbot responds to 90% of in-book questions with correct source citation
- **SC-005**: Chatbot returns "not found" for 100% of out-of-book questions tested
- **SC-006**: RAG response time <3 seconds for 95th percentile
- **SC-007**: Site loads and functions on mobile (375px viewport)
- **SC-008**: GitHub Pages deployment succeeds via GitHub Actions
- **SC-009**: Total infrastructure cost = $0/month
- **SC-010**: All code examples in book execute without errors

### Definition of Done

- [ ] All 6 chapters written and reviewed
- [ ] Docusaurus site builds without warnings
- [ ] Chatbot functional with RAG pipeline
- [ ] Select-text-to-ask feature working
- [ ] Mobile responsive verified
- [ ] Lighthouse scores >90
- [ ] GitHub Pages deployment automated
- [ ] README with setup instructions
- [ ] 10 test queries verified against chatbot

---

## Out of Scope (v1.0)

- User authentication/accounts
- Progress tracking/completion status
- Quiz/assessment system
- PDF/EPUB export
- Comments/discussions
- Multi-language beyond English (Urdu is P3 optional)
- Offline mode/PWA
- Analytics dashboard

---

## Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Free-tier LLM rate limits | Chat unavailable | Queue requests; show helpful error; cache common Q&A |
| Qdrant free tier storage limit | Can't embed all content | Aggressive chunking; prioritize key sections |
| Hallucination despite RAG | Wrong answers | Strict similarity threshold; prompt engineering; testing |
| Slow RAG response | Poor UX | Streaming responses; optimize embeddings; caching |
| GitHub Pages build fails | No deployment | Local build verification; incremental changes |

---

## Dependencies

### External Services (Free Tier)
- **Qdrant Cloud**: Vector storage (1GB free)
- **Neon PostgreSQL**: Query logging (0.5GB free) - *optional, can use SQLite*
- **Groq/OpenRouter**: LLM inference (free tier or pay-per-token)
- **GitHub Pages**: Static hosting (unlimited for public repos)
- **Vercel/Railway**: Backend hosting (free tier)

### Libraries
- **Frontend**: Docusaurus 3.x, React 18, TypeScript
- **Backend**: FastAPI, sentence-transformers, qdrant-client, httpx
- **Embeddings**: all-MiniLM-L6-v2 (22M params, 80MB)

---

## Appendix: Content Guidelines

### Code Example Format
```python
# Example: ROS 2 Publisher
# File: simple_publisher.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        # ... rest of implementation

# Output: [INFO] Publishing: "Hello World: 0"
```

### Diagram Requirements
- Format: SVG preferred, PNG acceptable
- Max dimensions: 800x600px
- Alt text required for accessibility
- Source files (e.g., draw.io) stored in `/static/diagrams/src/`

### Glossary Entry Format
```markdown
**URDF** (Unified Robot Description Format): An XML format for describing robot
kinematics, dynamics, and visual properties. Used by ROS and most simulators.
See: Chapter 2, Section "URDF".
```
