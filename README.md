# Physical AI & Humanoid Robotics

[![Docusaurus](https://img.shields.io/badge/Docusaurus-3.x-blue?logo=docusaurus)](https://docusaurus.io/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Deploy](https://img.shields.io/badge/Deploy-Vercel-black?logo=vercel)](https://vercel.com)
[![AI Chatbot](https://img.shields.io/badge/AI-Chatbot-purple?logo=openai)](https://physicalairumanoidroboticstextbook.vercel.app)

A comprehensive open-source textbook for building intelligent embodied systems. From ROS 2 middleware to reinforcement learning locomotion, this resource covers the complete Physical AI stack for humanoid robotics.

**Features an AI-powered chatbot** that answers questions about the textbook content!

[**Read the Textbook →**](https://physicalairumanoidroboticstextbook.vercel.app)

---

## Overview

Physical AI represents the convergence of machine learning, robotics, and embodied cognition—intelligence that doesn't just think, but acts in the physical world. This textbook provides a structured introduction to building humanoid robots that perceive, reason, and move.

**Key Features:**

- 📚 **6 comprehensive modules** covering the full humanoid robotics stack
- 🤖 **AI Chatbot** - Ask questions about any topic in the textbook
- 🎓 **University-level content** with practical, hands-on approach
- 🌙 **Dark mode support** for comfortable reading
- 📱 **Fully responsive** design for all devices
- 🔍 **RAG-powered** retrieval for accurate answers

---

## Modules

| # | Module | Topics |
|---|--------|--------|
| 1 | **The Robotic Nervous System (ROS 2)** | Nodes, topics, services, actions, URDF |
| 2 | **The Digital Twin (Gazebo & Unity)** | Physics simulation, sensor modeling, domain randomization |
| 3 | **The AI-Robot Brain (NVIDIA Isaac)** | GPU acceleration, synthetic data, visual SLAM |
| 4 | **Vision-Language-Action (VLA)** | Whisper, LLM planning, visual grounding |
| 5 | **Sensor Fusion & State Estimation** | Kalman filtering, VIO, contact estimation |
| 6 | **Reinforcement Learning for Locomotion** | Reward design, sim-to-real transfer, deployment |

Each module includes:
- Learning objectives
- Conceptual explanations
- Practical code examples
- Capstone projects
- Key takeaways

---

## Tech Stack

| Component | Technology |
|-----------|------------|
| Frontend | [Docusaurus 3.x](https://docusaurus.io/), React 18 |
| Backend | FastAPI, Python 3.11 |
| AI/ML | Groq LLM, Sentence Transformers |
| Vector DB | Qdrant Cloud |
| Frontend Hosting | Vercel (free) |
| Backend Hosting | Render (free) |

---

## Quick Start

### Frontend (Docusaurus)

```bash
# Clone the repository
git clone https://github.com/umemasultan/Physical-AI-Humanoid-Robotics-Textbook.git
cd Physical-AI-Humanoid-Robotics-Textbook

# Install dependencies
cd frontend
npm install

# Start development server
npm start
```

Open [http://localhost:3000](http://localhost:3000) to view the textbook.

### Backend (AI Chatbot)

```bash
# Navigate to backend
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env with your API keys (see below)

# Embed textbook content
python scripts/embed_content.py

# Start server
uvicorn app.main:app --reload
```

---

## AI Chatbot Deployment

### Step 1: Get Free API Keys

1. **Qdrant Cloud** (Vector Database)
   - Go to [cloud.qdrant.io](https://cloud.qdrant.io)
   - Create free account → Create cluster
   - Copy: `QDRANT_URL` and `QDRANT_API_KEY`

2. **Groq** (LLM API - Super Fast!)
   - Go to [console.groq.com](https://console.groq.com)
   - Create free account
   - Copy: `GROQ_API_KEY`

### Step 2: Deploy Backend to Render

1. Go to [render.com](https://render.com) → Sign up with GitHub
2. Click **New** → **Web Service**
3. Connect your GitHub repo
4. Configure:
   ```
   Name: physical-ai-chatbot-api
   Root Directory: backend
   Runtime: Python 3
   Build Command: pip install -r requirements.txt
   Start Command: uvicorn app.main:app --host 0.0.0.0 --port $PORT
   ```
5. Add Environment Variables:
   ```
   QDRANT_URL=your-qdrant-url
   QDRANT_API_KEY=your-qdrant-key
   GROQ_API_KEY=your-groq-key
   CORS_ORIGINS=https://physicalairumanoidroboticstextbook.vercel.app
   APP_ENV=production
   ```
6. Click **Create Web Service**

### Step 3: Embed Content (One-time)

After backend is deployed, run locally:
```bash
cd backend
python scripts/embed_content.py
```

### Step 4: Update Frontend API URL

In `frontend/src/components/ChatWidget/ChatWidget.js`, update:
```javascript
const API_URL = 'https://your-backend-name.onrender.com';
```

Push changes → Vercel will auto-deploy!

---

## Project Structure

```
Physical-AI-Humanoid-Robotics-Textbook/
├── frontend/                    # Docusaurus site
│   ├── docs/                    # Markdown content (6 modules)
│   ├── src/
│   │   ├── components/
│   │   │   └── ChatWidget/      # AI Chatbot component
│   │   ├── css/
│   │   └── theme/
│   ├── docusaurus.config.js
│   └── sidebars.js
├── backend/                     # FastAPI RAG service
│   ├── app/
│   │   ├── main.py              # FastAPI app
│   │   ├── routers/chat.py      # Chat endpoint
│   │   ├── services/
│   │   │   ├── rag_service.py   # RAG logic
│   │   │   └── embedding_service.py
│   │   └── models/schemas.py
│   ├── scripts/
│   │   └── embed_content.py     # Embedding script
│   ├── requirements.txt
│   └── render.yaml              # Render deployment config
└── README.md
```

---

## Environment Variables

### Backend (.env)

```env
# Qdrant Cloud (FREE)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-api-key

# Groq LLM (FREE)
GROQ_API_KEY=your-groq-key

# CORS
CORS_ORIGINS=http://localhost:3000,https://your-vercel-url.vercel.app

# App Config
APP_ENV=production
EMBEDDING_MODEL=all-MiniLM-L6-v2
```

---

## Author

**Umema Sultan**

Building the future of Physical AI and Humanoid Robotics.

[![LinkedIn](https://img.shields.io/badge/LinkedIn-Connect-blue?logo=linkedin)](https://www.linkedin.com/in/umema-sultan-385797341/)
[![GitHub](https://img.shields.io/badge/GitHub-Follow-black?logo=github)](https://github.com/umemasultan)
[![Instagram](https://img.shields.io/badge/Instagram-Follow-E4405F?logo=instagram)](https://www.instagram.com/codedreamer123/)

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- [Docusaurus](https://docusaurus.io/) for the documentation framework
- [Qdrant](https://qdrant.tech/) for vector database
- [Groq](https://groq.com/) for lightning-fast LLM inference
- [Render](https://render.com/) for free backend hosting
- [ROS 2](https://docs.ros.org/) community
- [NVIDIA Isaac](https://developer.nvidia.com/isaac-sim) for simulation tools

---

<p align="center">
  <i>The robots are learning to walk. Let's learn to build them.</i>
</p>
