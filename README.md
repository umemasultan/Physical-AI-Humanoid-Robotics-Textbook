# Physical AI & Humanoid Robotics

[![Docusaurus](https://img.shields.io/badge/Docusaurus-3.x-blue?logo=docusaurus)](https://docusaurus.io/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Deploy](https://img.shields.io/badge/Deploy-Vercel-black?logo=vercel)](https://vercel.com)

A comprehensive open-source textbook for building intelligent embodied systems. From ROS 2 middleware to reinforcement learning locomotion, this resource covers the complete Physical AI stack for humanoid robotics.

[**Read the Textbook →**](https://physical-ai-textbook.vercel.app)

---

## Overview

Physical AI represents the convergence of machine learning, robotics, and embodied cognition—intelligence that doesn't just think, but acts in the physical world. This textbook provides a structured introduction to building humanoid robots that perceive, reason, and move.

**Key Features:**

- 📚 **6 comprehensive modules** covering the full humanoid robotics stack
- 🎓 **University-level content** with practical, hands-on approach
- 🤖 **RAG-ready architecture** for AI-powered Q&A integration
- 🌙 **Dark mode support** for comfortable reading
- 📱 **Fully responsive** design for all devices
- 🖨️ **Print-optimized** styles for academic use

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
- Practical examples
- Capstone projects
- Key takeaways

---

## Tech Stack

| Component | Technology |
|-----------|------------|
| Documentation | [Docusaurus 3.x](https://docusaurus.io/) |
| Frontend | React 18, TypeScript |
| Styling | Custom CSS, Dark Mode |
| Hosting | Vercel (free tier) |
| RAG Backend | FastAPI, Qdrant, Groq (optional) |

---

## Quick Start

### Prerequisites

- Node.js 18+
- npm or yarn

### Local Development

```bash
# Clone the repository
git clone https://github.com/umemasultan/physical-ai-textbook.git
cd physical-ai-textbook

# Install dependencies
cd frontend
npm install

# Start development server
npm start
```

Open [http://localhost:3000](http://localhost:3000) to view the textbook.

### Build for Production

```bash
npm run build
```

The static files will be generated in `frontend/build/`.

---

## Deployment (Vercel)

Deploy to Vercel's free tier in minutes:

### Option 1: One-Click Deploy

[![Deploy with Vercel](https://vercel.com/button)](https://vercel.com/new/clone?repository-url=https://github.com/umemasultan/physical-ai-textbook)

### Option 2: Manual Setup

1. **Fork this repository**

2. **Connect to Vercel**
   - Go to [vercel.com](https://vercel.com)
   - Sign in with GitHub
   - Click "New Project"
   - Import your forked repository

3. **Configure Build Settings**
   ```
   Framework Preset: Docusaurus 2
   Root Directory: frontend
   Build Command: npm run build
   Output Directory: build
   ```

4. **Deploy**
   - Click "Deploy"
   - Your site will be live at `https://your-project.vercel.app`

### Custom Domain (Optional)

1. Go to Project Settings → Domains
2. Add your custom domain
3. Configure DNS as instructed

---

## Project Structure

```
physical-ai-textbook/
├── frontend/                    # Docusaurus site
│   ├── docs/                    # Markdown content
│   │   ├── overview/            # Preface & Introduction
│   │   ├── module-1-ros2/       # ROS 2 module
│   │   ├── module-2-digital-twin/
│   │   ├── module-3-ai-brain/
│   │   ├── module-4-vla/
│   │   ├── module-5-rl-locomotion/
│   │   ├── module-6-sensor-fusion/
│   │   └── conclusion/          # Future of Physical AI
│   ├── src/
│   │   ├── css/                 # Custom styles
│   │   └── pages/               # Custom pages
│   ├── static/                  # Images, assets
│   ├── docusaurus.config.js     # Site configuration
│   └── sidebars.js              # Navigation structure
├── backend/                     # RAG service (optional)
├── specs/                       # Design documents
└── README.md
```

---

## RAG Integration (Optional)

This textbook is designed to support RAG (Retrieval-Augmented Generation) for AI-powered Q&A:

```bash
# Set up backend
cd backend
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env with your API keys

# Run embedding pipeline
python scripts/embed_content.py

# Start API server
uvicorn app.main:app --reload
```

**Environment Variables:**

```env
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-key
GROQ_API_KEY=your-key
```

---

## Contributing

Contributions are welcome! Please read our contributing guidelines before submitting PRs.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

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
- [ROS 2](https://docs.ros.org/) community for robotics middleware
- [NVIDIA Isaac](https://developer.nvidia.com/isaac-sim) for simulation tools
- The open-source robotics community

---

<p align="center">
  <i>The robots are learning to walk. Let's learn to build them.</i>
</p>
