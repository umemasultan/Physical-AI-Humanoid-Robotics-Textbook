import React, { useState, useRef, useEffect, useCallback } from 'react';
import styles from './ChatWidget.module.css';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      role: 'assistant',
      content: '👋 **Welcome to Physical AI & Humanoid Robotics!**\n\nI\'m your intelligent assistant created by **Umema Sultan**. I can help you master:\n\n• 🤖 **ROS 2** - Robot middleware & architecture\n• 🎮 **Digital Twins** - Gazebo & Unity simulation\n• ⚡ **NVIDIA Isaac** - GPU-accelerated robotics\n• 🗣️ **VLA Models** - Voice-controlled robots\n• 📡 **Sensor Fusion** - State estimation\n• 🦿 **RL Locomotion** - Teaching robots to walk\n\nAsk me anything or try the suggestions below!',
      timestamp: new Date(),
    },
  ]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isMinimized, setIsMinimized] = useState(false);
  const [isListening, setIsListening] = useState(false);
  const [copiedIndex, setCopiedIndex] = useState(null);
  const [showEmoji, setShowEmoji] = useState(false);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);
  const recognitionRef = useRef(null);

  // Production API URL
  const API_URL = 'https://physical-ai-chatbot-api.onrender.com';

  // Enhanced fallback responses with rich formatting
  const fallbackResponses = {
    "ros": `## 🤖 ROS 2 (Robot Operating System 2)

ROS 2 is the **middleware backbone** that connects every part of a robot—sensors, actuators, and AI—into one coordinated system.

### 🎯 Key Features
| Feature | Description |
|---------|-------------|
| **Modular** | Build complex systems from reusable components |
| **Multi-language** | Write nodes in Python, C++, or both |
| **Real-time** | DDS-based communication with configurable QoS |
| **Portable** | Same code runs in simulation and on hardware |

### 📦 Core Concepts
- **Nodes** → Independent processes that do one thing well
- **Topics** → Publish/subscribe channels for streaming data
- **Services** → Request/response for discrete operations
- **Actions** → Long-running tasks with feedback

### 💡 Quick Example
\`\`\`python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
\`\`\`

📚 **Learn more in Module 1!**`,

    "digital twin": `## 🎮 Digital Twin Technology

A **Digital Twin** is a complete virtual replica that mirrors physical behavior of your robot in real-time.

### 🏗️ Architecture Components
\`\`\`
┌─────────────────────────────────────────┐
│           DIGITAL TWIN STACK            │
├─────────────────────────────────────────┤
│  🎨 Visualization    │  Unity/Unreal    │
│  ⚙️ Physics Engine   │  PhysX/Bullet    │
│  📐 Geometry Model   │  URDF/SDF        │
│  📡 Sensor Sim       │  Ray-tracing     │
│  🔗 ROS Bridge       │  ros2_control    │
└─────────────────────────────────────────┘
\`\`\`

### 🛠️ Simulation Tools
| Tool | Best For | Key Feature |
|------|----------|-------------|
| **Gazebo** | Physics accuracy | Native ROS 2 |
| **Unity** | Photorealism | ML-Agents |
| **Isaac Sim** | GPU acceleration | Domain randomization |

### 🎯 Why Use Digital Twins?
- ✅ Safe experimentation—crash thousands of times
- ✅ Parallel training for reinforcement learning
- ✅ Domain randomization for sim-to-real transfer
- ✅ Rapid prototyping before hardware exists

📚 **Dive deeper in Module 2!**`,

    "isaac": `## ⚡ NVIDIA Isaac Platform

**NVIDIA Isaac** brings GPU acceleration to robotics through three powerful pillars:

### 🎮 Isaac Sim
\`\`\`
Performance Metrics:
├── RTX Ray Tracing    → Photorealistic rendering
├── PhysX 5.0          → 10x faster physics
├── Synthetic Data     → Auto ground truth
└── Domain Random      → Built-in variation
\`\`\`

### 🧠 Isaac ROS
| Package | Function | Speedup |
|---------|----------|---------|
| **cuVSLAM** | Visual SLAM | 5-10x |
| **nvblox** | 3D Mapping | Real-time |
| **DNN Inference** | Perception | 10x+ |

### 🏋️ Isaac Gym (Now IsaacLab)
\`\`\`python
# Train 4096 robots simultaneously!
env = IsaacGymEnvs(
    num_envs=4096,
    physics_engine="GPU",
    headless=True
)
# Zero CPU-GPU transfer during training
\`\`\`

### 🚀 Performance Comparison
- Traditional: ~100 env steps/sec
- Isaac Gym: ~100,000+ env steps/sec
- **1000x speedup!**

📚 **Master GPU robotics in Module 3!**`,

    "vla": `## 🗣️ Vision-Language-Action Models

**VLA Models** bridge natural language understanding and physical robot action—the key to intuitive human-robot interaction.

### 🔄 Complete Pipeline
\`\`\`
┌──────────────────────────────────────────────────┐
│                  VLA PIPELINE                     │
├──────────────────────────────────────────────────┤
│  🎤 Voice  →  📝 Text  →  🧠 LLM  →  📋 Plan    │
│                   ↓                               │
│  👁️ Vision  →  🎯 Grounding  →  🤖 Action       │
└──────────────────────────────────────────────────┘
\`\`\`

### 🛠️ Technology Stack
| Component | Technology | Purpose |
|-----------|------------|---------|
| **Speech** | Whisper | Voice → Text |
| **Vision** | CLIP + YOLO | Scene understanding |
| **Planning** | GPT-4/Claude | Task decomposition |
| **Execution** | ROS 2 | Motor commands |

### 💬 Example Interaction
\`\`\`
User: "Pick up the red cup and place it on the table"

Robot Processing:
1. 🎤 Whisper: Transcribe speech
2. 🧠 LLM: Parse intent → [PICK(red_cup), PLACE(table)]
3. 👁️ YOLO: Detect red cup at (x, y, z)
4. 🦾 Motion: Plan & execute grasp
5. ✅ Verify: Confirm task completion
\`\`\`

📚 **Build voice robots in Module 4!**`,

    "sensor fusion": `## 📡 Sensor Fusion & State Estimation

**Sensor Fusion** combines data from multiple sensors to estimate robot state with high accuracy and reliability.

### 📊 Sensor Comparison
\`\`\`
┌─────────────┬──────────┬──────────┬───────────┐
│   Sensor    │   Rate   │ Accuracy │   Drift   │
├─────────────┼──────────┼──────────┼───────────┤
│ IMU         │ 400 Hz   │ Medium   │ High      │
│ Camera      │ 30-60 Hz │ High     │ Low       │
│ LiDAR       │ 10-20 Hz │ V.High   │ None      │
│ Encoders    │ 1000 Hz  │ High     │ Cumulative│
└─────────────┴──────────┴──────────┴───────────┘
\`\`\`

### 🧮 Key Algorithms

**Kalman Filter Family:**
\`\`\`
Standard KF  → Linear systems
     ↓
Extended KF  → Nonlinear (Jacobians)
     ↓
Unscented KF → Better nonlinear
     ↓
Particle Filter → Non-Gaussian
\`\`\`

### 🔧 EKF State Vector
\`\`\`python
state = [
    x, y, z,           # Position
    roll, pitch, yaw,  # Orientation
    vx, vy, vz,        # Linear velocity
    wx, wy, wz         # Angular velocity
]  # 12-DOF state estimation
\`\`\`

### 🎯 Fusion Strategy
- **IMU**: Fast but drifts → Short-term trust
- **Vision**: Slow but stable → Long-term correction
- **Result**: Best of both worlds!

📚 **Master estimation in Module 5!**`,

    "reinforcement learning": `## 🦿 Reinforcement Learning for Locomotion

**RL Locomotion** teaches robots to walk through millions of simulated trials—learning from experience, not programming.

### 🎯 The RL Framework
\`\`\`
┌─────────────────────────────────────────┐
│         LOCOMOTION RL LOOP              │
├─────────────────────────────────────────┤
│  📷 Observation → 🧠 Policy → 🎬 Action │
│         ↑                       ↓       │
│         └──── 🏆 Reward ←──────┘        │
└─────────────────────────────────────────┘
\`\`\`

### 📊 Reward Function Design
\`\`\`python
def compute_reward(robot):
    reward = (
        + 1.0 * forward_velocity      # Move forward
        - 0.1 * energy_consumed       # Efficiency
        - 0.5 * body_orientation_err  # Stay upright
        - 1.0 * ground_impact_force   # Smooth motion
        + 0.2 * foot_clearance        # Lift feet
    )
    return reward
\`\`\`

### 🏋️ Training Pipeline
| Stage | Environment | Duration |
|-------|-------------|----------|
| 1. Train | Isaac Gym (4096 robots) | ~2 hours |
| 2. Evaluate | Sim with noise | ~30 min |
| 3. Deploy | Real robot | Immediate |

### 🔧 Sim-to-Real Techniques
- ✅ **Domain Randomization** - Vary physics params
- ✅ **Curriculum Learning** - Easy → Hard tasks
- ✅ **System ID** - Match sim to real
- ✅ **Residual Learning** - Fine-tune on hardware

📚 **Train walking robots in Module 6!**`,

    "default": `## 👋 Hello from Physical AI Assistant!

I'm your **intelligent guide** to Physical AI & Humanoid Robotics, created by **Umema Sultan**.

### 📚 What I Can Teach You

| Module | Topic | Key Skills |
|--------|-------|------------|
| **1** | ROS 2 | Nodes, Topics, Services |
| **2** | Digital Twin | Gazebo, Unity, Simulation |
| **3** | NVIDIA Isaac | GPU Acceleration, IsaacLab |
| **4** | VLA Models | Voice Control, LLM Planning |
| **5** | Sensor Fusion | Kalman Filter, VIO |
| **6** | RL Locomotion | PPO, Sim-to-Real |

### 💡 Try Asking Me
- *"Explain ROS 2 nodes and topics"*
- *"How does Isaac Gym achieve 1000x speedup?"*
- *"What is the Kalman Filter?"*
- *"How do robots learn to walk?"*

### 🎯 Quick Tips
- Use **voice input** 🎤 for hands-free questions
- **Copy responses** 📋 to save for later
- Browse **modules** in the sidebar

---
*Built with ❤️ by Umema Sultan*
*Powered by RAG + FastAPI + React*`,

    "greeting": `## 👋 Hey there! Nice to meet you!

I'm your **Physical AI Assistant**, created by **Umema Sultan**!

I'm here to help you learn about **Humanoid Robotics** and **Physical AI**. 🤖

### 💬 You can ask me things like:
- "What is ROS 2?"
- "How do robots learn to walk?"
- "Explain Digital Twin"
- "What is sensor fusion?"

### 🎯 Or try these quick topics:
- 🤖 **ROS 2** - Robot Operating System
- 🎮 **Gazebo/Unity** - Simulation
- ⚡ **NVIDIA Isaac** - GPU Robotics
- 🦿 **RL Locomotion** - Teaching robots to walk

**Go ahead, ask me anything!** I'm happy to help! 😊

---
*Your friendly AI assistant by Umema Sultan*`,

    "thanks": `## 😊 You're welcome!

I'm glad I could help! If you have more questions about **Physical AI** or **Humanoid Robotics**, feel free to ask anytime!

### 🚀 Want to explore more?
- Check out the **modules** in the sidebar
- Try asking about **ROS 2**, **Isaac Gym**, or **Sensor Fusion**
- Use the **voice input** 🎤 for hands-free questions

Happy learning! 🤖

*— Your AI Assistant by Umema Sultan*`,

    "help": `## 🆘 How can I help you?

I'm your **Physical AI Textbook Assistant**! Here's what I can do:

### 📚 Topics I Know:
| Ask about... | I'll explain... |
|--------------|-----------------|
| **ROS 2** | Nodes, topics, services, actions |
| **Digital Twin** | Gazebo, Unity, simulation |
| **NVIDIA Isaac** | GPU acceleration, IsaacLab |
| **VLA Models** | Voice control, LLM planning |
| **Sensor Fusion** | Kalman filter, IMU, VIO |
| **RL Locomotion** | PPO, rewards, sim-to-real |

### 💡 Example Questions:
- "What is ROS 2?"
- "How does Isaac Gym work?"
- "Explain the Kalman Filter"
- "How do robots learn to walk?"

Just type your question and I'll do my best to help! 😊

*— Built by Umema Sultan*`
  };

  const getLocalResponse = (question) => {
    const q = question.toLowerCase().trim();

    // Greetings - Hi, Hello, Hey, etc.
    if (q.match(/^(hi|hello|hey|hii|hiii|hiiii|helo|hellow|assalam|salam|aoa|good morning|good afternoon|good evening|sup|yo|howdy)[\s!?.]*$/i) ||
        q.includes('how are you') || q.includes('kaise ho') || q.includes('kya hal'))
      return fallbackResponses['greeting'];

    // Thanks responses
    if (q.match(/^(thanks|thank you|thanku|thnx|ty|shukriya|dhanyawad|thx)[\s!?.]*$/i) ||
        q.includes('thanks') || q.includes('thank you'))
      return fallbackResponses['thanks'];

    // Help requests
    if (q.match(/^(help|help me|what can you do|kya kar sakte ho)[\s!?.]*$/i))
      return fallbackResponses['help'];

    // Technical topics
    if (q.includes('ros') || q.includes('node') || q.includes('topic') || q.includes('service'))
      return fallbackResponses['ros'];
    if (q.includes('digital twin') || q.includes('gazebo') || q.includes('simulation') || q.includes('unity') || q.includes('simulator'))
      return fallbackResponses['digital twin'];
    if (q.includes('isaac') || q.includes('nvidia') || q.includes('gpu') || q.includes('cuda'))
      return fallbackResponses['isaac'];
    if (q.includes('vla') || q.includes('vision') || q.includes('language') || q.includes('whisper') || q.includes('voice') || q.includes('speech'))
      return fallbackResponses['vla'];
    if (q.includes('sensor') || q.includes('fusion') || q.includes('kalman') || q.includes('imu') || q.includes('estimation') || q.includes('filter'))
      return fallbackResponses['sensor fusion'];
    if (q.includes('reinforcement') || q.includes('learning') || q.includes('locomotion') || q.includes('walk') || q.includes('ppo') || q.includes('reward'))
      return fallbackResponses['reinforcement learning'];

    return fallbackResponses['default'];
  };

  const suggestedQuestions = [
    "🤖 What is ROS 2?",
    "🎮 Explain Digital Twin",
    "⚡ How does Isaac Gym work?",
    "🗣️ What is VLA in robotics?",
    "📡 Explain Kalman Filter",
    "🦿 How do robots learn to walk?",
  ];

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  // Initialize speech recognition
  useEffect(() => {
    if ('webkitSpeechRecognition' in window || 'SpeechRecognition' in window) {
      const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
      recognitionRef.current = new SpeechRecognition();
      recognitionRef.current.continuous = false;
      recognitionRef.current.interimResults = false;
      recognitionRef.current.lang = 'en-US';

      recognitionRef.current.onresult = (event) => {
        const transcript = event.results[0][0].transcript;
        setInput(transcript);
        setIsListening(false);
      };

      recognitionRef.current.onerror = () => {
        setIsListening(false);
      };

      recognitionRef.current.onend = () => {
        setIsListening(false);
      };
    }
  }, []);

  const toggleVoiceInput = () => {
    if (!recognitionRef.current) {
      alert('Voice input is not supported in your browser');
      return;
    }

    if (isListening) {
      recognitionRef.current.stop();
      setIsListening(false);
    } else {
      recognitionRef.current.start();
      setIsListening(true);
    }
  };

  const formatTime = (date) => {
    return new Intl.DateTimeFormat('en-US', {
      hour: '2-digit',
      minute: '2-digit',
    }).format(date);
  };

  // Simple markdown-like formatting
  const formatMessage = (content) => {
    if (!content) return '';

    // Split by code blocks first
    const parts = content.split(/(```[\s\S]*?```)/g);

    return parts.map((part, index) => {
      if (part.startsWith('```')) {
        // Code block
        const code = part.replace(/```\w*\n?/g, '').replace(/```$/g, '');
        return (
          <pre key={index} className={styles.codeBlock}>
            <code>{code}</code>
          </pre>
        );
      }

      // Process other markdown
      let formatted = part
        // Headers
        .replace(/^### (.*$)/gm, '<h4>$1</h4>')
        .replace(/^## (.*$)/gm, '<h3>$1</h3>')
        // Bold
        .replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>')
        // Italic
        .replace(/\*(.*?)\*/g, '<em>$1</em>')
        // Inline code
        .replace(/`([^`]+)`/g, '<code class="inline-code">$1</code>')
        // Links
        .replace(/\[([^\]]+)\]\(([^)]+)\)/g, '<a href="$2" target="_blank" rel="noopener">$1</a>')
        // Line breaks
        .replace(/\n/g, '<br/>');

      return <span key={index} dangerouslySetInnerHTML={{ __html: formatted }} />;
    });
  };

  const copyMessage = useCallback((content, index) => {
    // Strip markdown for plain text copy
    const plainText = content
      .replace(/\*\*(.*?)\*\*/g, '$1')
      .replace(/\*(.*?)\*/g, '$1')
      .replace(/`([^`]+)`/g, '$1')
      .replace(/```[\s\S]*?```/g, '')
      .replace(/^#{1,6} /gm, '')
      .replace(/\[([^\]]+)\]\([^)]+\)/g, '$1');

    navigator.clipboard.writeText(plainText);
    setCopiedIndex(index);
    setTimeout(() => setCopiedIndex(null), 2000);
  }, []);

  const sendMessage = async (messageText = null) => {
    const userMessage = messageText || input.trim();
    if (!userMessage || isLoading) return;

    setInput('');
    const newUserMessage = {
      role: 'user',
      content: userMessage,
      timestamp: new Date(),
    };
    setMessages((prev) => [...prev, newUserMessage]);
    setIsLoading(true);

    try {
      const response = await fetch(`${API_URL}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ question: userMessage }),
      });

      if (!response.ok) {
        throw new Error('Failed to get response');
      }

      const data = await response.json();
      setMessages((prev) => [
        ...prev,
        {
          role: 'assistant',
          content: data.answer,
          sources: data.sources,
          timestamp: new Date(),
        },
      ]);
    } catch (error) {
      // Use local fallback responses when backend is unavailable
      const fallbackAnswer = getLocalResponse(userMessage);
      setMessages((prev) => [
        ...prev,
        {
          role: 'assistant',
          content: fallbackAnswer,
          timestamp: new Date(),
          sources: ['📚 Local Knowledge Base'],
        },
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const clearChat = () => {
    setMessages([
      {
        role: 'assistant',
        content: '🔄 **Chat cleared!**\n\nHow can I help you with Physical AI and Humanoid Robotics?\n\n*— Your assistant, by Umema Sultan*',
        timestamp: new Date(),
      },
    ]);
  };

  const quickEmojis = ['👍', '🤖', '💡', '🚀', '❤️', '🎯'];

  return (
    <div className={styles.chatWidget}>
      {/* Floating Action Button */}
      <button
        className={`${styles.fab} ${isOpen ? styles.fabOpen : ''}`}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle AI Assistant"
      >
        <div className={styles.fabIcon}>
          {isOpen ? (
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
              <line x1="18" y1="6" x2="6" y2="18"></line>
              <line x1="6" y1="6" x2="18" y2="18"></line>
            </svg>
          ) : (
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
              <path d="M21 11.5a8.38 8.38 0 0 1-.9 3.8 8.5 8.5 0 0 1-7.6 4.7 8.38 8.38 0 0 1-3.8-.9L3 21l1.9-5.7a8.38 8.38 0 0 1-.9-3.8 8.5 8.5 0 0 1 4.7-7.6 8.38 8.38 0 0 1 3.8-.9h.5a8.48 8.48 0 0 1 8 8v.5z"></path>
            </svg>
          )}
        </div>
        {!isOpen && <span className={styles.fabLabel}>AI Chatbot</span>}
        <div className={styles.fabPulse}></div>
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={`${styles.chatWindow} ${isMinimized ? styles.minimized : ''}`}>
          {/* Header */}
          <div className={styles.header}>
            <div className={styles.headerLeft}>
              <div className={styles.avatar}>
                <svg viewBox="0 0 24 24" fill="currentColor">
                  <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-1 17.93c-3.95-.49-7-3.85-7-7.93 0-.62.08-1.21.21-1.79L9 15v1c0 1.1.9 2 2 2v1.93zm6.9-2.54c-.26-.81-1-1.39-1.9-1.39h-1v-3c0-.55-.45-1-1-1H8v-2h2c.55 0 1-.45 1-1V7h2c1.1 0 2-.9 2-2v-.41c2.93 1.19 5 4.06 5 7.41 0 2.08-.8 3.97-2.1 5.39z"/>
                </svg>
              </div>
              <div className={styles.headerInfo}>
                <h3>AI Chatbot</h3>
                <span className={styles.status}>
                  <span className={styles.statusDot}></span>
                  Online • by Umema Sultan
                </span>
              </div>
            </div>
            <div className={styles.headerActions}>
              <button
                onClick={clearChat}
                className={styles.headerBtn}
                title="Clear chat"
              >
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M3 6h18M19 6v14a2 2 0 01-2 2H7a2 2 0 01-2-2V6m3 0V4a2 2 0 012-2h4a2 2 0 012 2v2"/>
                </svg>
              </button>
              <button
                onClick={() => setIsMinimized(!isMinimized)}
                className={styles.headerBtn}
                title={isMinimized ? "Expand" : "Minimize"}
              >
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  {isMinimized ? (
                    <path d="M4 12h16M12 4v16"/>
                  ) : (
                    <path d="M4 12h16"/>
                  )}
                </svg>
              </button>
              <button
                onClick={() => setIsOpen(false)}
                className={styles.headerBtn}
                title="Close"
              >
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <line x1="18" y1="6" x2="6" y2="18"></line>
                  <line x1="6" y1="6" x2="18" y2="18"></line>
                </svg>
              </button>
            </div>
          </div>

          {!isMinimized && (
            <>
              {/* Messages */}
              <div className={styles.messagesContainer}>
                {messages.map((msg, idx) => (
                  <div
                    key={idx}
                    className={`${styles.messageWrapper} ${
                      msg.role === 'user' ? styles.userWrapper : styles.assistantWrapper
                    }`}
                  >
                    {msg.role === 'assistant' && (
                      <div className={styles.messageAvatar}>
                        <svg viewBox="0 0 24 24" fill="currentColor">
                          <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2z"/>
                        </svg>
                      </div>
                    )}
                    <div className={`${styles.message} ${
                      msg.role === 'user' ? styles.userMessage : styles.assistantMessage
                    } ${msg.isError ? styles.errorMessage : ''}`}>
                      <div className={styles.messageContent}>
                        {msg.role === 'assistant' ? formatMessage(msg.content) : msg.content}
                      </div>

                      {/* Message Actions */}
                      {msg.role === 'assistant' && (
                        <div className={styles.messageActions}>
                          <button
                            className={styles.actionBtn}
                            onClick={() => copyMessage(msg.content, idx)}
                            title="Copy message"
                          >
                            {copiedIndex === idx ? (
                              <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                                <path d="M20 6L9 17l-5-5"/>
                              </svg>
                            ) : (
                              <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                                <rect x="9" y="9" width="13" height="13" rx="2" ry="2"/>
                                <path d="M5 15H4a2 2 0 01-2-2V4a2 2 0 012-2h9a2 2 0 012 2v1"/>
                              </svg>
                            )}
                          </button>
                        </div>
                      )}

                      {msg.sources && msg.sources.length > 0 && (
                        <div className={styles.sources}>
                          <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                            <path d="M14 2H6a2 2 0 00-2 2v16a2 2 0 002 2h12a2 2 0 002-2V8z"/>
                            <path d="M14 2v6h6M16 13H8M16 17H8M10 9H8"/>
                          </svg>
                          <span>{msg.sources.join(', ')}</span>
                        </div>
                      )}
                      <span className={styles.timestamp}>{formatTime(msg.timestamp)}</span>
                    </div>
                  </div>
                ))}

                {isLoading && (
                  <div className={`${styles.messageWrapper} ${styles.assistantWrapper}`}>
                    <div className={styles.messageAvatar}>
                      <svg viewBox="0 0 24 24" fill="currentColor">
                        <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2z"/>
                      </svg>
                    </div>
                    <div className={`${styles.message} ${styles.assistantMessage}`}>
                      <div className={styles.typingIndicator}>
                        <span></span>
                        <span></span>
                        <span></span>
                      </div>
                    </div>
                  </div>
                )}
                <div ref={messagesEndRef} />
              </div>

              {/* Suggested Questions */}
              {messages.length <= 1 && (
                <div className={styles.suggestions}>
                  <p>✨ Quick Questions:</p>
                  <div className={styles.suggestionChips}>
                    {suggestedQuestions.map((q, idx) => (
                      <button
                        key={idx}
                        className={styles.chip}
                        onClick={() => sendMessage(q)}
                      >
                        {q}
                      </button>
                    ))}
                  </div>
                </div>
              )}

              {/* Emoji Picker */}
              {showEmoji && (
                <div className={styles.emojiPicker}>
                  {quickEmojis.map((emoji, idx) => (
                    <button
                      key={idx}
                      className={styles.emojiBtn}
                      onClick={() => {
                        setInput(prev => prev + emoji);
                        setShowEmoji(false);
                      }}
                    >
                      {emoji}
                    </button>
                  ))}
                </div>
              )}

              {/* Input Area */}
              <div className={styles.inputArea}>
                <div className={styles.inputWrapper}>
                  <button
                    onClick={() => setShowEmoji(!showEmoji)}
                    className={styles.inputAction}
                    title="Add emoji"
                  >
                    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                      <circle cx="12" cy="12" r="10"/>
                      <path d="M8 14s1.5 2 4 2 4-2 4-2M9 9h.01M15 9h.01"/>
                    </svg>
                  </button>
                  <input
                    ref={inputRef}
                    type="text"
                    value={input}
                    onChange={(e) => setInput(e.target.value)}
                    onKeyPress={handleKeyPress}
                    placeholder="Ask about Physical AI & Robotics..."
                    className={styles.input}
                    disabled={isLoading}
                  />
                  <button
                    onClick={toggleVoiceInput}
                    className={`${styles.inputAction} ${isListening ? styles.listening : ''}`}
                    title="Voice input"
                  >
                    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                      <path d="M12 1a3 3 0 0 0-3 3v8a3 3 0 0 0 6 0V4a3 3 0 0 0-3-3z"/>
                      <path d="M19 10v2a7 7 0 0 1-14 0v-2M12 19v4M8 23h8"/>
                    </svg>
                  </button>
                  <button
                    onClick={() => sendMessage()}
                    disabled={isLoading || !input.trim()}
                    className={styles.sendBtn}
                    aria-label="Send message"
                  >
                    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                      <line x1="22" y1="2" x2="11" y2="13"></line>
                      <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
                    </svg>
                  </button>
                </div>
                <p className={styles.poweredBy}>
                  🤖 Built by <strong>Umema Sultan</strong> • Powered by RAG
                </p>
              </div>
            </>
          )}
        </div>
      )}
    </div>
  );
};

export default ChatWidget;
