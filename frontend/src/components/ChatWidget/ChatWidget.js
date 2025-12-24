import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatWidget.module.css';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      role: 'assistant',
      content: 'Welcome to the Physical AI & Humanoid Robotics assistant! I can help you with questions about ROS 2, Digital Twins, NVIDIA Isaac, Vision-Language-Action models, Sensor Fusion, and Reinforcement Learning. What would you like to learn?',
      timestamp: new Date(),
    },
  ]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isMinimized, setIsMinimized] = useState(false);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Production API URL - Update this after deploying backend to Render
  const API_URL = 'https://physical-ai-chatbot-api.onrender.com';

  // Fallback responses when backend is not available
  const fallbackResponses = {
    "ros": `**ROS 2 (Robot Operating System 2)** is the middleware that connects every part of a robot—sensors, actuators, and AI—into one coordinated system.

**Key Features:**
• **Modular architecture** — Build complex systems from independent, reusable components
• **Language agnostic** — Write nodes in Python, C++, or both
• **Real-time capable** — DDS-based communication with configurable QoS
• **Hardware abstraction** — Same code runs in simulation and on physical robots

**Core Concepts:**
• **Nodes** — Independent processes that do one thing well
• **Topics** — Publish/subscribe channels for streaming data
• **Services** — Request/response for discrete operations
• **Actions** — Long-running tasks with feedback

Learn more in Module 1 of this textbook!`,

    "digital twin": `**Digital Twin** is a complete virtual replica that mirrors physical behavior of your robot.

**Components:**
• **Geometric Model (URDF)** — 3D representation
• **Physics Simulation** — Dynamics and forces
• **Sensor Simulation** — Realistic noise models

**Tools:**
• **Gazebo** — Physics-accurate simulation with native ROS 2 integration
• **Unity** — Photorealistic rendering for vision-based tasks
• **Isaac Sim** — NVIDIA's GPU-accelerated simulator

**Why Use It:**
• Safe experimentation—robots can fall thousands of times without damage
• Parallel training for reinforcement learning
• Domain randomization for sim-to-real transfer

Learn more in Module 2!`,

    "isaac": `**NVIDIA Isaac** brings GPU acceleration to robotics through three pillars:

**Isaac Sim:**
• Photorealistic simulation with RTX ray tracing
• Automatic ground truth labeling
• Domain randomization built-in

**Isaac ROS:**
• GPU-accelerated perception (5-10x faster)
• cuVSLAM for visual SLAM (90-120 Hz)
• nvblox for real-time 3D mapping

**Isaac Gym:**
• Train 4096+ robots simultaneously
• Zero CPU-GPU transfer during training
• PPO/SAC algorithms built-in

Learn more in Module 3!`,

    "vla": `**Vision-Language-Action (VLA)** models bridge natural language understanding and physical robot action.

**Pipeline:**
\`\`\`
Voice → Text → Understanding → Plan → Actions → Robot
\`\`\`

**Components:**
• **Whisper** — Speech recognition
• **LLM (GPT-4)** — Task decomposition & planning
• **YOLO + CLIP** — Visual grounding
• **ROS 2** — Action execution

**Example:**
User: "Pick up the red cup"
Robot: Detects cup → Plans grasp → Executes motion

Learn more in Module 4!`,

    "sensor fusion": `**Sensor Fusion** combines data from multiple sensors to estimate robot state.

**Key Algorithms:**
• **Kalman Filter** — Optimal estimation for linear systems
• **Extended Kalman Filter (EKF)** — For nonlinear systems
• **Visual-Inertial Odometry (VIO)** — Camera + IMU fusion

**Sensors Used:**
• IMU (400 Hz) — Orientation, angular velocity
• Cameras (30-60 Hz) — Visual features
• Joint encoders (1000 Hz) — Position/velocity
• Force/Torque sensors — Contact detection

Learn more in Module 5!`,

    "reinforcement learning": `**Reinforcement Learning for Locomotion** teaches robots to walk through trial and error.

**Key Concepts:**
• **Policy** — Neural network that maps observations to actions
• **Reward Design** — Defines what "good walking" means
• **Sim-to-Real Transfer** — Making simulation-trained policies work on real robots

**Training Pipeline:**
1. Train in Isaac Gym (4096 parallel robots)
2. Apply domain randomization
3. Export policy to real robot
4. Fine-tune if needed

**Popular Algorithms:**
• PPO (Proximal Policy Optimization)
• SAC (Soft Actor-Critic)

Learn more in Module 6!`,

    "default": `I'm the Physical AI Textbook Assistant by **Umema Sultan**!

I can help you learn about:
• **ROS 2** — Robot middleware
• **Digital Twins** — Simulation with Gazebo/Unity
• **NVIDIA Isaac** — GPU-accelerated robotics
• **VLA Models** — Voice-controlled robots
• **Sensor Fusion** — State estimation
• **Reinforcement Learning** — Teaching robots to walk

Try asking:
• "What is ROS 2?"
• "Explain Digital Twin"
• "How does Isaac Gym work?"

Or browse the modules in the sidebar! 📚`
  };

  const getLocalResponse = (question) => {
    const q = question.toLowerCase();
    if (q.includes('ros')) return fallbackResponses['ros'];
    if (q.includes('digital twin') || q.includes('gazebo') || q.includes('simulation') || q.includes('unity')) return fallbackResponses['digital twin'];
    if (q.includes('isaac') || q.includes('nvidia') || q.includes('gpu')) return fallbackResponses['isaac'];
    if (q.includes('vla') || q.includes('vision') || q.includes('language') || q.includes('whisper') || q.includes('voice')) return fallbackResponses['vla'];
    if (q.includes('sensor') || q.includes('fusion') || q.includes('kalman') || q.includes('imu')) return fallbackResponses['sensor fusion'];
    if (q.includes('reinforcement') || q.includes('learning') || q.includes('locomotion') || q.includes('walk') || q.includes('ppo')) return fallbackResponses['reinforcement learning'];
    return fallbackResponses['default'];
  };

  const suggestedQuestions = [
    "What is ROS 2?",
    "Explain Digital Twin",
    "How does Isaac Gym work?",
    "What is VLA in robotics?",
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

  const formatTime = (date) => {
    return new Intl.DateTimeFormat('en-US', {
      hour: '2-digit',
      minute: '2-digit',
    }).format(date);
  };

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
          sources: ['Local Knowledge Base'],
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
        content: 'Chat cleared. How can I help you with Physical AI and Humanoid Robotics?',
        timestamp: new Date(),
      },
    ]);
  };

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
              <path d="M12 2a10 10 0 0 1 10 10c0 5.52-4.48 10-10 10a10 10 0 0 1-10-10A10 10 0 0 1 12 2z"></path>
              <path d="M8 10h.01M12 10h.01M16 10h.01"></path>
              <path d="M9 14c.83.64 1.86 1 3 1s2.17-.36 3-1"></path>
            </svg>
          )}
        </div>
        {!isOpen && <span className={styles.fabLabel}>Ask AI</span>}
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
                <h3>Physical AI Assistant</h3>
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
                      <div className={styles.messageContent}>{msg.content}</div>
                      {msg.sources && msg.sources.length > 0 && (
                        <div className={styles.sources}>
                          <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                            <path d="M14 2H6a2 2 0 00-2 2v16a2 2 0 002 2h12a2 2 0 002-2V8z"/>
                            <path d="M14 2v6h6M16 13H8M16 17H8M10 9H8"/>
                          </svg>
                          <span>Sources: {msg.sources.join(', ')}</span>
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
                  <p>Try asking:</p>
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

              {/* Input Area */}
              <div className={styles.inputArea}>
                <div className={styles.inputWrapper}>
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
                  Powered by RAG • Built with FastAPI & React
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
