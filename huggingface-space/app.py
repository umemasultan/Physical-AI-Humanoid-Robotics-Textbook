"""
Physical AI Chatbot - Hugging Face Spaces
Author: Umema Sultan
"""

import gradio as gr

# Fallback responses for the chatbot
fallback_responses = {
    "ros": """## 🤖 ROS 2 (Robot Operating System 2)

ROS 2 is the **middleware backbone** that connects every part of a robot—sensors, actuators, and AI—into one coordinated system.

### 🎯 Key Features
- **Modular** - Build complex systems from reusable components
- **Multi-language** - Write nodes in Python, C++, or both
- **Real-time** - DDS-based communication with configurable QoS
- **Portable** - Same code runs in simulation and on hardware

### 📦 Core Concepts
- **Nodes** → Independent processes that do one thing well
- **Topics** → Publish/subscribe channels for streaming data
- **Services** → Request/response for discrete operations
- **Actions** → Long-running tasks with feedback

📚 **Learn more in Module 1!**""",

    "digital_twin": """## 🎮 Digital Twin Technology

A **Digital Twin** is a complete virtual replica that mirrors physical behavior of your robot in real-time.

### 🛠️ Simulation Tools
- **Gazebo** - Physics accuracy, Native ROS 2
- **Unity** - Photorealism, ML-Agents
- **Isaac Sim** - GPU acceleration, Domain randomization

### 🎯 Why Use Digital Twins?
- ✅ Safe experimentation—crash thousands of times
- ✅ Parallel training for reinforcement learning
- ✅ Domain randomization for sim-to-real transfer
- ✅ Rapid prototyping before hardware exists

📚 **Dive deeper in Module 2!**""",

    "isaac": """## ⚡ NVIDIA Isaac Platform

**NVIDIA Isaac** brings GPU acceleration to robotics through three powerful pillars:

### 🎮 Isaac Sim
- RTX Ray Tracing → Photorealistic rendering
- PhysX 5.0 → 10x faster physics
- Synthetic Data → Auto ground truth

### 🧠 Isaac ROS
- **cuVSLAM** - Visual SLAM (5-10x speedup)
- **nvblox** - 3D Mapping (Real-time)
- **DNN Inference** - Perception (10x+)

### 🏋️ Isaac Gym
- Train 4096 robots simultaneously!
- Zero CPU-GPU transfer during training
- **1000x speedup** compared to traditional methods!

📚 **Master GPU robotics in Module 3!**""",

    "vla": """## 🗣️ Vision-Language-Action Models

**VLA Models** bridge natural language understanding and physical robot action.

### 🔄 Pipeline
🎤 Voice → 📝 Text → 🧠 LLM → 📋 Plan → 🤖 Action

### 🛠️ Technology Stack
- **Speech**: Whisper (Voice → Text)
- **Vision**: CLIP + YOLO (Scene understanding)
- **Planning**: GPT-4/Claude (Task decomposition)
- **Execution**: ROS 2 (Motor commands)

### 💬 Example
User: "Pick up the red cup"
Robot: Detects cup → Plans grasp → Executes motion

📚 **Build voice robots in Module 4!**""",

    "sensor_fusion": """## 📡 Sensor Fusion & State Estimation

**Sensor Fusion** combines data from multiple sensors to estimate robot state with high accuracy.

### 📊 Sensors Used
- IMU (400 Hz) - Fast but drifts
- Camera (30-60 Hz) - High accuracy
- LiDAR (10-20 Hz) - Very high accuracy
- Encoders (1000 Hz) - Position/velocity

### 🧮 Key Algorithms
- **Kalman Filter** → Linear systems
- **Extended KF** → Nonlinear (Jacobians)
- **Unscented KF** → Better nonlinear
- **Particle Filter** → Non-Gaussian

📚 **Master estimation in Module 5!**""",

    "rl": """## 🦿 Reinforcement Learning for Locomotion

**RL Locomotion** teaches robots to walk through millions of simulated trials.

### 🎯 The RL Framework
📷 Observation → 🧠 Policy → 🎬 Action → 🏆 Reward

### 🏋️ Training Pipeline
1. Train in Isaac Gym (4096 parallel robots)
2. Apply domain randomization
3. Export policy to real robot
4. Fine-tune if needed

### 🔧 Sim-to-Real Techniques
- ✅ Domain Randomization
- ✅ Curriculum Learning
- ✅ System ID
- ✅ Residual Learning

📚 **Train walking robots in Module 6!**""",

    "greeting": """## 👋 Hey there! Nice to meet you!

I'm your **Physical AI Assistant**, created by **Umema Sultan**!

I'm here to help you learn about **Humanoid Robotics** and **Physical AI**. 🤖

### 💬 You can ask me things like:
- "What is ROS 2?"
- "How do robots learn to walk?"
- "Explain Digital Twin"
- "What is sensor fusion?"

**Go ahead, ask me anything!** I'm happy to help! 😊

---
*Your friendly AI assistant by Umema Sultan*""",

    "thanks": """## 😊 You're welcome!

I'm glad I could help! If you have more questions about **Physical AI** or **Humanoid Robotics**, feel free to ask anytime!

Happy learning! 🤖

*— Your AI Assistant by Umema Sultan*""",

    "help": """## 🆘 How can I help you?

I'm your **Physical AI Textbook Assistant**! Here's what I can do:

### 📚 Topics I Know:
- **ROS 2** - Nodes, topics, services, actions
- **Digital Twin** - Gazebo, Unity, simulation
- **NVIDIA Isaac** - GPU acceleration, IsaacLab
- **VLA Models** - Voice control, LLM planning
- **Sensor Fusion** - Kalman filter, IMU, VIO
- **RL Locomotion** - PPO, rewards, sim-to-real

Just type your question and I'll do my best to help! 😊

*— Built by Umema Sultan*""",

    "default": """## 👋 Hello from Physical AI Assistant!

I'm your **intelligent guide** to Physical AI & Humanoid Robotics, created by **Umema Sultan**.

### 📚 What I Can Teach You
- **Module 1**: ROS 2 - Nodes, Topics, Services
- **Module 2**: Digital Twin - Gazebo, Unity, Simulation
- **Module 3**: NVIDIA Isaac - GPU Acceleration
- **Module 4**: VLA Models - Voice Control, LLM Planning
- **Module 5**: Sensor Fusion - Kalman Filter, VIO
- **Module 6**: RL Locomotion - PPO, Sim-to-Real

### 💡 Try Asking Me
- "What is ROS 2?"
- "How does Isaac Gym work?"
- "Explain the Kalman Filter"
- "How do robots learn to walk?"

---
*Built with ❤️ by Umema Sultan*"""
}

def get_response(message):
    """Get chatbot response based on user message"""
    q = message.lower().strip()

    # Greetings
    greetings = ['hi', 'hello', 'hey', 'hii', 'hiii', 'assalam', 'salam', 'aoa',
                 'good morning', 'good afternoon', 'good evening', 'sup', 'yo']
    if any(q.startswith(g) for g in greetings) or 'how are you' in q:
        return fallback_responses['greeting']

    # Thanks
    thanks_words = ['thanks', 'thank you', 'thanku', 'thnx', 'shukriya', 'dhanyawad']
    if any(t in q for t in thanks_words):
        return fallback_responses['thanks']

    # Help
    if q in ['help', 'help me'] or 'what can you do' in q:
        return fallback_responses['help']

    # Technical topics
    if 'ros' in q or 'node' in q or 'topic' in q:
        return fallback_responses['ros']
    if 'digital twin' in q or 'gazebo' in q or 'simulation' in q or 'unity' in q:
        return fallback_responses['digital_twin']
    if 'isaac' in q or 'nvidia' in q or 'gpu' in q:
        return fallback_responses['isaac']
    if 'vla' in q or 'vision' in q or 'language' in q or 'whisper' in q or 'voice' in q:
        return fallback_responses['vla']
    if 'sensor' in q or 'fusion' in q or 'kalman' in q or 'imu' in q:
        return fallback_responses['sensor_fusion']
    if 'reinforcement' in q or 'learning' in q or 'locomotion' in q or 'walk' in q or 'ppo' in q:
        return fallback_responses['rl']

    return fallback_responses['default']

def chat(message, history):
    """Chatbot function for Gradio"""
    response = get_response(message)
    return response

# Create Gradio Interface
with gr.Blocks(
    title="Physical AI Chatbot - by Umema Sultan",
    theme=gr.themes.Soft(primary_hue="purple", secondary_hue="blue"),
    css="""
    .gradio-container {
        max-width: 900px !important;
    }
    footer {display: none !important;}
    """
) as demo:
    gr.Markdown("""
    # 🤖 Physical AI & Humanoid Robotics Chatbot
    ### Created by **Umema Sultan**

    Ask me anything about ROS 2, Digital Twins, NVIDIA Isaac, VLA Models, Sensor Fusion, and Reinforcement Learning!
    """)

    chatbot = gr.ChatInterface(
        fn=chat,
        examples=[
            "Hi!",
            "What is ROS 2?",
            "Explain Digital Twin",
            "How does Isaac Gym work?",
            "What is sensor fusion?",
            "How do robots learn to walk?",
        ],
        title="",
        retry_btn=None,
        undo_btn=None,
    )

    gr.Markdown("""
    ---
    **📚 Topics covered:** ROS 2 | Digital Twins | NVIDIA Isaac | VLA Models | Sensor Fusion | RL Locomotion

    *Built with ❤️ by Umema Sultan | [View Textbook](https://physical-ai-humanoid-robotics-textbook.vercel.app)*
    """)

# Launch the app
if __name__ == "__main__":
    demo.launch()
