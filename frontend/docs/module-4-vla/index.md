---
sidebar_position: 5
---

# Module 4: Vision-Language-Action (VLA)

> **Author:** Umema Sultan

---

## Learning Objectives

By the end of this module, you will be able to:

- Explain the Vision-Language-Action pipeline from speech to robot motion
- Implement voice-to-text transcription using OpenAI Whisper
- Design LLM prompts that decompose natural language into robot primitives
- Apply visual grounding to connect language references to detected objects
- Build an action executor that bridges high-level plans to ROS 2 commands

---

VLA models are where language meets embodiment. Instead of robots that only follow programmed routines, we can build systems that understand natural language, perceive their environment, and take meaningful action. This module covers the practical integration of speech, vision, and robotic control.

---

## The VLA Pipeline

```
Voice → Text → Understanding → Plan → Actions → Robot
```

| Stage | Tool | Output |
|-------|------|--------|
| Speech | Whisper | Raw text |
| Understanding | LLM | Structured intent |
| Grounding | Vision model | Object locations |
| Planning | LLM + constraints | Action sequence |
| Execution | ROS 2 | Robot motion |

---

## Voice-to-Text with Whisper

OpenAI's Whisper provides robust speech recognition:

### Basic Transcription

```python
import whisper

model = whisper.load_model("base")  # or "small", "medium", "large"

def transcribe(audio_path: str) -> str:
    result = model.transcribe(audio_path)
    return result["text"]

# "Pick up the red cup and bring it to me"
command = transcribe("command.wav")
```

### Real-Time Streaming

```python
import sounddevice as sd
import numpy as np
import whisper

model = whisper.load_model("base")

def listen(duration: float = 5.0, sample_rate: int = 16000) -> str:
    print("Listening...")
    audio = sd.rec(
        int(duration * sample_rate),
        samplerate=sample_rate,
        channels=1,
        dtype=np.float32
    )
    sd.wait()

    result = model.transcribe(audio.flatten())
    return result["text"].strip()

# Usage
while True:
    command = listen()
    if command:
        print(f"Heard: {command}")
        process_command(command)
```

---

## Cognitive Planning with LLMs

Translate natural language to robot actions:

### Task Decomposition

```python
from openai import OpenAI

client = OpenAI()

SYSTEM_PROMPT = """You are a robot task planner. Given a natural language instruction
and a list of detected objects, output a JSON array of primitive actions.

Available primitives:
- navigate_to(location: str)
- look_at(target: str)
- grasp(object_id: str)
- place(location: str)
- release()
- say(message: str)

Output only valid JSON."""

def plan_task(instruction: str, detected_objects: list[dict]) -> list[dict]:
    response = client.chat.completions.create(
        model="gpt-4o",
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": f"""
Instruction: {instruction}

Detected objects:
{detected_objects}

Plan:"""}
        ],
        response_format={"type": "json_object"}
    )

    return json.loads(response.choices[0].message.content)["actions"]
```

### Example Interaction

```python
objects = [
    {"id": "cup_1", "label": "red cup", "location": [1.2, 0.5, 0.8]},
    {"id": "cup_2", "label": "blue cup", "location": [1.4, 0.3, 0.8]},
    {"id": "table_1", "label": "table", "location": [1.3, 0.4, 0.7]}
]

plan = plan_task("Pick up the red cup", objects)

# Output:
# [
#   {"action": "navigate_to", "args": {"location": "table_1"}},
#   {"action": "look_at", "args": {"target": "cup_1"}},
#   {"action": "grasp", "args": {"object_id": "cup_1"}},
#   {"action": "say", "args": {"message": "I have the red cup"}}
# ]
```

---

## Visual Grounding

Connect language references to detected objects:

### Object Detection with YOLO

```python
from ultralytics import YOLO

model = YOLO("yolov8n.pt")

def detect_objects(image) -> list[dict]:
    results = model(image)
    objects = []

    for r in results:
        for box in r.boxes:
            objects.append({
                "id": f"obj_{len(objects)}",
                "label": model.names[int(box.cls)],
                "confidence": float(box.conf),
                "bbox": box.xyxy[0].tolist()
            })

    return objects
```

### Grounding with CLIP

```python
import clip
import torch

model, preprocess = clip.load("ViT-B/32")

def ground_reference(query: str, objects: list[dict], image) -> dict:
    """Find which detected object matches the language reference"""

    # Crop object regions
    crops = [crop_image(image, obj["bbox"]) for obj in objects]

    # Encode crops and query
    image_features = model.encode_image(torch.stack([
        preprocess(crop) for crop in crops
    ]))
    text_features = model.encode_text(clip.tokenize([query]))

    # Find best match
    similarities = (image_features @ text_features.T).squeeze()
    best_idx = similarities.argmax().item()

    return objects[best_idx]
```

---

## Action Execution

Bridge plans to ROS 2:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from control_msgs.action import GripperCommand

class ActionExecutor(Node):
    def __init__(self):
        super().__init__('action_executor')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.gripper_client = ActionClient(self, GripperCommand, 'gripper_controller')

        self.primitives = {
            "navigate_to": self.navigate_to,
            "grasp": self.grasp,
            "release": self.release,
            "say": self.say
        }

    async def execute_plan(self, plan: list[dict]):
        for step in plan:
            action = step["action"]
            args = step.get("args", {})

            self.get_logger().info(f"Executing: {action}({args})")

            if action in self.primitives:
                await self.primitives[action](**args)
            else:
                self.get_logger().warn(f"Unknown action: {action}")

    async def navigate_to(self, location: str):
        goal = NavigateToPose.Goal()
        goal.pose = self.get_location_pose(location)
        await self.nav_client.send_goal_async(goal)

    async def grasp(self, object_id: str):
        # Move arm to object, then close gripper
        goal = GripperCommand.Goal()
        goal.command.position = 0.0  # Closed
        await self.gripper_client.send_goal_async(goal)

    async def release(self):
        goal = GripperCommand.Goal()
        goal.command.position = 0.08  # Open
        await self.gripper_client.send_goal_async(goal)

    def say(self, message: str):
        # Text-to-speech
        import pyttsx3
        engine = pyttsx3.init()
        engine.say(message)
        engine.runAndWait()
```

---

## Capstone: Autonomous Humanoid Assistant

Build a voice-controlled humanoid that can understand and execute natural language commands.

### Architecture

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   Whisper   │────▶│    GPT-4    │────▶│   ROS 2     │
│   (Voice)   │     │  (Planning) │     │  (Control)  │
└─────────────┘     └─────────────┘     └─────────────┘
                           │
                    ┌──────┴──────┐
                    │   YOLO +    │
                    │    CLIP     │
                    │ (Grounding) │
                    └─────────────┘
```

### Implementation Phases

**Phase 1: Perception**
- [ ] Deploy YOLO for object detection
- [ ] Add depth estimation for 3D positions
- [ ] Build scene graph with object relationships

**Phase 2: Voice Interface**
- [ ] Implement Whisper transcription node
- [ ] Add wake word detection ("Hey Robot")
- [ ] Implement text-to-speech responses

**Phase 3: Planning**
- [ ] Connect GPT-4 for task decomposition
- [ ] Implement object grounding with CLIP
- [ ] Add plan validation against robot capabilities

**Phase 4: Execution**
- [ ] Map primitives to ROS 2 actions
- [ ] Implement error recovery
- [ ] Add safety constraints

### Demo Scenario

```
User: "Hey Robot"
Robot: "Yes, I'm listening."

User: "Can you get me something to drink?"
Robot: "I see a water bottle and a coffee mug on the counter.
        Which would you prefer?"

User: "The water bottle please."
Robot: "Got it. I'll get the water bottle for you."

[Robot navigates to counter]
[Robot grasps water bottle]
[Robot returns to user]

Robot: "Here's your water bottle."
```

### Success Criteria

- [ ] Wake word activates listening mode
- [ ] Commands transcribed with >95% accuracy
- [ ] Robot asks clarifying questions for ambiguous requests
- [ ] Successful pick-and-place of at least 3 object types
- [ ] "Stop" command halts all motion within 500ms
- [ ] Graceful handling of unknown objects/commands

---

## Further Reading

- [RT-2: Vision-Language-Action Models](https://robotics-transformer2.github.io/)
- [OpenVLA: Open-Source VLA Models](https://openvla.github.io/)
- [LeRobot: Hugging Face Robotics](https://github.com/huggingface/lerobot)

---

## Key Takeaways

- **VLA models** bridge the gap between natural language understanding and physical robot action
- **Whisper** provides robust speech recognition that handles diverse accents and noisy environments
- **LLMs as planners** decompose high-level instructions into sequences of executable primitives
- **Visual grounding** connects language references ("the red cup") to specific detected objects
- **Modularity matters**—separating perception, planning, and execution enables debugging and improvement

---

**Previous:** [← Module 3 — NVIDIA Isaac](../module-3-ai-brain/index.md)

**Next:** [Module 5 — Sensor Fusion & State Estimation →](../module-6-sensor-fusion/index.md)
