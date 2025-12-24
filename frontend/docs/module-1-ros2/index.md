---
sidebar_position: 2
---

# Module 1: The Robotic Nervous System (ROS 2)

> **Author:** Umema Sultan

---

## Learning Objectives

By the end of this module, you will be able to:

- Explain the role of middleware in robotic systems
- Describe the ROS 2 communication patterns: nodes, topics, services, and actions
- Connect Python-based AI agents to robot hardware through ROS 2
- Interpret and create URDF files describing humanoid robot structure
- Build a basic teleoperation system for joint control

---

ROS 2 is the middleware that connects every part of a robot—sensors, actuators, and AI—into one coordinated system. It handles communication, timing, and hardware abstraction so you can focus on building intelligent behavior rather than low-level plumbing.

---

## Why ROS 2?

- **Modular architecture** — Build complex systems from independent, reusable components
- **Language agnostic** — Write nodes in Python, C++, or both
- **Real-time capable** — DDS-based communication with configurable QoS
- **Hardware abstraction** — Same code runs in simulation and on physical robots

---

## Core Concepts

### Nodes

Independent processes that do one thing well:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node started')

rclpy.init()
node = MyNode()
rclpy.spin(node)
```

### Topics

Publish/subscribe channels for streaming data:

| Topic | Message Type | Use Case |
|-------|--------------|----------|
| `/cmd_vel` | `Twist` | Velocity commands |
| `/joint_states` | `JointState` | Current joint positions |
| `/camera/image` | `Image` | Camera feed |
| `/scan` | `LaserScan` | LiDAR data |

### Services

Request/response for discrete operations:

```python
# Client
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)
result = future.result()
```

### Actions

Long-running tasks with feedback:

- Navigation to waypoint
- Arm trajectory execution
- Full-body motion sequences

---

## Bridging Python Agents to ROS

Connect your AI agent to robot hardware:

```python
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AgentBridge(Node):
    def __init__(self):
        super().__init__('agent_bridge')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def send_velocity(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)
```

**Key patterns:**

- Use `create_publisher()` for sending commands
- Use `create_subscription()` for receiving sensor data
- Use `create_client()` for calling services
- Set appropriate QoS profiles for reliability vs. speed

---

## URDF: Describing Your Humanoid

URDF (Unified Robot Description Format) defines robot structure:

```xml
<robot name="humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry><mesh filename="package://humanoid/meshes/torso.stl"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.3 0.2 0.5"/></geometry>
    </collision>
  </link>

  <!-- Shoulder Joint -->
  <joint name="left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>
</robot>
```

**URDF elements:**

- **Links** — Rigid body segments with visual and collision geometry
- **Joints** — Connections with defined motion (revolute, prismatic, fixed)
- **Limits** — Position, velocity, and effort constraints
- **Inertials** — Mass and inertia for physics simulation

---

## Project: Basic Teleoperation

Build a keyboard-controlled humanoid:

1. **Create a ROS 2 package**
   ```bash
   ros2 pkg create --build-type ament_python humanoid_teleop
   ```

2. **Write a keyboard input node** that publishes to `/cmd_vel`

3. **Write a joint commander node** that subscribes and moves joints

4. **Launch both nodes** and control your robot in RViz2

**Success criteria:**

- [ ] Keyboard input moves robot joints
- [ ] Joint states published at 50 Hz
- [ ] Emergency stop on spacebar

---

## Key Takeaways

- **ROS 2 is the glue** that connects sensors, actuators, and algorithms into a functioning robot system
- **Nodes** are independent processes; **topics** enable streaming data; **services** handle request-response; **actions** manage long-running tasks
- **URDF** defines robot structure through links (bodies) and joints (connections)
- **Quality of Service (QoS)** settings balance reliability against latency for different use cases
- The **publish-subscribe pattern** decouples components, enabling modular system design

---

**Previous:** [← Introduction](../overview/introduction.md)

**Next:** [Module 2 — The Digital Twin →](../module-2-digital-twin/index.md)
