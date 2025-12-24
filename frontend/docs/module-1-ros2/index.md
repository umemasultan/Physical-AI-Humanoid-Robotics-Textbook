---
sidebar_position: 2
---

# Module 1: The Robotic Nervous System (ROS 2)

> **Author:** Umema Sultan

---

## Learning Objectives

By the end of this module, you will be able to:

- Explain the architectural principles behind ROS 2 and why it supersedes ROS 1
- Master the four communication paradigms: nodes, topics, services, and actions
- Design and implement distributed robotic systems with proper QoS configuration
- Create comprehensive URDF/XACRO models for humanoid robots
- Build production-ready bridges between AI agents and robot hardware
- Implement lifecycle management for robust node operation

---

## Introduction: Why Middleware Matters

Imagine building a humanoid robot from scratch. You have cameras streaming video at 30 FPS, IMUs reporting orientation at 200 Hz, force sensors on each foot updating at 1000 Hz, and 20+ joint motors awaiting commands. How do you coordinate this chaos?

This is the middleware problem. Without a unifying framework, you'd spend more time writing communication code than building intelligence. ROS 2 (Robot Operating System 2) solves this by providing:

- **Standardized communication** between heterogeneous components
- **Hardware abstraction** so algorithms work across different robots
- **Time synchronization** for sensor fusion
- **A rich ecosystem** of tools, libraries, and community packages

ROS 2 is not an operating system—it's middleware that runs on top of Linux, Windows, or macOS. Think of it as the nervous system of your robot: it doesn't make decisions, but it ensures every part can communicate with every other part.

---

## The Evolution: ROS 1 to ROS 2

### Why ROS 2 Exists

ROS 1, developed at Willow Garage in 2007, revolutionized robotics research. But it had fundamental limitations:

| Limitation | ROS 1 Approach | ROS 2 Solution |
|------------|----------------|----------------|
| **Single point of failure** | Central rosmaster | Decentralized discovery (DDS) |
| **No real-time support** | Best-effort only | Real-time capable with rmw |
| **Security** | None | DDS-Security (authentication, encryption) |
| **Multi-robot** | Complex namespacing | Native multi-robot support |
| **Embedded systems** | Linux only | Cross-platform (MCUs to cloud) |

### The DDS Foundation

ROS 2 is built on the Data Distribution Service (DDS), an industry standard for real-time publish-subscribe communication. DDS provides:

- **Automatic discovery**: Nodes find each other without a central broker
- **Quality of Service (QoS)**: Fine-grained control over reliability, durability, and latency
- **Security**: Built-in authentication and encryption
- **Interoperability**: Multiple DDS vendors (Fast DDS, Cyclone DDS, Connext)

```
┌─────────────────────────────────────────────────────────┐
│                    Your Application                      │
├─────────────────────────────────────────────────────────┤
│                      ROS 2 Client Library (rclpy/rclcpp)│
├─────────────────────────────────────────────────────────┤
│                      RMW (ROS Middleware Interface)     │
├─────────────────────────────────────────────────────────┤
│                      DDS Implementation                  │
├─────────────────────────────────────────────────────────┤
│                      UDP/Shared Memory                   │
└─────────────────────────────────────────────────────────┘
```

---

## Core Architecture: The Four Pillars

### 1. Nodes: The Computational Units

A node is a single-purpose process that performs one specific task. Good node design follows the Unix philosophy: do one thing well.

**Example: A perception pipeline as separate nodes**

```
camera_driver → image_preprocessing → object_detector → tracker
     │                  │                    │             │
   Node 1            Node 2              Node 3        Node 4
```

Each node can:
- Crash without bringing down the entire system
- Be restarted independently
- Run on different machines
- Be written in different languages

**Creating a Node in Python:**

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Declare parameters with defaults
        self.declare_parameter('detection_threshold', 0.5)
        self.declare_parameter('model_path', '/models/yolo.pt')

        # Get parameter values
        self.threshold = self.get_parameter('detection_threshold').value

        # Create a timer for periodic processing
        self.timer = self.create_timer(0.033, self.process_frame)  # 30 Hz

        self.get_logger().info(f'Perception node initialized with threshold: {self.threshold}')

    def process_frame(self):
        # Processing logic here
        pass

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()

    # Use multi-threaded executor for concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Topics: Streaming Data Channels

Topics implement the publish-subscribe pattern for continuous data streams. Publishers send messages without knowing who's listening; subscribers receive messages without knowing who's sending.

**Standard Message Types for Humanoids:**

| Topic Name | Message Type | Typical Rate | Description |
|------------|--------------|--------------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | 100-1000 Hz | Current position, velocity, effort of all joints |
| `/imu/data` | `sensor_msgs/Imu` | 100-400 Hz | Orientation, angular velocity, linear acceleration |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 30-60 Hz | RGB camera stream |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 30 Hz | Depth image |
| `/cmd_vel` | `geometry_msgs/Twist` | 50-100 Hz | Velocity commands |
| `/joint_commands` | `trajectory_msgs/JointTrajectory` | 100+ Hz | Joint position/velocity commands |
| `/foot_contacts` | `std_msgs/Bool[4]` | 1000 Hz | Ground contact sensors |
| `/tf` | `tf2_msgs/TFMessage` | 100+ Hz | Coordinate frame transforms |

**Publisher Example:**

```python
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')

        # Configure QoS for high-frequency sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Only keep latest message
        )

        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            sensor_qos
        )

        self.timer = self.create_timer(0.001, self.publish_joints)  # 1000 Hz

    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['hip_pitch', 'hip_roll', 'knee', 'ankle_pitch', 'ankle_roll']
        msg.position = [0.0, 0.0, 0.5, -0.25, 0.0]
        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.effort = [10.0, 5.0, 20.0, 15.0, 5.0]
        self.publisher.publish(msg)
```

### 3. Services: Request-Response Operations

Services are for synchronous, one-time operations where you need a response.

**When to use services vs topics:**

| Use Services | Use Topics |
|--------------|------------|
| Changing robot mode | Streaming sensor data |
| Requesting current state | Continuous commands |
| Calibration triggers | Feedback during motion |
| Configuration changes | Periodic status updates |

**Service Example: Emergency Stop**

```python
from std_srvs.srv import SetBool
from rclpy.callback_groups import ReentrantCallbackGroup

class SafetyController(Node):
    def __init__(self):
        super().__init__('safety_controller')

        self.callback_group = ReentrantCallbackGroup()

        self.estop_service = self.create_service(
            SetBool,
            '/emergency_stop',
            self.estop_callback,
            callback_group=self.callback_group
        )

        self.is_stopped = False

    def estop_callback(self, request, response):
        self.is_stopped = request.data

        if self.is_stopped:
            self.get_logger().error('EMERGENCY STOP ACTIVATED')
            # Immediately command zero velocity to all joints
            self.stop_all_motors()
        else:
            self.get_logger().info('Emergency stop released')

        response.success = True
        response.message = f'Emergency stop {"activated" if self.is_stopped else "released"}'
        return response
```

### 4. Actions: Long-Running Tasks with Feedback

Actions handle tasks that take time and need progress updates. They're essential for humanoid motion planning.

**Action Structure:**

```
Goal → Server accepts/rejects
         ↓
      Feedback (continuous)
         ↓
      Result (final)
```

**Walking Action Example:**

```python
from rclpy.action import ActionServer
from humanoid_msgs.action import Walk

class WalkingController(Node):
    def __init__(self):
        super().__init__('walking_controller')

        self._action_server = ActionServer(
            self,
            Walk,
            'walk_to_pose',
            execute_callback=self.execute_walk,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):
        """Decide whether to accept or reject the goal."""
        self.get_logger().info(f'Received walk request to: {goal_request.target_pose}')

        # Check if target is reachable
        if self.is_pose_reachable(goal_request.target_pose):
            return GoalResponse.ACCEPT
        return GoalResponse.REJECT

    async def execute_walk(self, goal_handle):
        """Execute the walking action."""
        self.get_logger().info('Starting walk execution')

        feedback_msg = Walk.Feedback()

        while not self.reached_target():
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Walk.Result(success=False)

            # Update feedback
            feedback_msg.current_pose = self.get_current_pose()
            feedback_msg.distance_remaining = self.get_distance_to_target()
            feedback_msg.steps_taken = self.step_count
            goal_handle.publish_feedback(feedback_msg)

            # Execute one step
            await self.take_step()

        goal_handle.succeed()
        return Walk.Result(success=True, final_pose=self.get_current_pose())
```

---

## Quality of Service (QoS): Fine-Tuning Communication

QoS profiles let you balance reliability against latency for different data types.

### QoS Policies Explained

| Policy | Options | Use Case |
|--------|---------|----------|
| **Reliability** | RELIABLE / BEST_EFFORT | RELIABLE for commands, BEST_EFFORT for sensors |
| **Durability** | TRANSIENT_LOCAL / VOLATILE | TRANSIENT_LOCAL for late-joining subscribers |
| **History** | KEEP_LAST(n) / KEEP_ALL | KEEP_LAST(1) for real-time, KEEP_ALL for logging |
| **Deadline** | Duration | Detect sensor failures |
| **Lifespan** | Duration | Expire old messages |
| **Liveliness** | AUTOMATIC / MANUAL | Detect node crashes |

### Recommended QoS Profiles

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# For high-frequency sensor data (IMU, joint states)
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE
)

# For critical commands (joint commands, emergency stop)
COMMAND_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE
)

# For configuration/parameters that late-joiners need
CONFIG_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)
```

---

## URDF & XACRO: Modeling Your Humanoid

### URDF Fundamentals

URDF (Unified Robot Description Format) is an XML format describing robot kinematics and dynamics.

**Complete Humanoid Leg Definition:**

```xml
<?xml version="1.0"?>
<robot name="humanoid_leg" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Materials -->
  <material name="aluminum">
    <color rgba="0.7 0.7 0.7 1.0"/>
  </material>

  <!-- Hip Link -->
  <link name="hip">
    <visual>
      <geometry>
        <mesh filename="package://humanoid_description/meshes/hip.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="aluminum"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.5"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Hip Pitch Joint -->
  <joint name="hip_pitch" type="revolute">
    <parent link="hip"/>
    <child link="thigh"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0.5" effort="150" velocity="5.0"/>
    <dynamics damping="0.1" friction="0.05"/>
  </joint>

  <!-- Thigh Link -->
  <link name="thigh">
    <visual>
      <origin xyz="0 0 -0.2"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
      <material name="aluminum"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Knee Joint -->
  <joint name="knee" type="revolute">
    <parent link="thigh"/>
    <child link="shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="150" velocity="6.0"/>
    <dynamics damping="0.1" friction="0.05"/>
  </joint>

  <!-- Continue with shin, ankle, foot... -->

</robot>
```

### XACRO: Macros for Complex Robots

XACRO (XML Macros) makes URDF manageable for complex humanoids:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">

  <!-- Properties -->
  <xacro:property name="thigh_length" value="0.4"/>
  <xacro:property name="shin_length" value="0.4"/>
  <xacro:property name="motor_effort" value="150"/>

  <!-- Leg Macro -->
  <xacro:macro name="leg" params="side parent *origin">
    <xacro:property name="sign" value="${1 if side == 'left' else -1}"/>

    <joint name="${side}_hip_yaw" type="revolute">
      <xacro:insert_block name="origin"/>
      <parent link="${parent}"/>
      <child link="${side}_hip_yaw_link"/>
      <axis xyz="0 0 1"/>
      <limit lower="-0.5" upper="0.5" effort="${motor_effort}" velocity="5.0"/>
    </joint>

    <!-- Continue with full leg chain -->
  </xacro:macro>

  <!-- Instantiate both legs -->
  <xacro:leg side="left" parent="pelvis">
    <origin xyz="0 0.1 0" rpy="0 0 0"/>
  </xacro:leg>

  <xacro:leg side="right" parent="pelvis">
    <origin xyz="0 -0.1 0" rpy="0 0 0"/>
  </xacro:leg>

</robot>
```

---

## Lifecycle Nodes: Production-Ready State Management

For robust humanoid systems, use managed lifecycle nodes:

```
┌──────────────────────────────────────────────────────────┐
│                    Lifecycle States                       │
├──────────────────────────────────────────────────────────┤
│                                                          │
│   Unconfigured ──configure──→ Inactive                   │
│        ↑                          │                      │
│        │                      activate                   │
│     cleanup                       │                      │
│        │                          ↓                      │
│   Inactive ←──deactivate─── Active                       │
│                                   │                      │
│                               shutdown                   │
│                                   ↓                      │
│                              Finalized                   │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

```python
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

class ManagedMotorController(LifecycleNode):
    def __init__(self):
        super().__init__('motor_controller')
        self.motors = None

    def on_configure(self, state):
        """Load configuration, connect to hardware."""
        self.get_logger().info('Configuring motor controller...')

        try:
            self.motors = MotorInterface(self.get_parameter('can_bus').value)
            self.motors.connect()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Configuration failed: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        """Enable motors, start control loop."""
        self.get_logger().info('Activating motors...')
        self.motors.enable()
        self.control_timer = self.create_timer(0.001, self.control_loop)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """Stop control, disable motors safely."""
        self.get_logger().info('Deactivating motors...')
        self.control_timer.cancel()
        self.motors.disable()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        """Release resources."""
        self.motors.disconnect()
        self.motors = None
        return TransitionCallbackReturn.SUCCESS
```

---

## AI-Robot Bridge Architecture

Connecting AI agents to ROS 2 requires careful design:

```
┌─────────────────────────────────────────────────────────────────┐
│                        AI Agent Layer                            │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │
│  │ LLM Planner │  │   Vision    │  │   Policy    │              │
│  │             │  │   Model     │  │   Network   │              │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘              │
│         │                │                │                      │
│         └────────────────┼────────────────┘                      │
│                          │                                       │
├──────────────────────────┼───────────────────────────────────────┤
│                          │                                       │
│              ┌───────────▼───────────┐                          │
│              │    ROS 2 Bridge Node   │                          │
│              │  • Command translation │                          │
│              │  • State aggregation   │                          │
│              │  • Safety filtering    │                          │
│              └───────────┬───────────┘                          │
│                          │                                       │
├──────────────────────────┼───────────────────────────────────────┤
│                    ROS 2 Layer                                   │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐            │
│  │ /cmd_vel│  │/joint_  │  │ /camera │  │ /imu    │            │
│  │         │  │ states  │  │         │  │         │            │
│  └─────────┘  └─────────┘  └─────────┘  └─────────┘            │
└─────────────────────────────────────────────────────────────────┘
```

**Complete Bridge Implementation:**

```python
import numpy as np
from dataclasses import dataclass
from typing import Optional, Callable
import threading

@dataclass
class RobotState:
    """Aggregated robot state for AI consumption."""
    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    imu_orientation: np.ndarray  # quaternion
    imu_angular_velocity: np.ndarray
    foot_contacts: np.ndarray  # [left_heel, left_toe, right_heel, right_toe]
    timestamp: float

class AIBridge(Node):
    def __init__(self, state_callback: Callable[[RobotState], None]):
        super().__init__('ai_bridge')

        self.state_callback = state_callback
        self.state_lock = threading.Lock()

        # State storage
        self._joint_positions = np.zeros(20)
        self._joint_velocities = np.zeros(20)
        self._imu_orientation = np.array([0, 0, 0, 1])
        self._imu_angular_velocity = np.zeros(3)
        self._foot_contacts = np.zeros(4)

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_callback, SENSOR_QOS)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self._imu_callback, SENSOR_QOS)
        self.contact_sub = self.create_subscription(
            ContactsStamped, '/foot_contacts', self._contact_callback, SENSOR_QOS)

        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            JointTrajectoryPoint, '/joint_commands', COMMAND_QOS)

        # State aggregation timer (50 Hz to AI)
        self.state_timer = self.create_timer(0.02, self._publish_state)

    def _joint_callback(self, msg):
        with self.state_lock:
            self._joint_positions = np.array(msg.position)
            self._joint_velocities = np.array(msg.velocity)

    def _imu_callback(self, msg):
        with self.state_lock:
            q = msg.orientation
            self._imu_orientation = np.array([q.x, q.y, q.z, q.w])
            av = msg.angular_velocity
            self._imu_angular_velocity = np.array([av.x, av.y, av.z])

    def _contact_callback(self, msg):
        with self.state_lock:
            self._foot_contacts = np.array([c.force > 10.0 for c in msg.contacts])

    def _publish_state(self):
        """Aggregate state and send to AI."""
        with self.state_lock:
            state = RobotState(
                joint_positions=self._joint_positions.copy(),
                joint_velocities=self._joint_velocities.copy(),
                imu_orientation=self._imu_orientation.copy(),
                imu_angular_velocity=self._imu_angular_velocity.copy(),
                foot_contacts=self._foot_contacts.copy(),
                timestamp=self.get_clock().now().nanoseconds / 1e9
            )

        self.state_callback(state)

    def send_joint_command(self, positions: np.ndarray, velocities: np.ndarray):
        """Send joint command from AI to robot."""
        # Safety checks
        positions = np.clip(positions, self.joint_limits_lower, self.joint_limits_upper)

        msg = JointTrajectoryPoint()
        msg.positions = positions.tolist()
        msg.velocities = velocities.tolist()
        msg.time_from_start.sec = 0
        msg.time_from_start.nanosec = 10_000_000  # 10ms

        self.joint_cmd_pub.publish(msg)
```

---

## Launch Files: Orchestrating Complex Systems

Modern ROS 2 uses Python launch files:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Declare arguments
    use_sim = DeclareLaunchArgument('use_sim', default_value='true')
    robot_name = DeclareLaunchArgument('robot_name', default_value='humanoid')

    # Robot description
    robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', FindPackageShare('humanoid_description'), '/urdf/humanoid.urdf.xacro'])
        }]
    )

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            FindPackageShare('humanoid_control') + '/config/controllers.yaml'
        ],
        condition=IfCondition(LaunchConfiguration('use_sim'))
    )

    # Perception stack
    perception = GroupAction([
        PushRosNamespace('perception'),
        Node(package='humanoid_perception', executable='camera_processor'),
        Node(package='humanoid_perception', executable='object_detector'),
        Node(package='humanoid_perception', executable='human_tracker'),
    ])

    # AI bridge
    ai_bridge = Node(
        package='humanoid_ai',
        executable='ai_bridge',
        parameters=[{
            'model_path': '/models/policy.pt',
            'control_frequency': 50.0
        }]
    )

    return LaunchDescription([
        use_sim,
        robot_name,
        robot_description,
        controller_manager,
        perception,
        ai_bridge,
    ])
```

---

## Capstone Project: Complete Teleoperation System

Build a production-quality teleoperation system:

### Project Requirements

1. **Keyboard teleop node** with velocity ramping
2. **Joint commander** with safety limits
3. **State visualizer** in RViz2
4. **Emergency stop** service
5. **Data logger** for replay

### Implementation Checklist

- [ ] Create ROS 2 package with proper dependencies
- [ ] Implement keyboard input with non-blocking reads
- [ ] Design velocity command message with linear and angular components
- [ ] Add acceleration limits for smooth motion
- [ ] Implement joint limit checking before command execution
- [ ] Create RViz2 config showing robot model and joint states
- [ ] Add emergency stop service that immediately zeros all commands
- [ ] Implement rosbag2 logging with configurable topics
- [ ] Write launch file orchestrating all nodes
- [ ] Add parameter file for tuning without recompilation

---

## Key Takeaways

- **ROS 2 is the foundation** of modern robotics software—it handles communication, timing, and hardware abstraction so you can focus on intelligence

- **DDS provides enterprise-grade communication** with automatic discovery, configurable QoS, and built-in security

- **The four communication patterns** serve different needs:
  - **Nodes**: Independent, single-purpose processes
  - **Topics**: Streaming sensor data and continuous commands
  - **Services**: Synchronous request-response operations
  - **Actions**: Long-running tasks with feedback

- **QoS profiles are critical** for balancing reliability vs. latency across different data types

- **URDF/XACRO** define robot structure; accurate models are essential for simulation and visualization

- **Lifecycle nodes** enable robust, production-ready systems with proper state management

- **AI integration** requires careful bridge design handling state aggregation, command translation, and safety filtering

---

## Further Reading

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [DDS Specification](https://www.omg.org/spec/DDS/)
- [URDF Tutorials](https://wiki.ros.org/urdf/Tutorials)
- [ros2_control Framework](https://control.ros.org/)

---

**Previous:** [← Introduction](../overview/introduction.md)

**Next:** [Module 2 — The Digital Twin →](../module-2-digital-twin/index.md)
