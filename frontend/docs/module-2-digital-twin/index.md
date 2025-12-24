---
sidebar_position: 3
---

# Module 2: The Digital Twin (Gazebo & Unity)

> **Author:** Umema Sultan

---

## Learning Objectives

By the end of this module, you will be able to:

- Explain why simulation is essential for robot development
- Configure Gazebo physics parameters and spawn robots in simulated worlds
- Add sensor plugins (LiDAR, depth camera, IMU) to robot models
- Connect Unity to ROS 2 for high-fidelity rendering scenarios
- Apply domain randomization to improve sim-to-real transfer

---

Simulation is where you break things safely. A digital twin replicates your robot and environment with enough fidelity that behaviors transfer to the real world. This module covers physics simulation in Gazebo and high-fidelity rendering in Unity.

---

## Gazebo: Physics-First Simulation

Gazebo is the standard ROS 2 simulator—open source, tightly integrated, and physics-accurate.

### Setting Up a World

```xml
<!-- humanoid_world.sdf -->
<world name="humanoid_world">
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>

  <gravity>0 0 -9.81</gravity>

  <include>
    <uri>model://ground_plane</uri>
  </include>

  <include>
    <uri>model://sun</uri>
  </include>
</world>
```

### Spawning Your Robot

```bash
ros2 launch gazebo_ros gazebo.launch.py world:=humanoid_world.sdf
ros2 run gazebo_ros spawn_entity.py -file humanoid.urdf -entity humanoid
```

### Physics Configuration

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `max_step_size` | Simulation timestep | 0.001s |
| `real_time_factor` | Speed vs. real time | 1.0 |
| `friction` | Surface grip | 0.5–1.0 |
| `restitution` | Bounciness | 0.0–0.5 |

---

## Simulating Sensors

Add realistic sensor models to your URDF:

### LiDAR

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar">
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
      </range>
    </ray>
    <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=/scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

### Depth Camera

```xml
<gazebo reference="camera_link">
  <sensor type="depth" name="depth_camera">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>
    <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/humanoid</namespace>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### IMU

```xml
<gazebo reference="imu_link">
  <sensor type="imu" name="imu">
    <imu>
      <angular_velocity>
        <x><noise type="gaussian"><stddev>0.01</stddev></noise></x>
      </angular_velocity>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <remapping>~/out:=/imu/data</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

---

## Unity: High-Fidelity Rendering

When you need photorealism or complex human-robot interaction scenarios:

### ROS-Unity Bridge

```csharp
// Unity C# - Subscribe to robot state
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class RobotVisualizer : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance()
            .Subscribe<JointStateMsg>("/joint_states", UpdateJoints);
    }

    void UpdateJoints(JointStateMsg msg)
    {
        // Apply joint positions to Unity model
        for (int i = 0; i < msg.position.Length; i++)
        {
            SetJointAngle(msg.name[i], msg.position[i]);
        }
    }
}
```

### When to Use Unity vs. Gazebo

| Use Case | Gazebo | Unity |
|----------|--------|-------|
| Physics accuracy | ✓ | — |
| ROS integration | Native | Bridge |
| Photorealism | — | ✓ |
| Human avatars | Limited | ✓ |
| VR/AR testing | — | ✓ |

---

## Domain Randomization

Improve sim-to-real transfer by varying:

- **Lighting** — Direction, intensity, color temperature
- **Textures** — Surface materials and patterns
- **Object placement** — Position and orientation noise
- **Sensor noise** — Gaussian noise matching real hardware
- **Physics parameters** — Friction, mass variations

```python
# Randomize lighting in each episode
light.intensity = random.uniform(0.5, 1.5)
light.color = random_color_temperature(4000, 6500)
```

---

## Project: Indoor Navigation Environment

Build a simulation for humanoid navigation:

1. **Create an indoor environment**
   - Walls, doors, furniture
   - Multiple rooms with corridors

2. **Add your humanoid with sensors**
   - Head-mounted depth camera
   - Torso IMU
   - Optional: LiDAR on base

3. **Record test data**
   ```bash
   ros2 bag record /scan /camera/depth /imu/data /tf
   ```

4. **Validate sensor outputs in RViz2**

**Success criteria:**

- [ ] Robot spawns without collision errors
- [ ] Depth camera shows furniture/walls
- [ ] IMU responds to simulated motion
- [ ] 30+ FPS simulation performance

---

## Key Takeaways

- **Simulation enables safe experimentation**—robots can fall thousands of times without damage
- **Gazebo** provides physics-accurate simulation with native ROS 2 integration
- **Unity** offers photorealistic rendering for vision-based tasks and human-robot interaction
- **Sensor simulation** must include realistic noise models to prepare for real-world deployment
- **Domain randomization** varies simulation parameters to build policies that transfer to reality

---

**Previous:** [← Module 1 — ROS 2](../module-1-ros2/index.md)

**Next:** [Module 3 — NVIDIA Isaac →](../module-3-ai-brain/index.md)
