---
sidebar_position: 3
---

# Module 2: The Digital Twin (Gazebo & Unity)

> **Author:** Umema Sultan

---

## Learning Objectives

By the end of this module, you will be able to:

- Articulate why simulation-first development is the industry standard for robotics
- Configure Gazebo physics engines (ODE, Bullet, DART) for different simulation needs
- Design comprehensive sensor simulation with realistic noise models
- Implement parallel simulation environments for reinforcement learning
- Connect Unity to ROS 2 for photorealistic rendering and human-robot interaction
- Apply advanced domain randomization strategies for robust sim-to-real transfer
- Build and validate digital twin environments matching real-world conditions

---

## Introduction: The Simulation Imperative

In humanoid robotics, physical testing is expensive—both in time and hardware. A single fall can damage $100,000 worth of actuators. A control bug can shatter carbon fiber limbs. The solution is to develop in simulation first, where robots can fall millions of times without consequence.

But simulation is more than a safety net. Modern robotics treats simulation as a **first-class development environment**:

- **Parallel training**: Run thousands of robots simultaneously
- **Perfect repeatability**: Debug deterministic scenarios
- **Sensor ground truth**: Compare estimated vs. actual states
- **Accelerated time**: Train for years of experience in hours
- **Edge case generation**: Test scenarios impossible to create physically

The key insight is that **simulation fidelity is a spectrum**. You don't need perfect replication—you need sufficient fidelity that learned behaviors transfer. This module teaches you to build digital twins at the right fidelity level for your task.

---

## The Digital Twin Concept

A digital twin is not just a 3D model—it's a complete virtual replica that mirrors physical behavior:

```
┌─────────────────────────────────────────────────────────────────┐
│                        DIGITAL TWIN                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐         │
│  │  Geometric  │    │   Physics   │    │   Sensor    │         │
│  │    Model    │ ←→ │  Simulation │ ←→ │  Simulation │         │
│  │   (URDF)    │    │  (Dynamics) │    │   (Noise)   │         │
│  └─────────────┘    └─────────────┘    └─────────────┘         │
│         ↑                  ↑                  ↑                  │
│         └──────────────────┼──────────────────┘                  │
│                            │                                     │
│                    ┌───────▼───────┐                            │
│                    │  Environment  │                            │
│                    │    Model      │                            │
│                    └───────────────┘                            │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
                              ↕
              ┌───────────────────────────────┐
              │      PHYSICAL ROBOT           │
              │  (Synchronized for testing)   │
              └───────────────────────────────┘
```

### Fidelity Levels

| Level | Fidelity | Use Case | Tools |
|-------|----------|----------|-------|
| 1 | Kinematic only | Path planning, visualization | RViz, simple renderers |
| 2 | Rigid body dynamics | Contact, balance control | Gazebo, PyBullet |
| 3 | Deformable objects | Soft manipulation, cables | Gazebo/DART, SOFA |
| 4 | Photorealistic | Vision AI training | Isaac Sim, Unity |
| 5 | Multi-physics | Fluid, thermal interactions | Specialized solvers |

For humanoid locomotion, Level 2-3 is typically sufficient. For vision-based manipulation, Level 4 becomes important.

---

## Gazebo: The Physics Workhorse

Gazebo (now rebranded as Gazebo Sim, formerly Ignition Gazebo) is the standard simulator for ROS-based robotics.

### Architecture Overview

```
┌──────────────────────────────────────────────────────────┐
│                    Gazebo Architecture                    │
├──────────────────────────────────────────────────────────┤
│                                                          │
│   Server (gzserver)              Client (gzclient)       │
│   ┌────────────────────┐        ┌────────────────────┐  │
│   │  Physics Engine    │        │  3D Rendering      │  │
│   │  ├── ODE          │        │  (OGRE)            │  │
│   │  ├── Bullet       │◄──────►│                    │  │
│   │  ├── DART         │  TCP   │  GUI/Visualization │  │
│   │  └── Simbody      │        │                    │  │
│   │                    │        │                    │  │
│   │  Sensor Manager    │        │  Model Editor      │  │
│   │  Plugin System     │        │                    │  │
│   └────────────────────┘        └────────────────────┘  │
│            ↕                                             │
│   ┌────────────────────┐                                │
│   │   ROS 2 Bridge     │                                │
│   │   (gazebo_ros)     │                                │
│   └────────────────────┘                                │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

### Physics Engine Selection

| Engine | Strengths | Weaknesses | Best For |
|--------|-----------|------------|----------|
| **ODE** | Stable, well-tested | Slower, less accurate contacts | General purpose |
| **Bullet** | Fast, good constraints | Contact stability issues | Real-time sim |
| **DART** | Accurate articulated bodies | More complex setup | Humanoids, arms |
| **Simbody** | Biomechanics focused | Less general | Human modeling |

**Recommendation for Humanoids**: Use DART for best joint constraint handling:

```xml
<physics name="dart_physics" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <dart>
    <collision_detector>bullet</collision_detector>
    <solver>
      <solver_type>dantzig</solver_type>
    </solver>
  </dart>
</physics>
```

### Advanced World Configuration

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="humanoid_training_world">

    <!-- Physics Configuration -->
    <physics name="training_physics" type="dart">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>0</real_time_factor>  <!-- Run as fast as possible -->
      <real_time_update_rate>0</real_time_update_rate>
      <dart>
        <collision_detector>bullet</collision_detector>
      </dart>
    </physics>

    <!-- Gravity -->
    <gravity>0 0 -9.81</gravity>

    <!-- Magnetic Field (for compass simulation) -->
    <magnetic_field>5.5645e-6 22.8758e-6 -42.3884e-6</magnetic_field>

    <!-- Atmosphere (for drag simulation) -->
    <atmosphere type="adiabatic">
      <temperature>288.15</temperature>
      <pressure>101325</pressure>
    </atmosphere>

    <!-- Ground Plane with Friction -->
    <model name="ground">
      <static>true</static>
      <link name="ground_link">
        <collision name="ground_collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>
                <mu2>0.8</mu2>
              </ode>
              <bullet>
                <friction>0.8</friction>
                <friction2>0.8</friction2>
              </bullet>
            </friction>
            <contact>
              <ode>
                <kp>1e6</kp>
                <kd>100</kd>
              </ode>
            </contact>
          </surface>
        </collision>
        <visual name="ground_visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- ROS 2 Plugins -->
    <plugin filename="libgazebo_ros_init.so" name="gazebo_ros_init">
      <ros>
        <namespace>/humanoid_sim</namespace>
      </ros>
    </plugin>

    <plugin filename="libgazebo_ros_factory.so" name="gazebo_ros_factory"/>

  </world>
</sdf>
```

---

## Comprehensive Sensor Simulation

Accurate sensor simulation is crucial—your perception pipeline will only be as good as the training data.

### Sensor Noise Models

Real sensors have characteristic noise patterns:

| Sensor | Noise Type | Parameters |
|--------|------------|------------|
| IMU Gyroscope | Bias + White noise | Bias: 0.001 rad/s, σ: 0.0003 rad/s |
| IMU Accelerometer | Bias + White noise | Bias: 0.01 m/s², σ: 0.017 m/s² |
| Depth Camera | Quantization + Axial noise | σ = 0.0012z² |
| LiDAR | Gaussian range noise | σ: 0.01m |
| Force/Torque | Bias + Drift | σ: 0.1% full scale |

### High-Fidelity IMU Simulation

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>400</update_rate>
    <visualize>false</visualize>

    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0003</stddev>
            <bias_mean>0.001</bias_mean>
            <bias_stddev>0.0001</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0003</stddev>
            <bias_mean>0.001</bias_mean>
            <bias_stddev>0.0001</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0003</stddev>
            <bias_mean>0.001</bias_mean>
            <bias_stddev>0.0001</bias_stddev>
          </noise>
        </z>
      </angular_velocity>

      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.01</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </x>
        <!-- Similar for y, z -->
      </linear_acceleration>

      <orientation_reference_frame>
        <localization>GRAV_UP</localization>
        <grav_dir_x_parent_frame>0 0 -1</grav_dir_x_parent_frame>
      </orientation_reference_frame>
    </imu>

    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Depth Camera with Realistic Noise

```xml
<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <visualize>false</visualize>

    <camera>
      <horizontal_fov>1.211</horizontal_fov>  <!-- 69.4° like RealSense -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.005</stddev>
      </noise>
      <depth_camera>
        <clip>
          <near>0.28</near>  <!-- RealSense minimum -->
          <far>10.0</far>
        </clip>
      </depth_camera>
    </camera>

    <plugin name="depth_camera_plugin" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/humanoid</namespace>
      </ros>
      <camera_name>camera</camera_name>
      <frame_name>camera_optical_frame</frame_name>
      <hack_baseline>0.07</hack_baseline>  <!-- Stereo baseline -->
      <min_depth>0.28</min_depth>
      <max_depth>10.0</max_depth>
    </plugin>
  </sensor>
</gazebo>
```

### Force/Torque Sensors for Contact Detection

```xml
<gazebo reference="left_foot_ft_joint">
  <provideFeedback>true</provideFeedback>
</gazebo>

<gazebo>
  <plugin name="ft_sensor" filename="libgazebo_ros_ft_sensor.so">
    <ros>
      <namespace>/humanoid</namespace>
      <remapping>wrench:=left_foot/ft</remapping>
    </ros>
    <joint_name>left_foot_ft_joint</joint_name>
    <update_rate>1000</update_rate>
    <gaussian_noise>0.1</gaussian_noise>
  </plugin>
</gazebo>
```

---

## Parallel Simulation for RL Training

For reinforcement learning, you need thousands of environments running simultaneously:

### Gazebo Multi-Instance Setup

```python
import subprocess
import os
from multiprocessing import Pool

class ParallelGazeboManager:
    def __init__(self, num_envs: int, base_port: int = 11345):
        self.num_envs = num_envs
        self.base_port = base_port
        self.processes = []

    def launch_environments(self):
        """Launch multiple Gazebo instances with unique ports."""
        for i in range(self.num_envs):
            env = os.environ.copy()
            env['GAZEBO_MASTER_URI'] = f'http://localhost:{self.base_port + i}'
            env['ROS_DOMAIN_ID'] = str(i)

            proc = subprocess.Popen(
                ['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
                 'world.sdf'],
                env=env
            )
            self.processes.append(proc)

    def step_all(self, actions: list):
        """Send actions to all environments and collect observations."""
        # Use multiprocessing for parallel communication
        with Pool(self.num_envs) as pool:
            results = pool.starmap(self._step_single,
                                   zip(range(self.num_envs), actions))
        return results

    def reset_environment(self, env_id: int):
        """Reset a single environment."""
        # Call Gazebo reset service for specific environment
        pass
```

### Isaac Gym Alternative (Recommended for Scale)

For truly massive parallelization (4000+ environments), consider NVIDIA Isaac Gym:

```python
from isaacgym import gymapi, gymtorch
import torch

class HumanoidIsaacEnv:
    def __init__(self, num_envs: int = 4096):
        self.gym = gymapi.acquire_gym()
        self.num_envs = num_envs

        # Create simulation
        sim_params = gymapi.SimParams()
        sim_params.physx.num_threads = 4
        sim_params.physx.solver_type = 1  # TGS
        sim_params.physx.num_position_iterations = 4
        sim_params.physx.num_velocity_iterations = 1
        sim_params.physx.use_gpu = True

        self.sim = self.gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

        # Create environments
        self._create_envs()

    def _create_envs(self):
        """Create parallel environments."""
        asset_root = "assets"
        asset_file = "humanoid.urdf"

        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = False
        asset_options.angular_damping = 0.01

        humanoid_asset = self.gym.load_asset(
            self.sim, asset_root, asset_file, asset_options)

        # Create ground plane
        plane_params = gymapi.PlaneParams()
        plane_params.normal = gymapi.Vec3(0, 0, 1)
        self.gym.add_ground(self.sim, plane_params)

        # Spawn environments in a grid
        envs_per_row = int(np.sqrt(self.num_envs))
        env_spacing = 2.0

        self.envs = []
        self.actors = []

        for i in range(self.num_envs):
            env = self.gym.create_env(
                self.sim,
                gymapi.Vec3(-env_spacing, -env_spacing, 0),
                gymapi.Vec3(env_spacing, env_spacing, env_spacing),
                envs_per_row
            )

            pose = gymapi.Transform()
            pose.p = gymapi.Vec3(0, 0, 1.0)  # Start standing

            actor = self.gym.create_actor(env, humanoid_asset, pose, "humanoid", i, 1)
            self.envs.append(env)
            self.actors.append(actor)

    def step(self, actions: torch.Tensor):
        """Step all environments with GPU-accelerated physics."""
        # Apply actions
        self.gym.set_dof_actuation_force_tensor(
            self.sim,
            gymtorch.unwrap_tensor(actions)
        )

        # Simulate
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)

        # Get observations
        self.gym.refresh_dof_state_tensor(self.sim)
        self.gym.refresh_rigid_body_state_tensor(self.sim)

        return self._compute_observations(), self._compute_rewards()
```

---

## Unity for Photorealistic Rendering

When vision is critical, Unity provides unmatched rendering quality.

### Unity-ROS 2 Bridge Setup

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class HumanoidDigitalTwin : MonoBehaviour
{
    private ROSConnection ros;
    private ArticulationBody[] joints;
    private Dictionary<string, ArticulationBody> jointMap;

    // Publishers
    private string cameraTopic = "/unity/camera/image_raw";
    private string depthTopic = "/unity/camera/depth";

    // Subscribers
    private string jointStateTopic = "/joint_states";
    private string cmdVelTopic = "/cmd_vel";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Setup joint mapping
        joints = GetComponentsInChildren<ArticulationBody>();
        jointMap = new Dictionary<string, ArticulationBody>();
        foreach (var joint in joints)
        {
            jointMap[joint.name] = joint;
        }

        // Subscribe to joint states from Gazebo/real robot
        ros.Subscribe<JointStateMsg>(jointStateTopic, UpdateJointStates);

        // Subscribe to velocity commands for base motion
        ros.Subscribe<TwistMsg>(cmdVelTopic, UpdateBaseVelocity);

        // Register publishers for rendered images
        ros.RegisterPublisher<ImageMsg>(cameraTopic);
        ros.RegisterPublisher<ImageMsg>(depthTopic);

        // Start publishing camera images
        StartCoroutine(PublishCameraImages());
    }

    void UpdateJointStates(JointStateMsg msg)
    {
        for (int i = 0; i < msg.name.Length; i++)
        {
            if (jointMap.TryGetValue(msg.name[i], out ArticulationBody joint))
            {
                // Convert from ROS convention to Unity
                float angle = (float)msg.position[i] * Mathf.Rad2Deg;

                var drive = joint.xDrive;
                drive.target = angle;
                joint.xDrive = drive;
            }
        }
    }

    void UpdateBaseVelocity(TwistMsg msg)
    {
        // Apply base velocity for mobile humanoids
        var rootBody = GetComponent<ArticulationBody>();
        if (rootBody != null)
        {
            Vector3 linearVel = new Vector3(
                (float)msg.linear.x,
                (float)msg.linear.y,
                (float)msg.linear.z
            );
            rootBody.velocity = transform.TransformDirection(linearVel);
        }
    }

    IEnumerator PublishCameraImages()
    {
        Camera cam = GetComponentInChildren<Camera>();
        RenderTexture rt = new RenderTexture(640, 480, 24);

        while (true)
        {
            // Render to texture
            cam.targetTexture = rt;
            cam.Render();

            // Convert to ROS message
            RenderTexture.active = rt;
            Texture2D tex = new Texture2D(640, 480, TextureFormat.RGB24, false);
            tex.ReadPixels(new Rect(0, 0, 640, 480), 0, 0);
            tex.Apply();

            ImageMsg imageMsg = new ImageMsg();
            imageMsg.header.stamp = ROSConnection.GetTimeMsg();
            imageMsg.width = 640;
            imageMsg.height = 480;
            imageMsg.encoding = "rgb8";
            imageMsg.data = tex.GetRawTextureData();

            ros.Publish(cameraTopic, imageMsg);

            cam.targetTexture = null;
            RenderTexture.active = null;
            Destroy(tex);

            yield return new WaitForSeconds(0.033f);  // 30 FPS
        }
    }
}
```

### Synchronized Digital Twin

For real-time digital twin applications:

```csharp
public class SynchronizedTwin : MonoBehaviour
{
    [Header("Synchronization Settings")]
    public float syncRate = 100f;  // Hz
    public float predictionHorizon = 0.05f;  // 50ms lookahead

    private Queue<TimestampedState> stateBuffer;
    private float lastSyncTime;

    void Start()
    {
        stateBuffer = new Queue<TimestampedState>();
        ros.Subscribe<JointStateMsg>("/joint_states", BufferState);
    }

    void BufferState(JointStateMsg msg)
    {
        // Add to buffer with timestamp
        stateBuffer.Enqueue(new TimestampedState
        {
            timestamp = msg.header.stamp.ToDouble(),
            positions = msg.position,
            velocities = msg.velocity
        });

        // Keep buffer size manageable
        while (stateBuffer.Count > 100)
            stateBuffer.Dequeue();
    }

    void FixedUpdate()
    {
        // Interpolate/extrapolate to current time
        double currentTime = ROSConnection.GetTimeMsg().ToDouble();

        var state = InterpolateState(currentTime);
        ApplyState(state);
    }

    TimestampedState InterpolateState(double targetTime)
    {
        // Find bracketing states and interpolate
        // For low-latency, can extrapolate using velocities
        // ...
    }
}
```

---

## Domain Randomization: Bridging the Reality Gap

The "reality gap" is the difference between simulation and the real world. Domain randomization closes this gap by training on diverse simulated conditions.

### Randomization Strategies

```python
import numpy as np
from dataclasses import dataclass
from typing import Tuple

@dataclass
class DomainRandomizationConfig:
    # Physics
    mass_range: Tuple[float, float] = (0.8, 1.2)  # Multiplier
    friction_range: Tuple[float, float] = (0.5, 1.2)
    joint_damping_range: Tuple[float, float] = (0.5, 2.0)
    motor_strength_range: Tuple[float, float] = (0.8, 1.0)

    # Sensors
    imu_bias_range: Tuple[float, float] = (-0.02, 0.02)
    imu_noise_range: Tuple[float, float] = (0.001, 0.01)
    joint_encoder_noise: float = 0.001  # rad

    # Environment
    gravity_variation: float = 0.05  # ±5%
    ground_slope_range: Tuple[float, float] = (-5, 5)  # degrees

    # Visual (for vision-based policies)
    lighting_intensity_range: Tuple[float, float] = (0.3, 1.5)
    texture_randomization: bool = True
    camera_noise_stddev: float = 0.02


class DomainRandomizer:
    def __init__(self, config: DomainRandomizationConfig):
        self.config = config

    def randomize_physics(self, model):
        """Randomize physical parameters."""
        # Mass randomization
        for link in model.links:
            original_mass = link.inertial.mass
            link.inertial.mass = original_mass * np.random.uniform(
                *self.config.mass_range)

        # Friction randomization
        for collision in model.collisions:
            collision.surface.friction.mu = np.random.uniform(
                *self.config.friction_range)

        # Joint dynamics
        for joint in model.joints:
            joint.dynamics.damping *= np.random.uniform(
                *self.config.joint_damping_range)

        return model

    def randomize_sensors(self):
        """Return randomized sensor noise parameters."""
        return {
            'imu_bias': np.random.uniform(
                *self.config.imu_bias_range, size=6),
            'imu_noise': np.random.uniform(
                *self.config.imu_noise_range),
            'encoder_noise': self.config.joint_encoder_noise
        }

    def randomize_environment(self):
        """Generate randomized environment parameters."""
        gravity_z = -9.81 * (1 + np.random.uniform(
            -self.config.gravity_variation,
            self.config.gravity_variation))

        slope = np.random.uniform(*self.config.ground_slope_range)

        return {
            'gravity': [0, 0, gravity_z],
            'ground_slope_deg': slope
        }

    def randomize_visual(self):
        """Generate visual randomization parameters."""
        return {
            'light_intensity': np.random.uniform(
                *self.config.lighting_intensity_range),
            'light_direction': np.random.randn(3),
            'randomize_textures': self.config.texture_randomization
        }
```

### Curriculum-Based Randomization

Start with easy conditions and progressively increase difficulty:

```python
class CurriculumRandomizer:
    def __init__(self, base_config: DomainRandomizationConfig):
        self.base_config = base_config
        self.difficulty = 0.0  # 0 to 1

    def update_difficulty(self, success_rate: float, threshold: float = 0.7):
        """Adjust difficulty based on agent performance."""
        if success_rate > threshold:
            self.difficulty = min(1.0, self.difficulty + 0.1)
        elif success_rate < threshold - 0.2:
            self.difficulty = max(0.0, self.difficulty - 0.05)

    def get_randomization_range(self, base_range: Tuple[float, float]) -> Tuple[float, float]:
        """Scale randomization range by difficulty."""
        center = (base_range[0] + base_range[1]) / 2
        half_width = (base_range[1] - base_range[0]) / 2

        scaled_width = half_width * self.difficulty
        return (center - scaled_width, center + scaled_width)

    def randomize(self):
        """Get parameters scaled by current difficulty."""
        friction_range = self.get_randomization_range(
            self.base_config.friction_range)
        slope_range = self.get_randomization_range(
            self.base_config.ground_slope_range)

        return {
            'friction': np.random.uniform(*friction_range),
            'slope': np.random.uniform(*slope_range),
            # ... other parameters
        }
```

---

## Building Complete Environments

### Indoor Navigation Environment

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="indoor_nav">

    <!-- Include physics, lighting from base world -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Floor -->
    <model name="floor">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>20 20 0.1</size></box>
          </geometry>
          <surface>
            <friction>
              <ode><mu>0.8</mu></ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>20 20 0.1</size></box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <!-- Walls -->
    <model name="walls">
      <static>true</static>
      <!-- Room 1 -->
      <link name="wall_north">
        <pose>0 5 1.25 0 0 0</pose>
        <collision name="collision">
          <geometry><box><size>10 0.2 2.5</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>10 0.2 2.5</size></box></geometry>
          <material>
            <ambient>0.9 0.9 0.9 1</ambient>
          </material>
        </visual>
      </link>
      <!-- Add more walls, doors, etc. -->
    </model>

    <!-- Furniture -->
    <include>
      <uri>model://table</uri>
      <pose>2 2 0 0 0 0</pose>
    </include>

    <include>
      <uri>model://chair</uri>
      <pose>2 1 0 0 0 1.57</pose>
    </include>

    <!-- Dynamic obstacles (humans) -->
    <actor name="walking_human">
      <skin>
        <filename>walk.dae</filename>
      </skin>
      <animation name="walking">
        <filename>walk.dae</filename>
      </animation>
      <script>
        <trajectory id="0" type="walking">
          <waypoint>
            <time>0</time>
            <pose>-3 0 0 0 0 0</pose>
          </waypoint>
          <waypoint>
            <time>5</time>
            <pose>3 0 0 0 0 0</pose>
          </waypoint>
        </trajectory>
      </script>
    </actor>

  </world>
</sdf>
```

---

## Capstone Project: Multi-Terrain Training Environment

Build a comprehensive training environment for humanoid locomotion:

### Requirements

1. **Terrain Variety**
   - Flat ground with varying friction (0.3 - 1.0)
   - Stairs (15-20cm rise, 25-30cm run)
   - Slopes (5-15 degrees)
   - Rough terrain with height variations

2. **Sensor Suite**
   - IMU at 400 Hz with realistic noise
   - Joint encoders at 1000 Hz
   - Depth camera at 30 Hz
   - Foot contact sensors at 1000 Hz

3. **Randomization**
   - Physics parameters per episode
   - Terrain type selection
   - Perturbation forces (push recovery testing)

4. **Parallel Execution**
   - Support for 16+ simultaneous environments
   - Deterministic reset capability

### Implementation Checklist

- [ ] Create modular terrain tiles that can be combined
- [ ] Implement terrain height map generation with Perlin noise
- [ ] Add stair generator with configurable dimensions
- [ ] Configure all sensor plugins with calibrated noise models
- [ ] Build domain randomization system with curriculum learning
- [ ] Create reset service that randomizes initial robot pose
- [ ] Implement external force plugin for perturbation testing
- [ ] Set up rosbag2 recording for offline analysis
- [ ] Validate sensor outputs against real hardware specifications
- [ ] Benchmark simulation speed (target: 10x real-time minimum)

---

## Key Takeaways

- **Simulation is not optional** for modern humanoid development—it's where you iterate fastest and safest

- **Choose physics engine by task**: DART for articulated humanoids, ODE for general purpose, Isaac Gym for massive parallelization

- **Sensor fidelity matters**: Match real sensor noise characteristics or your perception pipeline will fail on hardware

- **Parallel simulation** enables reinforcement learning at scale—thousands of robots training simultaneously

- **Domain randomization** bridges the reality gap by exposing policies to diverse conditions during training

- **Unity complements Gazebo**: Use Gazebo for physics accuracy, Unity for photorealistic vision training

- **Digital twins enable** safe testing, rapid iteration, and continuous validation of deployed systems

---

## Further Reading

- [Gazebo Sim Documentation](https://gazebosim.org/docs)
- [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [Domain Randomization for Sim-to-Real Transfer](https://arxiv.org/abs/1703.06907)
- [Isaac Gym Paper](https://arxiv.org/abs/2108.10470)

---

**Previous:** [← Module 1 — ROS 2](../module-1-ros2/index.md)

**Next:** [Module 3 — NVIDIA Isaac →](../module-3-ai-brain/index.md)
