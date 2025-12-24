---
sidebar_position: 4
---

# Module 3: The AI-Robot Brain (NVIDIA Isaac)

> **Author:** Umema Sultan

---

## Learning Objectives

By the end of this module, you will be able to:

- Articulate the paradigm shift that GPU-accelerated robotics enables
- Configure Isaac Sim for photorealistic simulation with RTX ray tracing
- Generate synthetic datasets with automatic ground-truth labeling
- Deploy Isaac ROS perception modules for real-time visual SLAM and 3D mapping
- Train humanoid locomotion policies using Isaac Gym's massively parallel environments
- Design hybrid navigation systems combining classical planning with learned behaviors
- Implement production-grade perception pipelines with sensor fusion

---

## Introduction: The GPU Revolution in Robotics

Traditional robotics runs on CPUs—serial processors that handle one computation at a time. This was sufficient for simple control loops, but modern robotics demands more:

- **Perception**: Processing megapixels of image data at 30-60 FPS
- **Simulation**: Computing physics for thousands of parallel environments
- **Learning**: Training neural networks with billions of parameters
- **Planning**: Searching vast action spaces in real-time

GPUs change the equation. With thousands of parallel cores, operations that took hours on CPUs complete in minutes. NVIDIA's Isaac platform brings this power to robotics through three pillars:

```
┌────────────────────────────────────────────────────────────────┐
│                    NVIDIA Isaac Platform                        │
├────────────────────────────────────────────────────────────────┤
│                                                                 │
│   Isaac Sim            Isaac ROS           Isaac Gym            │
│   ┌─────────────┐     ┌─────────────┐     ┌─────────────┐      │
│   │ Photorealistic│    │ GPU-Accel   │     │  Parallel   │      │
│   │  Simulation  │     │ Perception  │     │  RL Train   │      │
│   │             │     │             │     │             │      │
│   │ • RTX render │     │ • cuVSLAM   │     │ • 4096 envs │      │
│   │ • Synthetic  │     │ • nvblox    │     │ • PPO/SAC   │      │
│   │   data gen   │     │ • Detection │     │ • Sim2Real  │      │
│   └─────────────┘     └─────────────┘     └─────────────┘      │
│         ↓                   ↓                   ↓               │
│   ┌─────────────────────────────────────────────────────┐      │
│   │              Omniverse Platform (USD)                │      │
│   │         Universal Scene Description Format           │      │
│   └─────────────────────────────────────────────────────┘      │
│                                                                 │
└────────────────────────────────────────────────────────────────┘
```

---

## Isaac Sim: Photorealistic Robot Simulation

Isaac Sim, built on NVIDIA Omniverse, delivers physically accurate rendering that enables vision models to transfer directly to real cameras.

### Why Photorealism Matters

Training vision models on synthetic data only works if that data looks like reality. Traditional simulators produce images that look obviously fake—uniform lighting, perfect textures, no sensor noise. Models trained on such data fail on real cameras.

Isaac Sim solves this with:

| Feature | Traditional Sim | Isaac Sim |
|---------|----------------|-----------|
| **Rendering** | Rasterization | RTX Ray Tracing |
| **Lighting** | Baked/Simple | Global Illumination |
| **Materials** | Flat textures | Physically-based (PBR) |
| **Reflections** | Screen-space | True ray-traced |
| **Shadows** | Shadow maps | Ray-traced soft shadows |

### Installation and Setup

```bash
# System requirements
# - NVIDIA RTX GPU (3070+ recommended)
# - Ubuntu 20.04/22.04 or Windows
# - 32GB+ RAM, 500GB+ SSD

# Install via Omniverse Launcher
# Download from: https://www.nvidia.com/en-us/omniverse/

# Or use Docker (recommended for CI/CD)
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1
docker run --gpus all -it \
  -v ~/isaac_ws:/workspace \
  nvcr.io/nvidia/isaac-sim:2023.1.1
```

### Creating Humanoid Simulation Environments

```python
from omni.isaac.kit import SimulationApp

# Launch with specific configuration
config = {
    "headless": False,  # Set True for training
    "width": 1920,
    "height": 1080,
    "renderer": "RayTracedLighting",
    "anti_aliasing": "FXAA"
}
simulation_app = SimulationApp(config)

# Now import Isaac modules
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.isaac.core.utils.prims as prim_utils

class HumanoidSimEnv:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)

        # Physics settings for stable humanoid simulation
        physics_dt = 1.0 / 500.0  # 500 Hz physics
        rendering_dt = 1.0 / 60.0  # 60 Hz rendering

        self.world.set_simulation_dt(
            physics_dt=physics_dt,
            rendering_dt=rendering_dt
        )

    def setup_scene(self):
        """Create a realistic indoor environment."""
        # Ground plane with realistic material
        self.world.scene.add_default_ground_plane(
            z_position=0,
            name="ground",
            prim_path="/World/ground",
            static_friction=0.8,
            dynamic_friction=0.6,
            restitution=0.1
        )

        # Add warehouse environment from Omniverse assets
        assets_root = get_assets_root_path()
        warehouse_path = assets_root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
        add_reference_to_stage(warehouse_path, "/World/Environment")

        # Realistic lighting
        from omni.isaac.core.utils.prims import create_prim
        create_prim(
            prim_path="/World/DomeLight",
            prim_type="DomeLight",
            attributes={
                "inputs:intensity": 1000,
                "inputs:texture:file": assets_root + "/NVIDIA/Materials/Base/HDR/kloofendal_48d_partly_cloudy.hdr"
            }
        )

    def add_humanoid(self, usd_path: str, position: tuple = (0, 0, 0.95)):
        """Add humanoid robot to scene."""
        self.humanoid = self.world.scene.add(
            Robot(
                prim_path="/World/Humanoid",
                usd_path=usd_path,
                name="humanoid",
                position=position
            )
        )

        # Get articulation controller for joint control
        self.controller = self.humanoid.get_articulation_controller()

        return self.humanoid

    def add_sensors(self):
        """Add perception sensors to humanoid."""
        from omni.isaac.sensor import Camera, IMUSensor

        # Head-mounted RGB-D camera
        self.camera = Camera(
            prim_path="/World/Humanoid/head/camera",
            resolution=(640, 480),
            frequency=30
        )

        # IMU on torso
        self.imu = IMUSensor(
            prim_path="/World/Humanoid/torso/imu",
            name="torso_imu",
            frequency=400,
            translation=(0, 0, 0)
        )

        self.world.scene.add(self.camera)
        self.world.scene.add(self.imu)

    def step(self, joint_targets: dict = None):
        """Advance simulation one step."""
        if joint_targets:
            self.controller.apply_action(joint_targets)

        self.world.step(render=True)

        # Return observations
        return {
            "rgb": self.camera.get_rgb(),
            "depth": self.camera.get_depth(),
            "imu": self.imu.get_sensor_reading(),
            "joint_positions": self.humanoid.get_joint_positions(),
            "joint_velocities": self.humanoid.get_joint_velocities()
        }
```

### Synthetic Data Generation

Isaac Sim's Replicator framework automates dataset creation:

```python
import omni.replicator.core as rep
from omni.isaac.synthetic_utils import SyntheticDataHelper

class SyntheticDataPipeline:
    def __init__(self, output_dir: str):
        self.output_dir = output_dir
        self.sdh = SyntheticDataHelper()

        # Configure what ground truth to capture
        self.sdh.initialize([
            "rgb",
            "depth",
            "semantic_segmentation",
            "instance_segmentation",
            "bounding_box_2d_tight",
            "bounding_box_3d",
            "normals",
            "motion_vectors"
        ])

    def setup_domain_randomization(self):
        """Configure automatic randomization."""
        # Randomize lighting
        with rep.new_layer("lighting_randomization"):
            lights = rep.get.prim_at_path("/World/DomeLight")
            with rep.trigger.on_frame():
                with lights:
                    rep.modify.attribute("inputs:intensity",
                        rep.distribution.uniform(500, 2000))
                    rep.modify.attribute("inputs:color",
                        rep.distribution.uniform((0.8, 0.8, 0.8), (1.2, 1.2, 1.0)))

        # Randomize camera pose
        with rep.new_layer("camera_randomization"):
            camera = rep.get.prim_at_path("/World/Humanoid/head/camera")
            with rep.trigger.on_frame():
                with camera:
                    rep.modify.pose(
                        position=rep.distribution.uniform(
                            (-0.02, -0.02, -0.02), (0.02, 0.02, 0.02)),
                        rotation=rep.distribution.uniform(
                            (-2, -2, -2), (2, 2, 2))
                    )

        # Randomize object positions
        with rep.new_layer("object_randomization"):
            objects = rep.get.prim_at_path("/World/Objects/*")
            with rep.trigger.on_frame():
                with objects:
                    rep.randomizer.scatter_2d(
                        surface_prims=["/World/ground"],
                        check_for_collisions=True
                    )

    def generate_dataset(self, num_frames: int = 10000):
        """Generate training dataset."""
        import json

        annotations = []

        for i in range(num_frames):
            # Step simulation with randomization
            self.world.step()

            # Capture ground truth
            gt = self.sdh.get_groundtruth([
                "rgb", "depth", "semantic_segmentation",
                "bounding_box_2d_tight", "bounding_box_3d"
            ])

            # Save images
            frame_dir = f"{self.output_dir}/frame_{i:06d}"
            os.makedirs(frame_dir, exist_ok=True)

            np.save(f"{frame_dir}/rgb.npy", gt["rgb"])
            np.save(f"{frame_dir}/depth.npy", gt["depth"])
            np.save(f"{frame_dir}/semantic.npy", gt["semantic_segmentation"])

            # Convert annotations to COCO format
            annotations.append({
                "frame_id": i,
                "bboxes_2d": gt["bounding_box_2d_tight"].tolist(),
                "bboxes_3d": gt["bounding_box_3d"].tolist()
            })

            if i % 100 == 0:
                print(f"Generated {i}/{num_frames} frames")

        # Save annotations
        with open(f"{self.output_dir}/annotations.json", "w") as f:
            json.dump(annotations, f)

        print(f"Dataset saved to {self.output_dir}")
```

---

## Isaac ROS: GPU-Accelerated Perception

Isaac ROS provides drop-in replacements for CPU-bound perception algorithms, achieving 5-10x speedups.

### Performance Benchmarks

| Algorithm | CPU Implementation | Isaac ROS (GPU) | Speedup |
|-----------|-------------------|-----------------|---------|
| **Visual SLAM** | 15-30 Hz | 90-120 Hz | 4-6x |
| **Stereo Depth** | 10-15 Hz | 60-90 Hz | 6x |
| **3D Reconstruction** | 1-2 Hz | 30+ Hz | 15-30x |
| **Object Detection** | 5-10 Hz | 60+ Hz | 6-12x |
| **Semantic Segmentation** | 2-5 Hz | 30+ Hz | 6-15x |

### Installing Isaac ROS

```bash
# Prerequisites
# - ROS 2 Humble
# - NVIDIA GPU with CUDA 11.8+
# - Docker (recommended)

# Clone Isaac ROS
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection.git

# Build in Docker container (recommended)
cd ~/isaac_ros_ws/src/isaac_ros_common
./scripts/run_dev.sh

# Inside container
cd /workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash
```

### cuVSLAM: Real-Time Visual SLAM

cuVSLAM runs visual odometry entirely on GPU:

```yaml
# config/visual_slam_params.yaml
visual_slam:
  ros__parameters:
    # Input configuration
    image_topic: /camera/color/image_raw
    camera_info_topic: /camera/color/camera_info

    # IMU integration (optional but recommended)
    enable_imu_fusion: true
    imu_topic: /imu/data

    # Performance tuning
    enable_localization: true
    enable_mapping: true
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"

    # Quality settings
    num_features: 500
    enable_debug_mode: false

    # For humanoid: account for dynamic motion
    max_velocity: 2.0  # m/s
    max_angular_velocity: 3.14  # rad/s
```

```python
# Launch cuVSLAM with custom configuration
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_ros_visual_slam',
            executable='visual_slam_node',
            name='visual_slam',
            parameters=['config/visual_slam_params.yaml'],
            remappings=[
                ('visual_slam/image', '/humanoid/head_camera/color/image_raw'),
                ('visual_slam/camera_info', '/humanoid/head_camera/color/camera_info'),
                ('visual_slam/imu', '/humanoid/imu/data')
            ],
            output='screen'
        )
    ])
```

### nvblox: Real-Time 3D Mapping

nvblox creates dense 3D reconstructions and navigation maps in real-time:

```yaml
# config/nvblox_params.yaml
nvblox_node:
  ros__parameters:
    # Voxel configuration
    voxel_size: 0.05  # 5cm voxels
    tsdf_truncation_distance: 0.15  # 3x voxel size

    # Input topics
    depth_image_topic: /humanoid/head_camera/depth/image_rect_raw
    depth_camera_info_topic: /humanoid/head_camera/depth/camera_info
    color_image_topic: /humanoid/head_camera/color/image_raw
    color_camera_info_topic: /humanoid/head_camera/color/camera_info

    # Transform frames
    global_frame: "odom"
    pose_frame: "base_link"

    # Output configuration
    publish_mesh: true
    publish_esdf: true
    publish_occupancy_grid: true

    # For humanoid navigation
    slice_height: 0.5  # Height for 2D costmap slice
    min_height: 0.1
    max_height: 2.0

    # Performance
    esdf_update_rate: 10.0  # Hz
    mesh_update_rate: 5.0   # Hz
```

**Integration with Nav2:**

```python
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid

class NvbloxNav2Bridge(Node):
    def __init__(self):
        super().__init__('nvblox_nav2_bridge')

        # Subscribe to nvblox outputs
        self.esdf_sub = self.create_subscription(
            OccupancyGrid,
            '/nvblox_node/occupancy_grid',
            self.esdf_callback,
            10
        )

        # Publish to Nav2 costmap
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/global_costmap/costmap',
            10
        )

    def esdf_callback(self, msg):
        # Convert nvblox occupancy to Nav2 costmap
        # Add inflation for humanoid footprint
        costmap = self.inflate_obstacles(msg, inflation_radius=0.3)
        self.costmap_pub.publish(costmap)
```

---

## Isaac Gym: Massively Parallel Robot Learning

Isaac Gym enables training across thousands of parallel environments on a single GPU.

### Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                      Isaac Gym Architecture                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   ┌─────────────────────────────────────────────────────────┐   │
│   │                    Python Interface                      │   │
│   │    env.reset() → observations (Tensor)                  │   │
│   │    env.step(actions) → obs, rewards, dones, info       │   │
│   └─────────────────────────────────────────────────────────┘   │
│                              ↕                                   │
│   ┌─────────────────────────────────────────────────────────┐   │
│   │                   PhysX GPU Simulation                   │   │
│   │                                                          │   │
│   │  ┌────┐ ┌────┐ ┌────┐ ┌────┐ ┌────┐ ┌────┐           │   │
│   │  │Env0│ │Env1│ │Env2│ │... │ │4094│ │4095│           │   │
│   │  └────┘ └────┘ └────┘ └────┘ └────┘ └────┘           │   │
│   │                                                          │   │
│   │  All environments stepped simultaneously on GPU          │   │
│   └─────────────────────────────────────────────────────────┘   │
│                              ↕                                   │
│   ┌─────────────────────────────────────────────────────────┐   │
│   │                    GPU Memory                            │   │
│   │                                                          │   │
│   │  States:     [4096 × state_dim]   (e.g., 4096 × 60)     │   │
│   │  Actions:    [4096 × action_dim]  (e.g., 4096 × 21)     │   │
│   │  Rewards:    [4096]                                      │   │
│   │  Dones:      [4096]                                      │   │
│   │                                                          │   │
│   │  Zero CPU-GPU transfer during training!                  │   │
│   └─────────────────────────────────────────────────────────┘   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Complete Humanoid Locomotion Environment

```python
from isaacgym import gymapi, gymtorch, gymutil
import torch
import numpy as np

class HumanoidLocomotionEnv:
    """
    Massively parallel humanoid walking environment.
    Trains 4096 humanoids simultaneously.
    """

    def __init__(self, num_envs: int = 4096, device: str = "cuda:0"):
        self.num_envs = num_envs
        self.device = device

        # Acquire gym interface
        self.gym = gymapi.acquire_gym()

        # Simulation parameters
        sim_params = gymapi.SimParams()
        sim_params.dt = 1.0 / 500.0  # 500 Hz physics
        sim_params.substeps = 2
        sim_params.up_axis = gymapi.UP_AXIS_Z
        sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)

        # PhysX parameters for stable humanoid sim
        sim_params.physx.solver_type = 1  # TGS (recommended for articulations)
        sim_params.physx.num_position_iterations = 4
        sim_params.physx.num_velocity_iterations = 1
        sim_params.physx.contact_offset = 0.02
        sim_params.physx.rest_offset = 0.0
        sim_params.physx.use_gpu = True
        sim_params.physx.num_threads = 4

        # Create simulation
        self.sim = self.gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

        # Environment dimensions
        self.state_dim = 60  # Joint pos (21) + vel (21) + base ori (4) + base vel (6) + ...
        self.action_dim = 21  # Joint position targets

        self._create_ground()
        self._create_envs()
        self._init_tensors()

    def _create_ground(self):
        """Create ground plane with friction."""
        plane_params = gymapi.PlaneParams()
        plane_params.normal = gymapi.Vec3(0, 0, 1)
        plane_params.static_friction = 0.8
        plane_params.dynamic_friction = 0.6
        plane_params.restitution = 0.0
        self.gym.add_ground(self.sim, plane_params)

    def _create_envs(self):
        """Create parallel environments with humanoid robots."""
        # Load humanoid asset
        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = False
        asset_options.angular_damping = 0.01
        asset_options.max_angular_velocity = 100.0
        asset_options.default_dof_drive_mode = gymapi.DOF_MODE_POS

        humanoid_asset = self.gym.load_asset(
            self.sim, "assets", "humanoid.urdf", asset_options
        )

        self.num_dof = self.gym.get_asset_dof_count(humanoid_asset)
        self.num_bodies = self.gym.get_asset_rigid_body_count(humanoid_asset)

        # Environment spacing
        spacing = 2.0
        envs_per_row = int(np.sqrt(self.num_envs))

        lower = gymapi.Vec3(-spacing, -spacing, 0.0)
        upper = gymapi.Vec3(spacing, spacing, spacing)

        self.envs = []
        self.actors = []

        for i in range(self.num_envs):
            env = self.gym.create_env(self.sim, lower, upper, envs_per_row)

            # Initial pose: standing
            start_pose = gymapi.Transform()
            start_pose.p = gymapi.Vec3(0.0, 0.0, 0.95)  # Standing height
            start_pose.r = gymapi.Quat(0, 0, 0, 1)

            actor = self.gym.create_actor(
                env, humanoid_asset, start_pose, "humanoid", i, 1
            )

            # Configure joint properties
            dof_props = self.gym.get_actor_dof_properties(env, actor)
            dof_props['driveMode'].fill(gymapi.DOF_MODE_POS)
            dof_props['stiffness'].fill(100.0)  # Position gain
            dof_props['damping'].fill(10.0)     # Velocity gain
            self.gym.set_actor_dof_properties(env, actor, dof_props)

            self.envs.append(env)
            self.actors.append(actor)

    def _init_tensors(self):
        """Initialize GPU tensors for fast access."""
        self.gym.prepare_sim(self.sim)

        # Get GPU tensor handles
        actor_root_state = self.gym.acquire_actor_root_state_tensor(self.sim)
        dof_state = self.gym.acquire_dof_state_tensor(self.sim)
        rigid_body_state = self.gym.acquire_rigid_body_state_tensor(self.sim)
        contact_force = self.gym.acquire_net_contact_force_tensor(self.sim)

        # Wrap as PyTorch tensors (zero-copy!)
        self.root_states = gymtorch.wrap_tensor(actor_root_state)
        self.dof_states = gymtorch.wrap_tensor(dof_state)
        self.rigid_body_states = gymtorch.wrap_tensor(rigid_body_state)
        self.contact_forces = gymtorch.wrap_tensor(contact_force)

        # Separate position and velocity views
        self.dof_pos = self.dof_states[:, 0].view(self.num_envs, self.num_dof)
        self.dof_vel = self.dof_states[:, 1].view(self.num_envs, self.num_dof)

        # Default joint positions (standing pose)
        self.default_dof_pos = torch.zeros(
            self.num_dof, dtype=torch.float, device=self.device
        )

        # Command velocity (target walking direction)
        self.commands = torch.zeros(
            (self.num_envs, 3), dtype=torch.float, device=self.device
        )  # [vx, vy, yaw_rate]

    def reset(self, env_ids: torch.Tensor = None):
        """Reset specified environments."""
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)

        num_resets = len(env_ids)

        # Reset root state (position, orientation, velocity)
        self.root_states[env_ids, :3] = 0  # Position
        self.root_states[env_ids, 2] = 0.95  # Height
        self.root_states[env_ids, 3:7] = torch.tensor([0, 0, 0, 1])  # Quaternion
        self.root_states[env_ids, 7:13] = 0  # Velocities

        # Reset joint states
        self.dof_pos[env_ids] = self.default_dof_pos
        self.dof_vel[env_ids] = 0

        # Apply resets
        self.gym.set_actor_root_state_tensor(
            self.sim, gymtorch.unwrap_tensor(self.root_states)
        )
        self.gym.set_dof_state_tensor(
            self.sim, gymtorch.unwrap_tensor(self.dof_states)
        )

        # Randomize commands for new episodes
        self.commands[env_ids, 0] = torch.rand(num_resets, device=self.device) * 1.0  # vx: 0-1 m/s
        self.commands[env_ids, 1] = 0  # vy: 0 (forward walking)
        self.commands[env_ids, 2] = (torch.rand(num_resets, device=self.device) - 0.5) * 0.5  # yaw rate

        return self._compute_observations()

    def step(self, actions: torch.Tensor):
        """Step all environments with given actions."""
        # Scale actions to joint position targets
        targets = self.default_dof_pos + actions * 0.5  # Scale factor

        # Apply joint targets
        self.gym.set_dof_position_target_tensor(
            self.sim, gymtorch.unwrap_tensor(targets.flatten())
        )

        # Step physics
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)

        # Refresh tensors from GPU
        self.gym.refresh_actor_root_state_tensor(self.sim)
        self.gym.refresh_dof_state_tensor(self.sim)
        self.gym.refresh_rigid_body_state_tensor(self.sim)
        self.gym.refresh_net_contact_force_tensor(self.sim)

        # Compute outputs
        observations = self._compute_observations()
        rewards = self._compute_rewards()
        dones = self._compute_dones()

        # Reset fallen robots
        reset_ids = dones.nonzero(as_tuple=False).flatten()
        if len(reset_ids) > 0:
            self.reset(reset_ids)

        return observations, rewards, dones, {}

    def _compute_observations(self) -> torch.Tensor:
        """Compute observation tensor for all envs."""
        # Base orientation (quaternion)
        base_quat = self.root_states[:, 3:7]

        # Base angular velocity (in base frame)
        base_ang_vel = self.root_states[:, 10:13]

        # Projected gravity (tells robot which way is up)
        gravity_vec = torch.tensor([0, 0, -1], device=self.device).repeat(self.num_envs, 1)
        projected_gravity = quat_rotate_inverse(base_quat, gravity_vec)

        # Command velocities
        commands = self.commands

        # Joint states
        dof_pos_scaled = (self.dof_pos - self.default_dof_pos) * 2.0
        dof_vel_scaled = self.dof_vel * 0.1

        obs = torch.cat([
            projected_gravity,     # 3
            commands,              # 3
            dof_pos_scaled,        # num_dof
            dof_vel_scaled,        # num_dof
            base_ang_vel * 0.25    # 3
        ], dim=-1)

        return obs

    def _compute_rewards(self) -> torch.Tensor:
        """Compute reward for all envs."""
        # Velocity tracking reward
        base_vel = self.root_states[:, 7:10]
        vel_error = torch.sum(torch.square(base_vel[:, :2] - self.commands[:, :2]), dim=1)
        velocity_reward = torch.exp(-vel_error / 0.25)

        # Orientation reward (stay upright)
        base_quat = self.root_states[:, 3:7]
        gravity_vec = torch.tensor([0, 0, -1], device=self.device).repeat(self.num_envs, 1)
        projected_gravity = quat_rotate_inverse(base_quat, gravity_vec)
        orientation_reward = torch.sum(torch.square(projected_gravity[:, :2]), dim=1)
        orientation_reward = torch.exp(-orientation_reward / 0.25)

        # Action smoothness (penalize jerky motions)
        # (would need to store previous actions)

        # Foot contact reward (encourage proper gait)
        # left_contact = self.contact_forces[:, left_foot_idx, 2] > 1.0
        # right_contact = self.contact_forces[:, right_foot_idx, 2] > 1.0

        # Combined reward
        reward = (
            velocity_reward * 1.0 +
            orientation_reward * 0.5
        )

        return reward

    def _compute_dones(self) -> torch.Tensor:
        """Determine which envs should reset."""
        # Termination conditions
        base_height = self.root_states[:, 2]
        fallen = base_height < 0.5  # Below standing height

        # Check for extreme orientations
        base_quat = self.root_states[:, 3:7]
        gravity_vec = torch.tensor([0, 0, -1], device=self.device).repeat(self.num_envs, 1)
        projected_gravity = quat_rotate_inverse(base_quat, gravity_vec)
        too_tilted = torch.abs(projected_gravity[:, 2]) < 0.5  # More than ~60° tilt

        return fallen | too_tilted


def quat_rotate_inverse(q, v):
    """Rotate vector by inverse of quaternion."""
    q_w = q[:, 3:4]
    q_vec = q[:, :3]
    a = v * (2.0 * q_w ** 2 - 1.0)
    b = torch.cross(q_vec, v, dim=-1) * q_w * 2.0
    c = q_vec * torch.sum(q_vec * v, dim=-1, keepdim=True) * 2.0
    return a - b + c
```

### Training with rl_games

```python
from rl_games.torch_runner import Runner
from rl_games.common import env_configurations

# Register custom environment
env_configurations.register(
    'humanoid_locomotion',
    {
        'env_creator': lambda **kwargs: HumanoidLocomotionEnv(**kwargs),
        'vecenv_type': 'ISAACGYM'
    }
)

# Training configuration
train_config = {
    'params': {
        'algo': {
            'name': 'a2c_continuous'
        },
        'model': {
            'name': 'continuous_a2c_logstd'
        },
        'network': {
            'name': 'actor_critic',
            'separate': False,
            'space': {'continuous': {'mu_activation': 'None'}},
            'mlp': {
                'units': [512, 256, 128],
                'activation': 'elu'
            }
        },
        'config': {
            'name': 'humanoid_walk',
            'env_name': 'humanoid_locomotion',
            'num_actors': 4096,
            'horizon_length': 24,
            'minibatch_size': 32768,
            'mini_epochs': 5,
            'critic_coef': 2,
            'lr_schedule': 'adaptive',
            'learning_rate': 3e-4,
            'kl_threshold': 0.016,
            'max_epochs': 5000,
            'save_frequency': 100
        }
    }
}

# Run training
runner = Runner()
runner.load(train_config)
runner.run({'train': True})
```

---

## Bipedal Navigation and Planning

Humanoid navigation requires more than wheeled robot approaches—you must plan where each foot goes.

### Footstep Planning Fundamentals

```python
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
from enum import Enum

class Foot(Enum):
    LEFT = 0
    RIGHT = 1

@dataclass
class Footstep:
    foot: Foot
    position: np.ndarray  # [x, y, z]
    orientation: float    # yaw angle
    timestamp: float

class FootstepPlanner:
    """A* based footstep planner for humanoid navigation."""

    def __init__(self,
                 step_length: float = 0.25,
                 step_width: float = 0.18,
                 max_step_angle: float = 0.3):
        self.step_length = step_length
        self.step_width = step_width
        self.max_step_angle = max_step_angle

        # Reachability constraints
        self.min_step_length = -0.1
        self.max_step_length = 0.35
        self.min_step_width = 0.12
        self.max_step_width = 0.30

    def plan(self, start: np.ndarray, goal: np.ndarray,
             obstacles: List[np.ndarray] = None) -> List[Footstep]:
        """
        Plan footstep sequence from start to goal.

        Args:
            start: [x, y, yaw] starting pose
            goal: [x, y, yaw] goal pose
            obstacles: List of obstacle polygons

        Returns:
            List of Footstep objects
        """
        footsteps = []
        current_pose = start.copy()
        current_foot = Foot.LEFT

        # Initial stance
        left_pos = current_pose[:2] + self._foot_offset(current_pose[2], Foot.LEFT)
        right_pos = current_pose[:2] + self._foot_offset(current_pose[2], Foot.RIGHT)

        while np.linalg.norm(current_pose[:2] - goal[:2]) > self.step_length:
            # Compute next step
            next_foot = Foot.RIGHT if current_foot == Foot.LEFT else Foot.LEFT

            # Direction to goal
            direction = goal[:2] - current_pose[:2]
            direction = direction / np.linalg.norm(direction)

            # Target position
            step_vec = direction * self.step_length
            foot_offset = self._foot_offset(current_pose[2], next_foot)
            target_pos = current_pose[:2] + step_vec + foot_offset

            # Target orientation (face goal)
            target_yaw = np.arctan2(direction[1], direction[0])

            # Clamp step to reachability
            target_pos, target_yaw = self._clamp_to_reachable(
                current_pose, target_pos, target_yaw, current_foot
            )

            # Check collision (simplified)
            if obstacles and self._check_collision(target_pos, obstacles):
                # TODO: Implement stepping over/around
                continue

            footstep = Footstep(
                foot=next_foot,
                position=np.array([target_pos[0], target_pos[1], 0.0]),
                orientation=target_yaw,
                timestamp=len(footsteps) * 0.5  # 0.5s per step
            )
            footsteps.append(footstep)

            # Update state
            current_foot = next_foot
            current_pose[:2] = target_pos - foot_offset
            current_pose[2] = target_yaw

        return footsteps

    def _foot_offset(self, yaw: float, foot: Foot) -> np.ndarray:
        """Compute lateral offset for foot placement."""
        sign = 1 if foot == Foot.LEFT else -1
        offset = np.array([0, sign * self.step_width / 2])
        rotation = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw), np.cos(yaw)]
        ])
        return rotation @ offset

    def _clamp_to_reachable(self, current, target_pos, target_yaw, stance_foot):
        """Ensure step is within kinematic limits."""
        # Convert to stance foot frame
        stance_pos = current[:2] + self._foot_offset(current[2], stance_foot)

        # Compute relative position
        dx = target_pos[0] - stance_pos[0]
        dy = target_pos[1] - stance_pos[1]
        dyaw = target_yaw - current[2]

        # Clamp
        dx = np.clip(dx, self.min_step_length, self.max_step_length)
        dy = np.clip(np.abs(dy), self.min_step_width, self.max_step_width) * np.sign(dy)
        dyaw = np.clip(dyaw, -self.max_step_angle, self.max_step_angle)

        return stance_pos + np.array([dx, dy]), current[2] + dyaw
```

### Dynamic Stability: Zero Moment Point

```python
class ZMPController:
    """Zero Moment Point based balance controller."""

    def __init__(self, robot_mass: float, com_height: float):
        self.mass = robot_mass
        self.com_height = com_height
        self.gravity = 9.81

    def compute_zmp(self, com_pos: np.ndarray, com_acc: np.ndarray) -> np.ndarray:
        """
        Compute Zero Moment Point from CoM state.

        ZMP = CoM_xy - (h/g) * CoM_acc_xy
        """
        zmp = com_pos[:2] - (self.com_height / self.gravity) * com_acc[:2]
        return zmp

    def is_stable(self, zmp: np.ndarray, support_polygon: np.ndarray) -> bool:
        """Check if ZMP is within support polygon."""
        from shapely.geometry import Point, Polygon
        point = Point(zmp[0], zmp[1])
        polygon = Polygon(support_polygon)
        return polygon.contains(point)

    def compute_capture_point(self, com_pos: np.ndarray,
                              com_vel: np.ndarray) -> np.ndarray:
        """
        Compute capture point for push recovery.

        Capture Point = CoM_xy + CoM_vel_xy / omega
        where omega = sqrt(g/h)
        """
        omega = np.sqrt(self.gravity / self.com_height)
        capture_point = com_pos[:2] + com_vel[:2] / omega
        return capture_point
```

---

## Capstone Project: Autonomous Humanoid Navigator

Build a complete navigation system using Isaac tools:

### System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                  Autonomous Navigation Stack                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐         │
│  │   Sensors   │    │  Perception │    │   Planning  │         │
│  │             │    │             │    │             │         │
│  │ RGB-D Camera│───▶│  cuVSLAM   │───▶│  Footstep   │         │
│  │     IMU     │    │  nvblox    │    │   Planner   │         │
│  │ Joint State │    │  Detection │    │             │         │
│  └─────────────┘    └─────────────┘    └──────┬──────┘         │
│                                               │                  │
│                                        ┌──────▼──────┐          │
│                                        │   Control   │          │
│                                        │             │          │
│  ┌─────────────┐    ┌─────────────┐   │ RL Walking  │          │
│  │   Motors    │◀───│    MPC     │◀──│   Policy    │          │
│  │             │    │  Balance   │    │             │          │
│  └─────────────┘    └─────────────┘    └─────────────┘          │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Implementation Checklist

- [ ] Set up Isaac Sim environment with indoor scene
- [ ] Configure humanoid with RGB-D camera and IMU
- [ ] Launch cuVSLAM for real-time localization
- [ ] Configure nvblox for 3D reconstruction
- [ ] Implement footstep planner with A* search
- [ ] Train locomotion policy in Isaac Gym (1M+ steps)
- [ ] Implement ZMP-based balance controller
- [ ] Create ROS 2 integration nodes
- [ ] Test end-to-end navigation in simulation
- [ ] Validate with domain randomization

---

## Key Takeaways

- **GPU acceleration** transforms robotics from constrained to capable—operations that took hours now take minutes

- **Isaac Sim** provides photorealistic simulation that enables training vision models that transfer to real cameras

- **Synthetic data generation** with automatic labeling solves the data bottleneck for robot perception

- **Isaac ROS** offers 5-10x speedups for perception tasks, enabling real-time SLAM and mapping

- **Isaac Gym** enables training across thousands of parallel environments, compressing training time from weeks to hours

- **Bipedal planning** requires explicit footstep sequences and balance constraints (ZMP, capture point)

- **Integration** across Isaac tools creates complete autonomy stacks from perception to control

---

## Further Reading

- [NVIDIA Isaac Documentation](https://developer.nvidia.com/isaac)
- [Isaac Gym Paper](https://arxiv.org/abs/2108.10470)
- [Legged Robots That Balance (Raibert)](https://mitpress.mit.edu/9780262681193/legged-robots-that-balance/)
- [Bipedal Walking Control Survey](https://ieeexplore.ieee.org/document/8968453)

---

**Previous:** [← Module 2 — Digital Twin](../module-2-digital-twin/index.md)

**Next:** [Module 4 — Vision-Language-Action →](../module-4-vla/index.md)
