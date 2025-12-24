---
sidebar_position: 6
---

# Module 5: Sensor Fusion & State Estimation

> **Author:** Umema Sultan

---

## Learning Objectives

By the end of this module, you will be able to:

- Explain why sensor fusion is necessary for accurate state estimation
- Describe the predict-update cycle of Kalman filtering
- Compare IMU, vision, and proprioceptive sensors in terms of strengths and limitations
- Explain how Visual-Inertial Odometry combines cameras and IMUs
- Identify observability limitations and strategies for handling unobservable states

---

Humanoid robots must continuously answer a fundamental question: *Where am I, and what is happening to my body?* The answer comes not from any single sensor, but from the intelligent combination of many imperfect measurements. Sensor fusion is the discipline of merging diverse data streams into a unified, accurate estimate of the robot's state—the foundation upon which all planning and control depends.

---

## The Perception Challenge

A humanoid robot operates in a world of uncertainty. Its sensors are imperfect instruments, each offering a partial and noisy view of reality.

Consider the challenge of simply knowing where the robot's torso is located in space. No single sensor provides this information directly:

- An **accelerometer** measures forces but cannot distinguish gravity from motion
- A **gyroscope** tracks rotation but drifts over time
- A **camera** sees the environment but struggles with rapid motion and poor lighting
- **Joint encoders** know limb positions but not where the base is anchored
- **Foot pressure sensors** detect contact but say nothing about global position

Each sensor tells part of the story. Sensor fusion assembles these fragments into a coherent narrative.

---

## What is Robot State?

The **state** of a robot is the minimum set of variables needed to fully describe its current situation. For a humanoid, this typically includes:

**Kinematic State**
- Position of the base (torso) in the world frame
- Orientation of the base (which way it faces, how it tilts)
- Angles of every joint in the kinematic chain

**Dynamic State**
- Linear velocity of the base
- Angular velocity of the base
- Joint velocities

**Contact State**
- Which feet (or hands) are in contact with the environment
- Contact forces and moments

**Internal State**
- Sensor biases that drift over time
- Calibration parameters that may change

Accurate knowledge of this state enables the robot to plan movements, maintain balance, and interact safely with its environment.

---

## Sensor Characteristics

Understanding each sensor's strengths and limitations is essential for effective fusion.

### Inertial Measurement Units

The IMU is the workhorse of robot state estimation. It contains accelerometers measuring linear acceleration and gyroscopes measuring angular velocity. Modern IMUs sample at hundreds or thousands of hertz, providing rapid updates about the robot's motion.

However, IMUs suffer from **drift**. Small errors in the gyroscope accumulate over time, causing the estimated orientation to gradually diverge from reality. Accelerometer bias causes similar drift in velocity and position estimates. Left uncorrected, an IMU-only estimate becomes useless within seconds.

### Vision Systems

Cameras provide rich information about the environment. By tracking visual features across frames, vision systems can estimate motion and recognize locations. Unlike IMUs, vision does not drift over time—each frame provides an independent measurement.

The weakness of vision is its **latency and fragility**. Processing images takes time, limiting update rates to tens of frames per second. Rapid motion causes blur. Poor lighting, textureless surfaces, and visual occlusion all degrade performance.

### Proprioceptive Sensors

Encoders at each joint measure angles with high precision and minimal drift. Combined with the robot's kinematic model, joint measurements enable computation of limb positions relative to the base.

Proprioception alone cannot determine where the robot is in the world. It provides **relative** information within the body, not **absolute** positioning in space.

### Contact and Force Sensors

Force-torque sensors in the feet measure ground reaction forces. These measurements reveal not only whether contact exists but also how the robot's weight is distributed.

Contact information is crucial for state estimation because a foot in solid contact provides a **fixed reference point**. If the foot is not moving relative to the ground, observed motion at that foot (from kinematics) directly reveals base motion.

---

## The Fusion Framework

Sensor fusion algorithms share a common structure: **prediction** followed by **correction**.

### Prediction Step

Between sensor measurements, the algorithm predicts how the state evolves. This prediction uses a **motion model**—a mathematical description of the robot's dynamics.

For example, if the robot is moving forward at a known velocity, the predicted position one timestep later equals the current position plus velocity times the time interval. The prediction carries forward not just the state estimate but also its **uncertainty**, which grows over time as errors accumulate.

### Correction Step

When a sensor measurement arrives, the algorithm compares the predicted measurement (what the sensor should see given the predicted state) with the actual measurement. The difference—called the **innovation** or **residual**—drives a correction.

The magnitude of the correction depends on relative confidence. If the prediction is uncertain but the sensor is reliable, the correction is large. If the prediction is confident and the sensor is noisy, the correction is small. This balancing is formalized through **covariance matrices** that quantify uncertainty.

### The Kalman Filter Family

The **Kalman Filter** is the optimal solution for linear systems with Gaussian noise. Real robots are nonlinear, so practitioners use extensions:

The **Extended Kalman Filter (EKF)** linearizes the system around the current estimate, applying standard Kalman equations to the linearized model. It is computationally efficient and widely used.

The **Unscented Kalman Filter (UKF)** avoids linearization by propagating carefully chosen sample points through the nonlinear model. It handles strong nonlinearities better than the EKF at modest additional cost.

**Particle Filters** represent uncertainty with many weighted samples, enabling representation of complex, multi-modal distributions. They are powerful but computationally expensive.

For humanoid robots, the EKF remains the most common choice, offering a practical balance of accuracy and computational efficiency.

---

## Visual-Inertial Odometry

The combination of cameras and IMUs—**Visual-Inertial Odometry (VIO)**—has become the gold standard for robot pose estimation.

### Complementary Strengths

IMUs and cameras compensate for each other's weaknesses:

| IMU Provides | Camera Provides |
|--------------|-----------------|
| High-rate motion sensing | Drift-free position reference |
| Works in darkness | Works when stationary |
| Immune to visual occlusion | Corrects accumulated IMU drift |
| Instant response | Absolute scale (with stereo or depth) |

### Tight vs. Loose Coupling

In **loosely coupled** systems, the camera and IMU run separate estimators whose outputs are merged. This is simpler to implement but discards information.

In **tightly coupled** systems, raw sensor data feeds directly into a unified estimator. This preserves all information and achieves better accuracy, but requires more careful engineering.

Modern VIO systems are predominantly tightly coupled, with the IMU providing predictions between camera frames and the camera providing corrections that bound drift.

---

## Legged Robot State Estimation

Humanoid robots present unique estimation challenges beyond those faced by wheeled or flying robots.

### The Floating Base Problem

Unlike a wheeled robot whose base is constrained to the ground plane, a humanoid's torso **floats** in space, connected to the ground only through its legs. The base has six degrees of freedom (three position, three orientation) that must be estimated continuously.

### Contact as a Reference

When a foot is planted on the ground, it provides a fixed reference. The kinematic chain from foot to torso becomes a measurement: if the foot isn't moving and we know the joint angles, we can compute where the torso must be.

The challenge is that contact is intermittent. During walking, each foot alternates between stance (fixed) and swing (moving). The estimator must track which contacts are active and weight their contributions appropriately.

### Leg Odometry

By integrating foot positions during stance phases, the robot can estimate its motion—a technique called **leg odometry**. This is analogous to wheel odometry but accounts for the discrete, alternating nature of footsteps.

Leg odometry drifts over time due to foot slip and kinematic calibration errors, but it provides valuable information between camera updates and during visual degradation.

---

## Multi-Sensor Architecture

A complete humanoid state estimator typically fuses many sensor streams:

**High-Rate Core (500-1000 Hz)**
- IMU integration for orientation and velocity
- Joint encoder updates for kinematic state
- Contact detection from force sensors

**Medium-Rate Corrections (30-100 Hz)**
- Visual odometry updates for position and orientation
- Leg odometry updates during stance phases

**Low-Rate Anchoring (1-10 Hz)**
- Loop closure detection (recognizing previously visited locations)
- External positioning systems if available

The estimator runs at the highest sensor rate, performing prediction at each IMU sample and incorporating other measurements as they arrive.

---

## Observability Considerations

Not all states can be estimated from any given set of sensors. **Observability** analysis determines what can and cannot be inferred.

### Fundamental Limitations

Certain states are inherently unobservable without specific sensors:

- **Global position** requires either a map, GPS, or external reference
- **Absolute yaw** cannot be determined from IMU and camera alone (only relative changes)
- **Scale** is ambiguous with a single camera unless depth is measured

### Handling Unobservable States

When states are unobservable, the estimator's uncertainty for those states grows without bound. Practical systems must either:

- Accept drift in unobservable directions
- Incorporate additional sensors that provide observability
- Use external references (maps, landmarks, infrastructure)

For many humanoid applications, relative odometry suffices—the robot tracks motion from a known starting point without needing absolute global position.

---

## Practical Implementation

### Time Synchronization

Sensors sample at different times. An IMU measurement timestamped at *t* = 100ms and a camera frame timestamped at *t* = 105ms must be handled correctly. Without precise synchronization, fusion algorithms introduce errors.

Hardware triggering (sensors share a clock signal) provides the best synchronization. Software approaches estimate and compensate for timing offsets but are less reliable.

### Calibration Requirements

Sensor fusion assumes accurate knowledge of:

- **Intrinsic parameters**: IMU biases, camera focal lengths, encoder scales
- **Extrinsic parameters**: relative positions and orientations between sensors
- **Temporal offsets**: timing differences between sensor clocks

Calibration errors propagate through the estimator, causing systematic biases. Regular calibration—especially of IMU biases, which drift with temperature—is essential.

### Failure Detection

A robust estimator must recognize when something is wrong:

- **Innovation monitoring**: large residuals suggest sensor failure or environmental change
- **Consistency checking**: estimated uncertainty should match observed errors
- **Sanity bounds**: the robot cannot be underground or moving faster than physically possible

When failures are detected, the system should increase uncertainty, reject suspect measurements, or alert higher-level systems.

---

## ROS 2 Integration

State estimation integrates into the ROS 2 ecosystem through standard interfaces.

### The TF Tree

ROS 2 maintains a **transform tree** representing spatial relationships between coordinate frames. The state estimator publishes transforms connecting:

- The world frame (fixed reference)
- The robot's base frame (moving with the robot)
- Sensor frames (camera, IMU, attached to the body)
- Foot frames (at the end effectors)

Other nodes query this tree to transform data between frames, enabling modular system design.

### Standard Topics

Estimators typically publish:

- **Odometry**: pose and velocity estimate with covariance
- **IMU biases**: current estimates of sensor drift
- **Contact states**: which feet are grounded

They subscribe to:

- **Raw IMU data**: accelerations and angular velocities
- **Joint states**: encoder readings
- **Visual odometry**: camera-based pose updates
- **Contact sensors**: force and pressure readings

### Available Packages

The ROS ecosystem provides mature state estimation tools:

- **robot_localization**: general-purpose EKF/UKF for fusing odometry and IMU
- **ROVIO, VINS, ORB-SLAM**: visual-inertial odometry implementations
- **contact_estimation**: specialized packages for legged robots

These provide starting points, though humanoid applications often require custom extensions.

---

## Summary

Sensor fusion transforms the cacophony of imperfect measurements into the clear signal of state knowledge. The key principles:

**Combine complementary sensors** to overcome individual limitations. IMUs provide speed; cameras provide stability. Proprioception provides body knowledge; contact provides ground truth.

**Predict and correct** in a continuous cycle. Motion models propagate estimates forward; measurements pull them back toward reality.

**Quantify uncertainty** at every step. Knowing what you don't know is as important as knowing what you do.

**Design for failure** because sensors will fail, lighting will change, and contacts will slip. Robust estimation degrades gracefully rather than catastrophically.

With accurate state estimation, the humanoid robot knows itself—the essential foundation for intelligent action in the physical world.

---

## Key Takeaways

- **No single sensor suffices**—accurate state estimation requires fusing complementary sources
- **Predict-update cycles** form the foundation of all filtering approaches
- **IMUs provide speed; cameras provide stability**—their combination in VIO is the gold standard
- **Contact information** is critical for legged robots, providing fixed reference points during stance
- **Calibration and synchronization** are prerequisites for effective fusion—garbage in, garbage out
- **Failure detection** is as important as estimation—knowing when estimates are unreliable prevents catastrophic errors

---

**Previous:** [← Module 4 — Vision-Language-Action](../module-4-vla/index.md)

**Next:** [Module 6 — Reinforcement Learning for Locomotion →](../module-5-rl-locomotion/index.md)
