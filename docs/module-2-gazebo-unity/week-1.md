---
id: module-2-gazebo-unity-week-1
title: "Week 1: Gazebo Simulation Basics"
sidebar_position: 2
---

# Week 1: Gazebo Simulation Basics

## Introduction

Week 1 introduces Gazebo, a physics-based simulation environment for robotics. You'll learn to create robot models using URDF/SDF, build simulation worlds, and integrate Gazebo with ROS 2. This week emphasizes understanding physics simulation fundamentals and evaluating simulation realism.

**Learning Approach**: Co-teaching with AI suggestions. AI can suggest simulation patterns, but you must critically evaluate: "How does this simulation differ from real-world physics?" This evaluation skill is essential.

## Topics Covered

### 1. Physics Simulation Fundamentals

**What is Physics Simulation?**:
- Modeling physical laws (gravity, friction, collisions) in software
- Discrete time steps vs continuous physics
- Numerical integration methods
- Trade-offs between accuracy and performance

**Physics Engines**:
- ODE (Open Dynamics Engine) - Gazebo Classic default
- Bullet Physics - Alternative engine
- DART - More accurate but slower

**Reasoning Question**: "What physics phenomena are hard to simulate accurately? How do approximations affect robot behavior?"

### 2. URDF Robot Models

**URDF Structure**:
- Links: Rigid bodies (robot parts)
- Joints: Connections between links (revolute, prismatic, fixed)
- Visual: Appearance (meshes, colors)
- Collision: Physics collision geometry
- Inertial: Mass and inertia properties

**Creating URDF Files**:
- XML syntax
- Link definitions with visual/collision/inertial
- Joint definitions with parent/child links
- Robot description best practices

**Manual Practice**: Create a simple robot URDF (e.g., mobile base with two wheels).

**Reasoning Question**: "What's the difference between visual and collision geometry? Why do we need both?"

### 3. SDF World Files

**SDF vs URDF**:
- SDF: More detailed, includes physics properties
- URDF: ROS 2 standard, simpler
- Conversion between formats

**World Files**:
- Ground plane and environment
- Lighting and physics properties
- Model placement
- Plugin configuration

**Manual Practice**: Create a Gazebo world with your robot model.

**Reasoning Question**: "When should I use SDF instead of URDF? What additional capabilities does SDF provide?"

### 4. ROS 2 Integration with Gazebo

**gazebo_ros_pkgs**:
- Bridge between ROS 2 and Gazebo
- Plugin system for ROS 2 communication
- Topic and service integration

**Launch Files**:
- Starting Gazebo with robot models
- Spawning robots in worlds
- Configuring ROS 2-Gazebo bridge

**Manual Practice**: Launch Gazebo with a robot and control it via ROS 2 topics.

**Reasoning Question**: "How does Gazebo communicate with ROS 2? What's the role of plugins?"

### 5. Basic Sensor Simulation

**Sensor Types**:
- Camera: RGB images, depth images
- LiDAR: Laser scan data
- IMU: Inertial measurement data
- Contact sensors: Force/torque feedback

**Sensor Configuration**:
- Noise models (Gaussian, uniform)
- Update rates
- Field of view and resolution

**Manual Practice**: Add sensors to your robot model and visualize data in RViz2.

**Reasoning Question**: "What sensor noise should I model? How does noise affect robot behavior?"

## Hands-On Exercises

### Exercise 1: Create a Simple Robot URDF

**Objective**: Define a basic robot model in URDF format.

**Steps**:
1. Create URDF file with base link
2. Add visual geometry (box, cylinder, or mesh)
3. Define collision geometry
4. Add inertial properties
5. Visualize in RViz2: `ros2 run rviz2 rviz2`

**Validation**: Robot model displays correctly in RViz2.

**Reasoning Question**: "Why do we need separate visual and collision geometries? What happens if they don't match?"

### Exercise 2: Launch Robot in Gazebo

**Objective**: Spawn robot model in Gazebo simulation.

**Steps**:
1. Install Gazebo and ROS 2 integration packages
2. Create launch file to start Gazebo
3. Spawn robot model in world
4. Verify robot appears in Gazebo GUI

**Validation**: Robot spawns and is visible in Gazebo.

**Reasoning Question**: "How does Gazebo load robot models? What's the difference between URDF and SDF in Gazebo?"

### Exercise 3: Control Robot via ROS 2

**Objective**: Control simulated robot using ROS 2 topics.

**Steps**:
1. Launch Gazebo with robot
2. Publish velocity commands to `/cmd_vel` topic
3. Observe robot movement in simulation
4. Use `ros2 topic echo` to monitor odometry

**Validation**: Robot moves in response to ROS 2 commands.

**Reasoning Question**: "How does ROS 2 control work in simulation? What's the same/different from physical hardware?"

### Exercise 4: Add Sensor to Robot

**Objective**: Simulate a camera or LiDAR sensor.

**Steps**:
1. Add sensor plugin to URDF/SDF
2. Configure sensor parameters (noise, update rate)
3. Launch simulation and verify sensor data
4. Visualize sensor output in RViz2

**Validation**: Sensor publishes data to ROS 2 topics.

**Reasoning Question**: "What sensor noise should I model? How does noise affect perception algorithms?"

### Exercise 5: Evaluate Simulation Realism

**Objective**: Critically assess simulation accuracy.

**Steps**:
1. Run robot through test scenario
2. Identify physics approximations
3. Compare simulation behavior to expected real-world behavior
4. Document limitations and assumptions

**Validation**: Understand simulation limitations and when they matter.

**Reasoning Question**: "How do I know if my simulation is realistic enough? What validation is needed?"

## Key Takeaways

1. **URDF Defines Robot Structure**: Links, joints, and properties define robot geometry and physics

2. **Physics Simulation Approximates Reality**: Understanding approximations is crucial for valid simulation results

3. **Gazebo-ROS 2 Integration**: Plugins bridge simulation and ROS 2 communication

4. **Sensor Simulation Requires Noise Models**: Realistic sensors include noise that affects algorithms

5. **Evaluation is Critical**: Always ask "How does this differ from reality?" when using simulation

6. **Simulation Enables Safe Testing**: Test dangerous scenarios and iterate quickly without hardware risk

## Summary

Week 1 established Gazebo as a tool for creating digital twins. You've learned to:
- Create robot models using URDF
- Build Gazebo worlds
- Integrate Gazebo with ROS 2
- Simulate basic sensors
- Evaluate simulation realism

**Critical Understanding**: Simulation is a tool that requires critical evaluation. A simulation that doesn't match reality can lead to false confidence. Always validate simulation assumptions and understand limitations.

**Next Steps**: Week 2 will introduce Unity for high-fidelity rendering and sim-to-real transfer principles.
