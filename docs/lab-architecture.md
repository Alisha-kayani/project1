---
id: lab-architecture
title: Lab Architecture
sidebar_position: 8
---

# Lab Architecture

## Introduction

This section describes the lab architecture and setup for the Physical AI & Humanoid Robotics course. The lab environment is designed to support both individual learning and collaborative development, with a focus on simulation-first workflows that scale to physical hardware deployment.

**Architecture Philosophy**: The lab setup mirrors production robotics environments, teaching you industry-standard practices for developing, testing, and deploying Physical AI systems.

## Lab Environment Overview

### Development Modes

The course supports three development modes:

1. **Local Development** (Modules 1-2)
   - Single-machine setup
   - ROS 2 on local host
   - Gazebo simulation locally
   - Ideal for learning fundamentals

2. **Distributed Simulation** (Module 3)
   - Multiple machines or containers
   - ROS 2 DDS across network
   - Isaac Sim on GPU-enabled nodes
   - Mirrors production multi-robot scenarios

3. **Hybrid Physical-Simulation** (Module 4 & Capstone)
   - Physical robots + simulation validation
   - ROS 2 bridging real and simulated components
   - Sim-to-real transfer validation

### Container Architecture

**Docker-Based Setup** (Recommended):
- Consistent environments across machines
- Isolated ROS 2 workspaces
- Reproducible simulation environments
- Easy sharing of configurations

**Container Components**:
- ROS 2 base images (ros:humble-desktop)
- Gazebo simulation containers
- Isaac Sim containers (NVIDIA GPU required)
- Development tools (VS Code, debugging tools)

## Network Architecture

### ROS 2 DDS Communication

ROS 2 uses DDS (Data Distribution Service) for inter-process communication:

**Network Requirements**:
- **Discovery Ports**: 7400-7500 (UDP)
- **Data Ports**: Dynamic assignment (TCP/UDP)
- **Multicast**: Required for automatic node discovery
- **Firewall**: Must allow ROS 2 traffic for distributed setups

**Network Topology**:
```
[Development Machine] ←→ [ROS 2 DDS Domain] ←→ [Simulation Nodes]
                              ↓
                    [Physical Robot] (if connected)
```

### Local vs Distributed

**Local Development** (Default):
- All nodes on same machine
- DDS uses shared memory (fast)
- No network configuration needed
- Ideal for Modules 1-2

**Distributed Setup**:
- Nodes across multiple machines
- DDS uses network transport
- Requires network configuration
- Needed for Module 3+ and multi-robot scenarios

**Configuration**:
- Set `ROS_DOMAIN_ID` environment variable
- Isolate different projects/environments
- Default domain: 0 (all nodes discover each other)

## Software Stack

### Layer 1: Operating System

- **Primary**: Ubuntu 22.04 LTS
- **Alternative**: Ubuntu 20.04 LTS (ROS 2 Humble)
- **Container**: Docker with Ubuntu base images

### Layer 2: ROS 2 Middleware

- **ROS 2 Distribution**: Humble Hawksbill (LTS)
- **DDS Implementation**: Fast DDS (default) or Cyclone DDS
- **Build System**: Colcon (CMake + Python)
- **Package Management**: `apt` for system packages, `rosdep` for ROS packages

### Layer 3: Simulation Engines

**Gazebo**:
- Gazebo Classic (stable, widely used)
- Gazebo Garden (newer, improved physics)
- URDF/SDF model support
- ROS 2 integration via `gazebo_ros_pkgs`

**Unity**:
- Unity 2022 LTS or later
- Unity Robotics packages
- ROS 2 integration via ROS-TCP-Connector
- High-fidelity rendering

**NVIDIA Isaac Sim**:
- Isaac Sim 2023.1+ (requires NVIDIA GPU)
- Omniverse-based
- Photorealistic simulation
- Isaac ROS integration

### Layer 4: AI/ML Tools

**NVIDIA Isaac**:
- Isaac Sim: Simulation and synthetic data
- Isaac ROS: Hardware-accelerated perception
- Isaac GEMs: Pre-built robotics components

**VLA Integration**:
- OpenAI Whisper: Voice-to-text
- LLM APIs: GPT-4, Claude, or local models
- ROS 2 action servers: Bridge LLM to robot control

### Layer 5: Development Tools

**Code Development**:
- VS Code with ROS 2 extensions
- Python 3.10+ (ROS 2 Humble requirement)
- C++17 compiler (for C++ nodes)

**Version Control**:
- Git for project management
- GitHub/GitLab for collaboration
- ROS 2 workspace structure compatible with Git

**Debugging**:
- `ros2` CLI tools for introspection
- `rviz2` for visualization
- `rqt` for GUI tools
- GDB/LLDB for C++ debugging

## Development Workflow

### Workspace Structure

Standard ROS 2 workspace layout:

```
~/ros2_ws/
├── src/                    # Source packages
│   ├── my_robot_pkg/       # Your robot package
│   └── ...
├── build/                  # Build artifacts (gitignored)
├── install/                # Installed packages (gitignored)
└── log/                    # Build logs (gitignored)
```

### Typical Workflow

1. **Setup Workspace**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

2. **Create/Clone Package**:
   ```bash
   cd src
   ros2 pkg create --build-type ament_python my_package
   # or clone existing package
   ```

3. **Install Dependencies**:
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build**:
   ```bash
   colcon build
   source install/setup.bash
   ```

5. **Run/Test**:
   ```bash
   ros2 run my_package my_node
   ```

### Simulation Workflow

**Module 1-2 (Gazebo)**:
1. Launch Gazebo with robot model
2. Start ROS 2 nodes
3. Monitor topics/services
4. Debug using `ros2` CLI tools

**Module 3 (Isaac Sim)**:
1. Launch Isaac Sim
2. Load scene and robot
3. Start Isaac ROS nodes
4. Monitor perception pipelines
5. Validate sim-to-real transfer

**Module 4 (VLA Integration)**:
1. Start ROS 2 system
2. Launch LLM integration node
3. Test voice commands
4. Validate action execution
5. Iterate on cognitive planning

### Version Control Strategy

**What to Commit**:
- Source code (`src/` directory)
- Configuration files (URDF, launch files, configs)
- Documentation
- Package manifests

**What to Ignore**:
- Build artifacts (`build/`, `install/`, `log/`)
- Generated files
- Large simulation assets (use Git LFS if needed)
- IDE-specific files

### Collaboration Patterns

**Shared Workspace**:
- Use consistent `ROS_DOMAIN_ID` for team
- Share package repositories
- Document environment setup

**Code Review**:
- Review ROS 2 node implementations
- Validate message/service definitions
- Test launch file configurations

## Summary

The lab architecture for Physical AI & Humanoid Robotics is designed to:

- **Support Progressive Learning**: Start simple (local) and scale to complex (distributed)
- **Mirror Production**: Use industry-standard tools and practices
- **Enable Collaboration**: Shared workspaces and version control
- **Facilitate Simulation**: Simulation-first approach reduces hardware dependencies

**Key Principles**:
- **Reproducibility**: Docker containers ensure consistent environments
- **Scalability**: Architecture supports single-machine to multi-robot deployments
- **Modularity**: Each layer (OS, ROS 2, simulation, AI) is independently configurable
- **Industry Alignment**: Tools and practices match real-world robotics development

This architecture prepares you for production robotics environments where distributed systems, simulation validation, and collaborative development are standard practices.
