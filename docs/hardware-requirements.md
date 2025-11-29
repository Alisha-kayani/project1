---
id: hardware-requirements
title: Hardware Requirements
sidebar_position: 7
---

# Hardware Requirements

## Introduction

This section outlines the hardware requirements for the Physical AI & Humanoid Robotics course. The course is designed to work with simulation-first approaches, meaning you can complete most modules without physical robot hardware. However, for the capstone project and real-world deployment, specific robot hardware requirements are provided.

**Note**: The course progression (Modules 1-4) can be completed entirely in simulation. Physical hardware becomes necessary only for the capstone project and real-world validation.

## Minimum System Requirements

### Operating System

- **Linux**: Ubuntu 22.04 LTS (recommended) or Ubuntu 20.04 LTS
- **Windows**: Windows 10/11 with WSL2 (Windows Subsystem for Linux) for ROS 2
- **macOS**: macOS 12+ (limited ROS 2 support, simulation may have limitations)

**Why Linux?**: ROS 2 is primarily developed and tested on Linux. While Windows and macOS are possible, Linux provides the most stable experience.

### CPU

- **Minimum**: 4-core processor (Intel i5 or AMD Ryzen 5 equivalent)
- **Recommended**: 8-core processor (Intel i7/i9 or AMD Ryzen 7/9)
- **For Simulation**: More cores significantly improve Gazebo and Unity performance

### Memory (RAM)

- **Minimum**: 8 GB RAM
- **Recommended**: 16 GB RAM
- **For Advanced Simulation**: 32 GB RAM for complex Isaac Sim scenarios

### Storage

- **Minimum**: 50 GB free space
- **Recommended**: 100 GB+ free space for:
  - ROS 2 installation (~5 GB)
  - Gazebo models and worlds (~10 GB)
  - NVIDIA Isaac Sim (~20 GB)
  - Unity installation (~10 GB)
  - Development workspace and projects (~20 GB)
  - Docker images and containers (~15 GB)

### Graphics

- **Minimum**: Integrated graphics (for basic Gazebo)
- **Recommended**: Dedicated NVIDIA GPU with CUDA support
  - NVIDIA GTX 1060 or better (6 GB VRAM minimum)
  - For Isaac Sim: NVIDIA RTX 3060 or better (8 GB VRAM recommended)
  - CUDA 11.8+ support required for NVIDIA Isaac

**Why NVIDIA GPU?**: Module 3 (NVIDIA Isaac) requires CUDA-capable NVIDIA GPUs. AMD GPUs can work for Modules 1-2 but will limit Module 3 capabilities.

## Recommended Hardware

### Development Workstation

For optimal learning experience:

- **CPU**: 12-core+ processor (Intel i9 or AMD Ryzen 9)
- **RAM**: 32 GB
- **GPU**: NVIDIA RTX 3070/4070 or better (12 GB+ VRAM)
- **Storage**: 500 GB+ NVMe SSD
- **Network**: Gigabit Ethernet for downloading large simulation assets

### Laptop Considerations

Laptops can work but with limitations:

- **GPU**: NVIDIA RTX 3060 laptop GPU or better
- **Cooling**: Ensure adequate cooling for sustained simulation workloads
- **Battery**: Keep plugged in during intensive simulation sessions
- **External Monitor**: Recommended for better workspace management

## Robot Hardware

### Capstone Project Requirements

For the autonomous humanoid capstone project, you'll need:

#### Humanoid Robot Platform

**Option 1: Full Humanoid Robot**
- Humanoid form factor (bipedal, human-like proportions)
- ROS 2 compatible (ROS 2 Humble or later)
- Joint control via ROS 2 topics/services
- Examples: Unitree Go1, Boston Dynamics Spot (if accessible), custom builds

**Option 2: Humanoid Robot Kit**
- Modular humanoid kits (e.g., Robotis OP3, Trossen Robotics kits)
- ROS 2 support available
- Programmable joints and sensors

**Option 3: Simulation-Only Capstone**
- Complete capstone in Isaac Sim or Gazebo
- No physical hardware required
- Validates all software integration concepts

#### Required Sensors

- **Vision**: RGB camera(s) for perception
- **Depth Sensing**: LiDAR or depth camera (optional but recommended)
- **IMU**: Inertial Measurement Unit for balance and orientation
- **Joint Encoders**: Position feedback for all actuators

#### Actuators

- **Servo Motors**: Position-controlled actuators for joints
- **Torque Control**: Optional but enables advanced manipulation
- **Safety**: Emergency stop capability

### Budget Considerations

**Simulation-Only Path**: $0 (uses free software)
- Complete Modules 1-4 entirely in simulation
- Capstone project in Gazebo/Isaac Sim

**Entry-Level Hardware**: $500-$2,000
- Basic humanoid kit or robot arm
- Sufficient for learning and capstone validation

**Professional Setup**: $5,000-$20,000+
- Full humanoid platform
- High-end sensors and actuators
- Production-grade capabilities

## Development Environment

### Software Stack

The course uses the following software (installation covered in respective modules):

1. **ROS 2 Humble** (Module 1)
   - Ubuntu 22.04 native
   - Windows: WSL2 + ROS 2
   - macOS: Docker-based (limited)

2. **Gazebo Classic/Garden** (Module 2)
   - Physics simulation engine
   - URDF/SDF model support

3. **Unity** (Module 2)
   - High-fidelity rendering
   - ROS 2 integration via Unity Robotics packages

4. **NVIDIA Isaac Sim** (Module 3)
   - Requires NVIDIA GPU with CUDA
   - Photorealistic simulation
   - Synthetic data generation

5. **Docker** (Optional but recommended)
   - Containerized development environments
   - Consistent setup across machines

### Network Requirements

- **Internet**: Stable connection for:
  - Downloading ROS 2 packages
  - Installing simulation assets
  - Accessing course materials
- **Bandwidth**: 10+ Mbps recommended for large downloads
- **Firewall**: May need to allow ROS 2 DDS communication (ports 7400-7500)

### Development Tools

- **Code Editor**: VS Code with ROS 2 extensions (recommended)
- **Version Control**: Git for project management
- **Terminal**: Linux terminal or WSL2 terminal
- **Package Manager**: `apt` (Linux), `choco` (Windows), `homebrew` (macOS)

## Summary

The Physical AI & Humanoid Robotics course is designed to be accessible:

- **Minimum Setup**: Modern laptop with Linux (or WSL2) can complete Modules 1-2
- **Recommended Setup**: Desktop/workstation with NVIDIA GPU enables full course including Module 3
- **Hardware Optional**: Physical robot hardware only needed for capstone real-world deployment
- **Simulation-First**: Complete the entire course in simulation if hardware is unavailable

**Key Takeaway**: Start with what you have. The course progression allows you to begin with minimal hardware and scale up as needed. Simulation provides a safe, scalable learning environment before investing in physical hardware.
