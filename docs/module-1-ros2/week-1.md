---
id: module-1-ros2-week-1
title: "Week 1: ROS 2 Fundamentals"
sidebar_position: 2
---

# Week 1: ROS 2 Fundamentals

## Introduction

Week 1 establishes the foundational concepts of ROS 2 through hands-on practice. You'll install ROS 2, create your first workspace, understand nodes and topics, and build your first ROS 2 nodes. This week emphasizes **manual practice**—you'll execute every command yourself to build mental models of how ROS 2 works.

**Learning Approach**: This week uses step-by-step walkthroughs with no AI assistance. You'll validate your own work and build intuition through practice. This foundation is critical for understanding distributed robotic systems.

## Topics Covered

### 1. ROS 2 Installation and Setup

**Installation**:
- Installing ROS 2 Humble on Ubuntu 22.04
- Setting up environment variables
- Verifying installation with `ros2` command

**Workspace Creation**:
- Understanding ROS 2 workspace structure (`src/`, `build/`, `install/`, `log/`)
- Creating your first workspace
- Using `colcon` build system

**Reasoning Question**: "Why do we need a workspace? What happens if I put packages in different locations?"

### 2. Understanding ROS 2 Nodes

**What is a Node?**:
- Nodes as independent processes
- Each node performs a specific task
- Nodes communicate via topics, services, and actions

**Node Lifecycle**:
- Creating a node
- Spinning a node (processing callbacks)
- Shutting down a node

**Manual Practice**: Create a simple "hello world" node that prints messages.

**Reasoning Question**: "Why are nodes separate processes? What happens if one node crashes?"

### 3. Topics and Publish/Subscribe Pattern

**Topics Explained**:
- Topics as communication channels
- Asynchronous, many-to-many communication
- Topic names and namespaces

**Publish/Subscribe**:
- Publishers send messages to topics
- Subscribers receive messages from topics
- Message types define data structure

**Common Topics**:
- `/cmd_vel` - Velocity commands
- `/camera/image` - Camera images
- `/scan` - LiDAR data
- `/odom` - Odometry data

**Manual Practice**: Create a publisher node and a subscriber node. Verify communication using CLI tools.

**Reasoning Question**: "Why does this node need to publish to that topic? What if multiple nodes subscribe to the same topic?"

### 4. ROS 2 CLI Tools

**Essential Commands**:
- `ros2 node list` - List running nodes
- `ros2 topic list` - List available topics
- `ros2 topic echo <topic>` - Print messages from a topic
- `ros2 topic info <topic>` - Get topic information
- `ros2 interface show <type>` - Show message definition

**Debugging Workflow**:
- Discovering what's running
- Inspecting message flow
- Verifying node communication

**Manual Practice**: Use CLI tools to introspect a running ROS 2 system.

**Reasoning Question**: "How do I know if my nodes are communicating correctly? What tools help me debug?"

### 5. Creating Your First Package

**Package Structure**:
- `package.xml` - Package metadata
- `setup.py` or `CMakeLists.txt` - Build configuration
- Source code organization

**Package Creation**:
- Using `ros2 pkg create` command
- Understanding package dependencies
- Building and installing packages

**Manual Practice**: Create a Python package with a simple node.

## Hands-On Exercises

### Exercise 1: ROS 2 Installation Verification

**Objective**: Verify ROS 2 is correctly installed and accessible.

**Steps**:
1. Open a terminal
2. Source ROS 2: `source /opt/ros/humble/setup.bash`
3. Verify installation: `ros2 --help`
4. Check version: `ros2 doctor`

**Validation**: All commands should execute without errors.

**Reasoning Question**: "What does `source` do? Why do I need to run it in every terminal?"

### Exercise 2: Create Your First Workspace

**Objective**: Set up a ROS 2 workspace for development.

**Steps**:
1. Create workspace directory: `mkdir -p ~/ros2_ws/src`
2. Navigate to workspace: `cd ~/ros2_ws`
3. Build empty workspace: `colcon build`
4. Source workspace: `source install/setup.bash`

**Validation**: Workspace builds successfully (even though it's empty).

**Reasoning Question**: "Why do we have `src/`, `build/`, and `install/` directories? What's the purpose of each?"

### Exercise 3: Create a Simple Publisher Node

**Objective**: Write a Python node that publishes messages to a topic.

**Steps**:
1. Create package: `ros2 pkg create --build-type ament_python my_first_package`
2. Create Python node file with publisher
3. Update `setup.py` to register the node
4. Build package: `colcon build --packages-select my_first_package`
5. Run node: `ros2 run my_first_package my_publisher_node`

**Validation**: Node runs and publishes messages (verify with `ros2 topic echo`).

**Reasoning Question**: "What happens if I publish to a topic with no subscribers? What if multiple nodes subscribe?"

### Exercise 4: Create a Subscriber Node

**Objective**: Write a node that subscribes to messages from a topic.

**Steps**:
1. Create subscriber node in same package
2. Update `setup.py`
3. Build and run subscriber
4. Verify it receives messages from publisher

**Validation**: Subscriber receives and prints messages from publisher.

**Reasoning Question**: "How does the subscriber know what message type to expect? What happens if types don't match?"

### Exercise 5: Use ROS 2 CLI for Introspection

**Objective**: Use CLI tools to understand a running ROS 2 system.

**Steps**:
1. Start your publisher and subscriber nodes
2. Use `ros2 node list` to see running nodes
3. Use `ros2 topic list` to see available topics
4. Use `ros2 topic echo` to see message flow
5. Use `ros2 topic info` to inspect topic details

**Validation**: CLI tools correctly show your nodes and topics.

**Reasoning Question**: "What information do these CLI tools reveal? How would I use them to debug a problem?"

## Key Takeaways

1. **ROS 2 Workspace Structure**: Understanding `src/`, `build/`, `install/` is fundamental to ROS 2 development

2. **Nodes are Independent Processes**: Each node runs separately, enabling distributed systems and fault isolation

3. **Topics Enable Loose Coupling**: Publishers and subscribers don't need to know about each other—they only need to agree on topic name and message type

4. **CLI Tools are Essential**: `ros2` command-line tools are your primary debugging and introspection mechanism

5. **Manual Practice Builds Intuition**: Executing commands yourself builds mental models that AI tools can't replace

6. **Message Types Define Contracts**: Topics use specific message types—understanding these types is crucial for system design

## Summary

Week 1 establishes the foundational concepts of ROS 2 through hands-on practice. You've learned to:
- Install and verify ROS 2
- Create workspaces and packages
- Understand nodes, topics, and the publish/subscribe pattern
- Use CLI tools for introspection and debugging
- Create your first publisher and subscriber nodes

**Critical Understanding**: ROS 2 nodes communicate asynchronously via topics. This loose coupling enables distributed systems where components can be developed, tested, and deployed independently. This architecture is the foundation for all Physical AI systems.

**Next Steps**: Week 2 will introduce ROS 2 services, custom message types, and more advanced distributed systems patterns.
