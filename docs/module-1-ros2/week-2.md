---
id: module-1-ros2-week-2
title: "Week 2: ROS 2 Topics and Services"
sidebar_position: 3
---

# Week 2: ROS 2 Services and Advanced Patterns

## Introduction

Week 2 builds on Week 1's foundation by introducing ROS 2 services (synchronous request/response communication), custom message types, and advanced distributed systems patterns. You'll learn when to use topics vs services, how to define custom message types, and how to manage multi-node systems with launch files.

**Learning Approach**: Continue with manual practice, but now you'll make design decisions: "Should I use a topic or a service for this communication?" These decisions build your understanding of distributed systems architecture.

## Topics Covered

### 1. ROS 2 Services

**Services vs Topics**:
- **Topics**: Asynchronous, one-way, many-to-many (publish/subscribe)
- **Services**: Synchronous, request/response, one-to-one (client/server)

**When to Use Services**:
- Requesting specific information (e.g., "get current position")
- Triggering actions that need confirmation (e.g., "set robot mode")
- Operations that require immediate response

**Service Definition**:
- Service files define request and response types
- Services are defined in `.srv` files
- Similar structure to message types but with request/response separation

**Manual Practice**: Create a service server and client. Understand the synchronous nature.

**Reasoning Question**: "When should I use a service instead of a topic? What are the trade-offs?"

### 2. Custom Message and Service Types

**Message Types**:
- Standard message types (`std_msgs`, `geometry_msgs`, `sensor_msgs`)
- Creating custom message types for robot-specific data
- Message definition files (`.msg`)

**Service Types**:
- Standard service types
- Creating custom service types
- Service definition files (`.srv`)

**Package Structure for Custom Types**:
- `msg/` directory for message definitions
- `srv/` directory for service definitions
- `CMakeLists.txt` and `package.xml` configuration

**Manual Practice**: Create a custom message type for your robot's sensor data.

**Reasoning Question**: "What information should be in a message? How do I design message types for my robot?"

### 3. Understanding DDS and Distributed Communication

**DDS (Data Distribution Service)**:
- The underlying middleware for ROS 2
- Enables distributed communication across network
- Discovery mechanism for automatic node finding

**ROS 2 Domain ID**:
- Isolates different ROS 2 networks
- Set via `ROS_DOMAIN_ID` environment variable
- Default: 0 (all nodes discover each other)

**Local vs Distributed**:
- **Local**: All nodes on same machine (uses shared memory)
- **Distributed**: Nodes across network (uses network transport)

**Manual Practice**: Run nodes on different machines (or simulate with different domain IDs).

**Reasoning Question**: "How does ROS 2 find nodes on the network? What happens if nodes are on different machines?"

### 4. Launch Files

**Why Launch Files?**:
- Start multiple nodes together
- Configure node parameters
- Organize complex systems

**Launch File Structure**:
- XML or Python format (Python recommended)
- Node declarations
- Parameter setting
- Namespace management

**Common Patterns**:
- Launching sensor nodes
- Starting robot drivers
- Coordinating multiple components

**Manual Practice**: Create a launch file that starts multiple nodes for a simple robot system.

**Reasoning Question**: "How do launch files help manage complexity? What's the difference between launching nodes individually vs with a launch file?"

### 5. Debugging Distributed Systems

**Common Issues**:
- Nodes not discovering each other
- Message type mismatches
- Network connectivity problems
- DDS configuration issues

**Debugging Tools**:
- `ros2 node list` - See what's running
- `ros2 topic list` - See available topics
- `ros2 service list` - See available services
- `ros2 doctor` - Diagnose system issues
- `ros2 topic hz` - Check message frequency

**Debugging Workflow**:
1. Verify nodes are running
2. Check topic/service availability
3. Inspect message flow
4. Validate message types
5. Check network/DDS configuration

**Manual Practice**: Intentionally break something and debug it using ROS 2 tools.

**Reasoning Question**: "How do I systematically debug a distributed system? What tools help me understand what's happening?"

## Hands-On Exercises

### Exercise 1: Create a Service Server and Client

**Objective**: Implement synchronous request/response communication.

**Steps**:
1. Create a custom service type (e.g., `GetPosition.srv`)
2. Create service server node that responds to requests
3. Create service client node that makes requests
4. Build and test the service communication

**Validation**: Client successfully receives response from server.

**Reasoning Question**: "What makes services synchronous? How is this different from topics?"

### Exercise 2: Define Custom Message Types

**Objective**: Create robot-specific message types.

**Steps**:
1. Create `msg/` directory in your package
2. Define a custom message (e.g., `RobotState.msg`)
3. Update `CMakeLists.txt` and `package.xml`
4. Build and use the custom message type

**Validation**: Custom message type compiles and can be used in nodes.

**Reasoning Question**: "What information belongs in a message? How do I design message types for my robot?"

### Exercise 3: Multi-Node System with Launch File

**Objective**: Coordinate multiple nodes using a launch file.

**Steps**:
1. Create multiple nodes (publisher, subscriber, service server)
2. Create a Python launch file
3. Launch all nodes together
4. Verify system operation

**Validation**: All nodes start and communicate correctly via launch file.

**Reasoning Question**: "How do launch files simplify system management? What are the benefits of coordinated startup?"

### Exercise 4: Distributed Communication

**Objective**: Understand ROS 2 distributed communication.

**Steps**:
1. Set different `ROS_DOMAIN_ID` values
2. Verify nodes in different domains don't see each other
3. Set same domain ID and verify communication
4. Use `ros2 doctor` to diagnose issues

**Validation**: Understand how domain IDs isolate ROS 2 networks.

**Reasoning Question**: "How does ROS 2 enable distributed systems? What are the implications for multi-robot scenarios?"

### Exercise 5: Debug a Broken System

**Objective**: Practice systematic debugging.

**Steps**:
1. Create a system with intentional bugs (wrong topic names, mismatched types)
2. Use ROS 2 CLI tools to identify problems
3. Fix issues systematically
4. Document debugging process

**Validation**: Successfully identify and fix all issues using ROS 2 tools.

**Reasoning Question**: "What's a systematic approach to debugging distributed systems? How do ROS 2 tools help?"

## Key Takeaways

1. **Services for Synchronous Communication**: Use services when you need request/response patterns, not just one-way data flow

2. **Custom Types Define Contracts**: Message and service types are contracts between nodes—design them carefully

3. **DDS Enables Distribution**: ROS 2's DDS layer enables nodes to communicate across networks automatically

4. **Launch Files Manage Complexity**: Launch files coordinate multi-node systems and simplify deployment

5. **Systematic Debugging**: Use ROS 2 CLI tools methodically to understand and fix distributed system issues

6. **Design Decisions Matter**: Choosing topics vs services, message types, and system architecture requires reasoning about requirements

## Summary

Week 2 advanced your ROS 2 understanding with services, custom types, and distributed systems patterns. You've learned to:
- Use services for synchronous request/response communication
- Define custom message and service types
- Understand DDS and distributed communication
- Create launch files for multi-node systems
- Debug distributed systems systematically

**Critical Understanding**: ROS 2 provides both asynchronous (topics) and synchronous (services) communication patterns. Choosing the right pattern depends on your system's requirements. This understanding is essential for designing Physical AI systems where different components need different communication patterns.

**Module 1 Complete**: You now have the foundational "nervous system" knowledge. Module 2 will introduce simulation environments where you'll apply ROS 2 to control simulated robots.
