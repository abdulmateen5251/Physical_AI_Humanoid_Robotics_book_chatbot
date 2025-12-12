---
sidebar_position: 1
title: Introduction to ROS 2
description: Learn about ROS 2, the next generation Robot Operating System
---

# Introduction to ROS 2

## Course Overview

Welcome to the Physical AI & Humanoid Robotics course! This comprehensive guide will take you from ROS 2 fundamentals through advanced topics in physical AI, simulation with Gazebo and Isaac Sim, and culminating in Vision-Language-Action (VLA) models for humanoid robotics.

## What is ROS 2?

**ROS 2 (Robot Operating System 2)** is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

### Key Features

- **Middleware Communication**: Built on top of DDS (Data Distribution Service) for reliable, real-time communication
- **Cross-Platform**: Supports Linux, Windows, and macOS
- **Real-Time Capable**: Designed with real-time systems in mind
- **Security**: Built-in security features including authentication and encryption
- **Modular Architecture**: Create reusable, composable software components

### ROS 1 vs ROS 2

ROS 2 represents a complete redesign of ROS 1, addressing many limitations:

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| **Communication** | Custom TCPROS | DDS standard |
| **Real-time** | Limited support | First-class support |
| **Platform** | Primarily Linux | Linux, Windows, macOS |
| **Security** | Minimal | Built-in (SROS2) |
| **Python Version** | Python 2.7 | Python 3.6+ |
| **Build System** | catkin | colcon + ament |

## Why ROS 2 for Physical AI?

Physical AI systems require:

1. **Real-time Performance**: Humanoid robots need to react to their environment instantly
2. **Modularity**: Complex behaviors built from simple, testable components
3. **Scalability**: From single robots to fleets
4. **Reliability**: Safety-critical applications demand robust communication
5. **Industry Standards**: DDS is used in aerospace, automotive, and industrial automation

ROS 2 excels in all these areas, making it the ideal foundation for physical AI development.

## Course Learning Objectives

By the end of this course, you will be able to:

### Module 1: ROS 2 Fundamentals
- ✓ Understand ROS 2 architecture and core concepts
- ✓ Create nodes, publishers, and subscribers in Python (rclpy)
- ✓ Implement services and actions for robot control
- ✓ Configure Quality of Service (QoS) settings
- ✓ Use ROS 2 command-line tools for debugging

### Module 2: Robot Simulation with Gazebo
- ✓ Set up Gazebo simulation environments
- ✓ Create and import URDF robot models
- ✓ Integrate sensors (cameras, LiDAR, IMU)
- ✓ Implement physics-based robot control
- ✓ Test algorithms in simulation before deployment

### Module 3: NVIDIA Isaac Sim
- ✓ Leverage GPU-accelerated simulation
- ✓ Create photorealistic robot environments
- ✓ Generate synthetic training data
- ✓ Simulate complex sensor arrays
- ✓ Scale to thousands of parallel simulations

### Module 4: Vision-Language-Action Models
- ✓ Understand VLA architecture and training
- ✓ Integrate vision transformers with language models
- ✓ Implement end-to-end robot policies
- ✓ Fine-tune VLAs for specific tasks
- ✓ Deploy VLAs on humanoid robots

## Prerequisites

To succeed in this course, you should have:

- **Python Programming**: Intermediate level (classes, async/await, decorators)
- **Linux Command Line**: Basic familiarity with terminal commands
- **Linear Algebra**: Vectors, matrices, transformations
- **Basic Robotics**: Understanding of sensors, actuators, and control loops (helpful but not required)

## Development Environment

You'll need:

- **Operating System**: Ubuntu 22.04 LTS (recommended) or Ubuntu 24.04
- **ROS 2 Distribution**: Humble Hawksbill (LTS) or Iron Irwini
- **Python**: 3.10 or 3.11
- **Hardware**: 
  - CPU: Multi-core processor (Intel i5/Ryzen 5 or better)
  - RAM: 16GB minimum, 32GB recommended
  - GPU: NVIDIA GPU with 8GB+ VRAM (for Module 3-4)
  - Storage: 50GB free space

## Installation Quick Start

### Ubuntu 22.04 + ROS 2 Humble

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop python3-argcomplete
sudo apt install ros-dev-tools

# Environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Verify Installation

```bash
# Check ROS 2 installation
ros2 --version

# Test with turtlesim
ros2 run turtlesim turtlesim_node

# In another terminal
ros2 run turtlesim turtle_teleop_key
```

If you see the turtle and can control it with arrow keys, your installation is successful!

## Course Structure

Each module follows this pattern:

1. **Conceptual Introduction**: Understand the "why" before the "how"
2. **Hands-On Examples**: Learn by building real projects
3. **Code Walkthroughs**: Detailed explanation of every line
4. **Exercises**: Reinforce learning with challenges
5. **Mini-Projects**: Apply concepts to solve realistic problems

## Getting Help

Throughout this course, you can:

- **Ask the Chatbot**: Click the chat icon to ask questions about any topic
- **Selection Mode**: Highlight text and ask specific questions about it
- **Code Examples**: All code is tested and ready to run
- **Troubleshooting**: Common errors and solutions are documented

## Next Steps

Ready to dive in? Let's start with ROS 2 fundamentals:

→ **Next Chapter**: [Nodes, Topics, and Services](./02-nodes-topics-services.md)

---

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS Discourse Community](https://discourse.ros.org/)
- [ROS 2 GitHub Repository](https://github.com/ros2)

## Learning Tips

1. **Hands-On Practice**: Type out code examples instead of copy-pasting
2. **Break Things**: Experiment and learn from errors
3. **Build Projects**: Apply concepts to your own robot ideas
4. **Join Community**: Engage with other learners and experts
5. **Stay Updated**: ROS 2 is actively developed; follow release notes

Let's begin your journey into Physical AI and Humanoid Robotics! 🤖
