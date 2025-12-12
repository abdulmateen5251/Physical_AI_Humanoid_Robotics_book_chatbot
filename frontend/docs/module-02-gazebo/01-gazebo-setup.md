# Chapter 1: Gazebo Setup and Configuration

## What is Gazebo?

Gazebo is a powerful open-source 3D robotics simulator that provides:
- **Physics Engine**: Accurate simulation of rigid body dynamics using ODE, Bullet, or DART
- **Sensor Simulation**: Realistic sensor output including noise and distortion
- **ROS 2 Integration**: Native topic/service support for ROS communication
- **Rendering**: High-fidelity visualization with Ignition Rendering
- **Extensibility**: Plugin system for custom physics, sensors, and behaviors

## Installation and Setup

**On Ubuntu 22.04 with ROS 2 Humble:**

```bash
# Install Gazebo
sudo apt-get install ros-humble-gazebo-ros-pkgs

# Install additional simulation packages
sudo apt-get install ros-humble-gazebo-ros2-control
sudo apt-get install ros-humble-gazebo-plugins

# Verify installation
gazebo --version
```

**Environment Setup:**

```bash
# Source ROS 2 and Gazebo
source /opt/ros/humble/setup.bash

# Export common paths
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/.gazebo/models:/opt/ros/humble/share/gazebo_ros/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/opt/ros/humble/share/gazebo_ros
```

## Creating Your First Gazebo World

A Gazebo world file defines the environment, physics, and initial state. Here's a minimal example:

**worlds/empty.world:**
```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="empty_world">
    <!-- Physics settings -->
    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Light -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.5 -1</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

**Launch Gazebo with this world:**
```bash
gazebo worlds/empty.world
```

## Physics Configuration

The physics section controls simulation accuracy and speed:

```xml
<physics name="default_physics" default="true" type="ode">
  <!-- Time step (seconds) - smaller = more accurate but slower -->
  <max_step_size>0.001</max_step_size>
  
  <!-- Real-time factor: 1.0 = realtime, 2.0 = 2x speed -->
  <real_time_factor>1.0</real_time_factor>
  
  <!-- Update rate (Hz) -->
  <real_time_update_rate>1000</real_time_update_rate>
  
  <!-- Solver iterations for collision detection -->
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <precon_iters>0</precon_iters>
      <precon_type>identity</precon_type>
      <use_dynamic_moi_rescaling>false</use_dynamic_moi_rescaling>
    </solver>
    <constraints>
      <cfm>0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## Key Concepts

### Real-Time Factor
Controls simulation speed relative to wall-clock time:
- **1.0**: Simulation runs at real-time speed
- **< 1.0**: Slower than real-time (more accurate)
- **> 1.0**: Faster than real-time (for quick testing)

### Max Step Size
Defines the time step for physics integration:
- **0.001s (1ms)**: Default, good balance
- **0.0001s (0.1ms)**: Higher accuracy, slower
- **0.01s (10ms)**: Faster, less accurate

### ODE Solver Parameters
- **iters**: Collision solver iterations (higher = more accurate)
- **erp**: Error reduction parameter (0-1, higher = more correction)
- **cfm**: Constraint force mixing (prevents instability)
