# Module 2: The Digital Twin (Gazebo & Unity)

## Learning Objectives

By the end of this module, you will be able to:
- Set up and configure Gazebo for robot simulation
- Create physics-based simulation worlds with obstacles and environment elements
- Design URDF models and convert them to SDF format
- Simulate realistic sensors (LiDAR, depth cameras, IMU) in Gazebo
- Interface Gazebo simulations with ROS 2 topics and services
- Understand simulation-to-reality transfer principles
- Use Unity for visualization and basic simulation workflows

## Module Overview

Module 2 focuses on creating digital twins of physical robots using physics simulation. A digital twin is a virtual replica of a robot that behaves like its physical counterpart, allowing developers to test algorithms safely before deployment.

### Key Topics
- Gazebo simulation framework (setup, worlds, plugins)
- URDF to SDF conversion and best practices
- Sensor simulation (depth cameras, LiDAR, IMU, cameras)
- ROS 2 integration with Gazebo
- Visualization with Rviz2 and optional Unity workflows
- Recording and playback with rosbag2
- Performance optimization and debugging

## Chapter 1: Gazebo Setup and Configuration

### What is Gazebo?

Gazebo is a powerful open-source 3D robotics simulator that provides:
- **Physics Engine**: Accurate simulation of rigid body dynamics using ODE, Bullet, or DART
- **Sensor Simulation**: Realistic sensor output including noise and distortion
- **ROS 2 Integration**: Native topic/service support for ROS communication
- **Rendering**: High-fidelity visualization with Ignition Rendering
- **Extensibility**: Plugin system for custom physics, sensors, and behaviors

### Installation and Setup

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

### Creating Your First Gazebo World

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

### Physics Configuration

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

## Chapter 2: URDF to SDF Conversion

### Understanding URDF vs SDF

| Feature | URDF | SDF |
|---------|------|-----|
| **Format** | XML for robots | XML for worlds + robots |
| **Physics** | Basic | Advanced |
| **Sensors** | Limited | Full support |
| **Simulation** | Design oriented | Simulation oriented |
| **Use Case** | ROS description | Gazebo/sim environments |

### Converting URDF to SDF

**Original URDF (2-link arm):**
```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <link name="link1">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <origin xyz="0 0 0.15"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <origin xyz="0 0 0.15"/>
    </collision>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="10" velocity="1.0"/>
    <origin xyz="0 0 0.1"/>
  </joint>
</robot>
```

**Automated conversion:**
```bash
# Using gz (Gazebo) tool
gz sdf convert model.urdf > model.sdf

# Or use the URDF parser
rosrun gazebo_ros gazebo_model_spawner.py -urdf -param robot_description -model my_robot
```

### SDF Best Practices

1. **Always include collisions** for physics simulation
2. **Set accurate masses and inertias** for realistic dynamics
3. **Use plugins** for sensors and custom behaviors
4. **Define friction coefficients** for accurate contact simulation
5. **Test in headless mode** for CI/CD integration

## Chapter 3: Sensor Simulation

### Depth Camera (RealSense-like)

**SDF plugin definition:**
```xml
<model name="camera">
  <pose>0.1 0 0.5 0 0 0</pose>
  <link name="camera_link">
    <sensor name="depth_camera" type="depth_camera">
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
      <plugin filename="ignition-gazebo-depth-camera-system" name="ignition::gazebo::systems::DepthCameraSensor">
        <output_type>depth</output_type>
      </plugin>
    </sensor>
  </link>
</model>
```

**Subscribe to depth data in ROS 2:**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class DepthListener(Node):
    def __init__(self):
        super().__init__('depth_listener')
        self.subscription = self.create_subscription(
            Image,
            '/depth_camera/depth',
            self.depth_callback,
            10)
    
    def depth_callback(self, msg):
        # Convert ROS Image to numpy array
        depth_array = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
        self.get_logger().info(f'Depth range: {depth_array.min():.2f}m - {depth_array.max():.2f}m')

if __name__ == '__main__':
    rclpy.init()
    listener = DepthListener()
    rclpy.spin(listener)
    rclpy.shutdown()
```

### LiDAR Sensor

**SDF plugin:**
```xml
<sensor name="lidar" type="lidar">
  <pose>0 0 0.15 0 0 0</pose>
  <lidar>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1.0</resolution>
        <min_angle>0.0</min_angle>
        <max_angle>6.28318</max_angle>
      </horizontal>
      <vertical>
        <samples>1</samples>
        <resolution>1.0</resolution>
        <min_angle>0.0</min_angle>
        <max_angle>0.0</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.08</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
  </lidar>
  <always_on>1</always_on>
  <visualize>true</visualize>
  <update_rate>10</update_rate>
  <plugin filename="ignition-gazebo-lidar-system" name="ignition::gazebo::systems::Lidar">
  </plugin>
</sensor>
```

### IMU Sensor

**SDF plugin:**
```xml
<sensor name="imu" type="imu">
  <pose>0 0 0 0 0 0</pose>
  <always_on>1</always_on>
  <update_rate>250</update_rate>
  <visualize>false</visualize>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.02</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.02</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.02</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
  <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu">
  </plugin>
</sensor>
```

**Read IMU data:**
```python
from sensor_msgs.msg import Imu

def imu_callback(msg: Imu):
    print(f"Linear acceleration: {msg.linear_acceleration}")
    print(f"Angular velocity: {msg.angular_velocity}")
    print(f"Orientation: {msg.orientation}")

node.create_subscription(Imu, '/imu', imu_callback, 10)
```

## Chapter 4: ROS 2 Integration with Gazebo

### Spawning Models at Runtime

**Python script to spawn a robot:**
```python
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import xacro

class ModelSpawner(Node):
    def __init__(self):
        super().__init__('model_spawner')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.spawn_client.wait_for_service()
    
    def spawn_model(self, urdf_path, model_name, x=0.0, y=0.0, z=1.0):
        # Load and parse URDF
        doc = xacro.process_file(urdf_path)
        urdf_string = xacro.process_file(urdf_path).toxml()
        
        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = urdf_string
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z
        request.initial_pose.orientation.w = 1.0
        
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            self.get_logger().info(f'Spawned {model_name}')
        else:
            self.get_logger().error(f'Failed to spawn {model_name}')

if __name__ == '__main__':
    rclpy.init()
    spawner = ModelSpawner()
    spawner.spawn_model('robot.urdf.xacro', 'my_robot', x=0.0, y=0.0, z=0.5)
    rclpy.shutdown()
```

### Controlling Models with ROS 2

**Publish twist commands to move a robot:**
```python
import rclpy
from geometry_msgs.msg import Twist

def move_robot():
    rclpy.init()
    node = rclpy.create_node('robot_commander')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    
    twist = Twist()
    twist.linear.x = 0.5  # Move forward
    twist.angular.z = 0.2  # Rotate
    
    rate = node.create_rate(10)  # 10 Hz
    for _ in range(50):
        publisher.publish(twist)
        rate.sleep()
    
    rclpy.shutdown()

if __name__ == '__main__':
    move_robot()
```

### Recording and Playback with rosbag2

**Record all topics:**
```bash
ros2 bag record -a -o my_simulation

# Or record specific topics
ros2 bag record -o my_sim /cmd_vel /odom /imu /depth_camera/depth
```

**Playback:**
```bash
ros2 bag play my_simulation
```

**Analyze bag file:**
```bash
ros2 bag info my_simulation
```

## Chapter 5: Performance Optimization

### Tips for Faster Simulation

1. **Reduce Physics Update Rate** (if accuracy allows):
```xml
<max_step_size>0.002</max_step_size>  <!-- 500 Hz instead of 1000 Hz -->
```

2. **Simplify Meshes** - Use primitive shapes instead of complex meshes

3. **Disable Rendering** in headless mode:
```bash
gazebo -s libignition_gazebo_headless_system.so worlds/empty.world
```

4. **Parallel Threads:**
```bash
export IGN_GAZEBO_THREAD_POOL_SIZE=8
```

5. **Reduce Sensor Update Rates:**
```xml
<update_rate>5</update_rate>  <!-- 5 Hz instead of 30 Hz if possible -->
```

## Exercises

### Lab 2.1: Create and Spawn a Robot

**Objective**: Create a URDF model of a 2-joint arm and spawn it in Gazebo

**Steps**:
1. Create `robot_arm.urdf` with 2 revolute joints
2. Create a Gazebo world file
3. Write a Python script to spawn the robot
4. Verify the robot appears in Gazebo

**Expected Output**: A 2-link arm visible in Gazebo interface

### Lab 2.2: Simulate Sensors

**Objective**: Add a depth camera and LiDAR to the robot, and subscribe to their data

**Steps**:
1. Add sensor plugins to the URDF
2. Create a ROS 2 node that subscribes to `/depth_camera/depth` and `/scan`
3. Print depth range and scan data to console
4. Run simulation and verify data is being published

**Expected Output**:
```
Depth range: 0.10m - 3.50m
LiDAR scan: 360 points at 0.1m - 10.0m
```

## Key Takeaways

✓ Gazebo provides physics-based simulation with ROS 2 integration
✓ URDF models must be converted to SDF for use in Gazebo
✓ Sensors can be simulated with realistic noise and distortion
✓ ROS 2 topics and services enable bidirectional communication
✓ Simulation is essential for safe testing before hardware deployment

## References

- [Gazebo Official Documentation](https://gazebosim.org/)
- [ROS 2 Gazebo Integration](https://github.com/ros-simulation/gazebo_ros2_control)
- [URDF / SDF Specification](https://wiki.ros.org/urdf)
- [gazebo_ros Package](http://wiki.ros.org/gazebo_ros)
