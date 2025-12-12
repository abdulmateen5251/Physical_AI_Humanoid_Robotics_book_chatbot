# Chapter 5: Labs and Exercises

## Lab 2.1: Spawn a 2-DOF Arm in Gazebo

**Objective**: Create a simple 2-joint robotic arm and spawn it in Gazebo

**URDF Definition (arm.urdf):**
```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <link name="base_link">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
      <material name="base_material">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <link name="link1">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
      <material name="link_material">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
    </collision>
  </link>

  <link name="link2">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.08 0.08 0.4"/>
      </geometry>
      <material name="link_material">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.08 0.08 0.4"/>
      </geometry>
    </collision>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="50" velocity="1.0"/>
  </joint>

  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="1.0"/>
  </joint>
</robot>
```

**Spawning Script (spawn_arm.py):**
```python
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose

class ArmSpawner(Node):
    def __init__(self):
        super().__init__('arm_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            pass
    
    def spawn(self):
        with open('arm.urdf', 'r') as f:
            urdf = f.read()
        
        req = SpawnEntity.Request()
        req.name = 'simple_arm'
        req.xml = urdf
        req.robot_namespace = ''
        req.initial_pose = Pose()
        req.initial_pose.position.z = 0.5
        
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Arm spawned!')

def main():
    rclpy.init()
    spawner = ArmSpawner()
    spawner.spawn()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected Output:**
- Arm appears in Gazebo at position (0, 0, 0.5)
- Base link (gray cylinder) at origin
- Link 1 (blue box) extends upward
- Link 2 (smaller blue box) attached to Link 1

## Lab 2.2: Simulate Sensors and Subscribe

**Objective**: Add a depth camera and LiDAR to the arm, then subscribe to their data

**Enhanced URDF with Sensors:**
```xml
<!-- Add to simple_arm URDF -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </collision>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="link2"/>
  <child link="camera_link"/>
  <origin xyz="0 0 0.25" rpy="0 0 0"/>
</joint>

<!-- In SDF, add sensor to camera_link -->
<sensor name="depth_camera" type="depth_camera">
  <topic>camera/depth</topic>
  <update_rate>30</update_rate>
  <!-- ... sensor config ... -->
</sensor>
```

**Sensor Subscriber (sensor_subscriber.py):**
```python
import rclpy
from sensor_msgs.msg import Image, LaserScan
import numpy as np

class SensorSubscriber(rclpy.Node):
    def __init__(self):
        super().__init__('sensor_subscriber')
        
        # Subscribe to sensors
        self.create_subscription(Image, 'camera/depth', 
                                self.depth_callback, 10)
        self.create_subscription(LaserScan, 'scan', 
                                self.scan_callback, 10)
    
    def depth_callback(self, msg):
        self.get_logger().info(f'Depth image: {msg.width}x{msg.height}')
    
    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        if len(ranges) > 0:
            min_dist = np.nanmin(ranges)
            self.get_logger().info(f'LiDAR min distance: {min_dist:.2f}m')

def main():
    rclpy.init()
    node = SensorSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Running Both:**
```bash
# Terminal 1: Start Gazebo with arm
gazebo worlds/arm_world.world

# Terminal 2: Spawn arm
ros2 run your_package spawn_arm.py

# Terminal 3: Subscribe to sensors
ros2 run your_package sensor_subscriber.py
```

**Expected Output:**
```
[sensor_subscriber]: Depth image: 640x480
[sensor_subscriber]: LiDAR min distance: 2.34m
```

## Exercise 1: Interactive Arm Control

**Task**: Create a node that:
1. Subscribes to `/cmd_vel` commands
2. Converts linear velocity to joint angles
3. Publishes joint commands

**Hints**:
- Use inverse kinematics or simple mapping
- Limit joint velocities to safe ranges
- Add safety checks for joint limits

## Exercise 2: Data Recording Challenge

**Task**: Record sensor data while moving the arm

**Steps**:
```bash
# Record data
ros2 bag record -a

# Move arm manually in Gazebo for 30 seconds

# Analyze recorded data
ros2 bag play rosbag2_*/
```

**Rubric**:
- ✓ Sensor data recorded (image + scan)
- ✓ Rosbag file created and playable
- ✓ Timestamp alignment verified

## Key Takeaways

✓ Gazebo simulates realistic physics and sensors
✓ URDF models can be converted to SDF with full sensor support
✓ ROS 2 nodes integrate seamlessly with Gazebo
✓ Sensor data (depth, LiDAR, IMU) available for algorithm testing
✓ Rosbag records reproducible test data
