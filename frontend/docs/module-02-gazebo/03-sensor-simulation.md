# Chapter 3: Sensor Simulation

## Types of Sensors

### Depth Camera
Simulates RGB-D cameras (Kinect, RealSense):

```xml
<sensor name="depth_camera" type="depth">
  <pose relative_to="camera_link">0 0 0 0 0 0</pose>
  <topic>camera/depth</topic>
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.01</near>
      <far>300</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
</sensor>
```

**ROS 2 Integration:**
```python
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthSubscriber(rclpy.Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera/depth',
            self.depth_callback,
            10
        )
        self.bridge = CvBridge()
    
    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        
        # Convert to meters
        depth_meters = depth_image / 1000.0
        
        # Get center pixel
        center_distance = depth_meters[240, 320]
        self.get_logger().info(f"Center distance: {center_distance:.2f}m")
```

### LiDAR (2D)
Simulates 2D laser scanner:

```xml
<sensor name="lidar" type="ray">
  <pose relative_to="lidar_link">0 0 0 0 0 0</pose>
  <topic>scan</topic>
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1.0</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.08</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.02</stddev>
    </noise>
  </ray>
</sensor>
```

**ROS 2 Integration:**
```python
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarSubscriber(rclpy.Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
    
    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Find closest obstacle
        valid_ranges = ranges[~np.isinf(ranges)]
        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            self.get_logger().info(f"Closest obstacle: {min_distance:.2f}m")
```

### IMU (Inertial Measurement Unit)
Simulates accelerometer, gyroscope, magnetometer:

```xml
<sensor name="imu" type="imu">
  <pose relative_to="imu_link">0 0 0 0 0 0</pose>
  <topic>imu</topic>
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.05</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.05</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.05</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.1</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.1</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.1</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

**ROS 2 Integration:**
```python
from sensor_msgs.msg import Imu

class IMUSubscriber(rclpy.Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10
        )
    
    def imu_callback(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        
        # Calculate tilt angle
        pitch = np.arctan2(ax, az)
        roll = np.arctan2(ay, az)
        
        self.get_logger().info(
            f"Pitch: {np.degrees(pitch):.1f}°, Roll: {np.degrees(roll):.1f}°"
        )
```

## Sensor Noise Models

### Gaussian Noise
Realistic sensor measurement noise:
```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.01</stddev>
</noise>
```

### Gaussian Quantized Noise
Combines Gaussian noise with quantization (like integer sensors):
```xml
<noise>
  <type>gaussian_quantized</type>
  <mean>0.0</mean>
  <stddev>0.01</stddev>
</noise>
```

## Recording Sensor Data with rosbag2

```bash
# Start recording all topics
ros2 bag record -a

# Start recording specific topics
ros2 bag record /camera/depth /scan /imu

# Play back recorded data
ros2 bag play rosbag2_2024_12_07/

# View rosbag info
ros2 bag info rosbag2_2024_12_07/
```

**Python integration:**
```python
from rosbag2_py import SequentialReader
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

class RosbagReader:
    def __init__(self, bag_path):
        self.reader = SequentialReader()
        self.reader.open(bag_path)
    
    def read_messages(self, topic_name):
        while self.reader.has_next():
            topic, timestamp, buffer = self.reader.read_next()
            if topic == topic_name:
                msg = deserialize_message(buffer, get_message(topic_name))
                yield msg
```
