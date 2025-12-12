# Chapter 3: Isaac ROS 2 Integration

## ROS 2 Topic Publishing

**Automatic ROS 2 Publishing:**
```python
from isaacsim import SimulationApp
from omni.isaac.core import World
from omni.isaac.ros2_bridge import ROS2Bridge

# Enable ROS 2 bridge
ros2_bridge = ROS2Bridge(namespace="")

simulation_app = SimulationApp({"headless": False})
world = World()

# All sensors automatically publish to ROS 2 topics:
# /camera/rgb/image_raw
# /camera/depth/image_raw
# /lidar/scan
# /imu/data
# /odometry/local_xy_plane/odometry

world.play()

for i in range(10000):
    world.step(render=True)

simulation_app.close()
```

## Custom ROS 2 Publishers

**Publish Custom Data:**
```python
import rclpy
from geometry_msgs.msg import Twist
from omni.isaac.core.robots import Robot

class IsaacROS2Controller(rclpy.Node):
    def __init__(self, world):
        super().__init__('isaac_controller')
        
        self.world = world
        self.robot = Robot(prim_path="/World/UR10", name="ur10")
        
        # Create ROS 2 publisher for joint commands
        self.joint_pub = self.create_publisher(
            Twist,
            '/joint_commands',
            10
        )
        
        # Create ROS 2 subscriber for velocity goals
        self.vel_sub = self.create_subscription(
            Twist,
            '/vel_goal',
            self.velocity_callback,
            10
        )
        
        self.timer = self.create_timer(0.02, self.control_loop)
    
    def velocity_callback(self, msg):
        """Handle incoming velocity commands"""
        # Convert Twist to robot actions
        self.target_vel = [msg.linear.x, msg.linear.y, msg.angular.z]
    
    def control_loop(self):
        """Main control loop"""
        # Get robot state
        state = self.robot.get_state()
        
        # Publish current joint angles
        joint_angles = self.robot.get_joint_positions()
        self.joint_pub.publish(self.format_message(joint_angles))
```

## Isaac ROS Nodes

### DetectNetNode - Object Detection

```python
from isaac_ros_detectnet.detectnet_node import DetectNetNode
from isaac_ros_common.isaac_ros_common import IsaacROSCommon

class ObjectDetector(IsaacROSCommon):
    def __init__(self):
        super().__init__(node_namespace='detectnet')
        
        self.detectnet_node = DetectNetNode(
            context=self.context,
            model_name='peoplesemsegnet_shufflenet',
            engine_file_path='/path/to/model.etlt',
            input_binding_names=['input'],
            output_binding_names=['output_0']
        )
    
    def run(self):
        # Subscribes to /image
        # Publishes detection results to /detections
        while True:
            self.context.graph.run(self.detectnet_node)

# Detection output format
# Type: DetectionList2D
# Fields:
#   - detections (array of Detection2D)
#     - bbox (x, y, width, height)
#     - label (class name)
#     - confidence (0-1)
```

### PoseEstimationNode - 3D Pose Estimation

```python
from isaac_ros_pose_estimation.pose_estimation_node import PoseEstimationNode

class PoseEstimator(IsaacROSCommon):
    def __init__(self):
        super().__init__(node_namespace='pose_est')
        
        self.pose_node = PoseEstimationNode(
            model_name='pose_estimation_model',
            engine_file_path='/path/to/pose_model.etlt'
        )
    
    def run(self):
        # Subscribes to /image + /detections
        # Publishes 3D pose to /pose_estimation/poses
        while True:
            self.context.graph.run(self.pose_node)

# Pose output format
# Type: PoseFrameMessage
# Fields:
#   - pose (x, y, z, qx, qy, qz, qw)
#   - timestamp
#   - object_id
```

## Integration Example

**Complete Pipeline:**
```python
#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from vision_msgs.msg import DetectionArray
from isaacsim import SimulationApp
from omni.isaac.core import World

class FullPipeline(rclpy.Node):
    def __init__(self):
        super().__init__('isaac_full_pipeline')
        
        # Initialize Isaac
        self.simulation_app = SimulationApp({"headless": False})
        self.world = World()
        
        # Load scene
        self.setup_scene()
        
        # ROS 2 subscriptions
        self.create_subscription(DetectionArray, '/detections', 
                                self.detection_callback, 10)
        self.create_subscription(TransformStamped, '/object_pose',
                                self.pose_callback, 10)
        
        # ROS 2 publisher
        self.cmd_pub = self.create_publisher(Twist, '/ur10/cmd_vel', 10)
        
        # Main loop
        self.timer = self.create_timer(0.02, self.step)
    
    def setup_scene(self):
        """Load UR10 and environment"""
        from omni.isaac.core.utils.stage import add_reference_to_stage
        
        add_reference_to_stage(
            usd_path="omniverse://localhost/NVIDIA/Assets/Robots/UR/ur10/ur10.usd",
            prim_path="/World/UR10"
        )
        
        self.world.reset()
    
    def detection_callback(self, msg):
        """Process detections from Isaac ROS DetectNetNode"""
        for detection in msg.detections:
            x, y, width, height = detection.bbox.center.position.x, \
                                 detection.bbox.center.position.y, \
                                 detection.bbox.size_x, \
                                 detection.bbox.size_y
            confidence = detection.results[0].score
            
            self.get_logger().info(
                f"Detected: {detection.results[0].id} "
                f"at ({x:.2f}, {y:.2f}) conf={confidence:.2f}"
            )
    
    def pose_callback(self, msg):
        """Process 3D pose from PoseEstimationNode"""
        x, y, z = msg.transform.translation.x, \
                 msg.transform.translation.y, \
                 msg.transform.translation.z
        
        self.target_pose = [x, y, z]
        self.get_logger().info(f"Target pose: ({x:.2f}, {y:.2f}, {z:.2f})")
    
    def step(self):
        """Step simulation"""
        self.world.step(render=True)

def main():
    rclpy.init()
    pipeline = FullPipeline()
    rclpy.spin(pipeline)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topic Mapping

**Default Isaac Sim ROS 2 Topics:**

| Sensor | Topic | Message Type |
|--------|-------|--------------|
| RGB Camera | `/camera/rgb/image_raw` | `sensor_msgs/Image` |
| Depth Camera | `/camera/depth/image_raw` | `sensor_msgs/Image` |
| LiDAR | `/lidar/scan` | `sensor_msgs/LaserScan` |
| IMU | `/imu/data` | `sensor_msgs/Imu` |
| Odometry | `/odometry` | `nav_msgs/Odometry` |
| TF | `/tf` | `tf2_msgs/TFMessage` |

**Custom Topic Remapping:**
```python
ros2_context = {"namespace": "/isaac", 
                "remappings": [("/image_raw", "/camera/rgb"),
                               ("/depth", "/camera/depth_registered")]}
```
