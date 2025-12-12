# Chapter 4: ROS 2 Integration and Control

## Spawning Models in Simulation

### Using spawn_entity Service

```python
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import xacro

class ModelSpawner(Node):
    def __init__(self):
        super().__init__('model_spawner')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
    
    def spawn_model(self, name, urdf_path):
        # Read URDF
        with open(urdf_path, 'r') as f:
            urdf_content = f.read()
        
        # Parse URDF with xacro if needed
        if urdf_path.endswith('.urdf.xacro'):
            urdf_content = xacro.process(urdf_path)
        
        # Create request
        request = SpawnEntity.Request()
        request.name = name
        request.xml = urdf_content
        request.robot_namespace = ''
        request.initial_pose = Pose()
        request.initial_pose.position.x = 0.0
        request.initial_pose.position.y = 0.0
        request.initial_pose.position.z = 1.0
        
        # Send request
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Model {name} spawned successfully')
        else:
            self.get_logger().error(f'Failed to spawn {name}')

def main():
    rclpy.init()
    spawner = ModelSpawner()
    spawner.spawn_model('my_robot', 'urdf/my_robot.urdf')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Robot Control via Twist Messages

### Publishing Velocity Commands

```python
from geometry_msgs.msg import Twist

class RobotController(rclpy.Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
    
    def move_forward(self, speed=0.5, duration=5.0):
        """Move robot forward"""
        twist = Twist()
        twist.linear.x = speed
        
        end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=duration)
        
        while self.get_clock().now() < end_time:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        # Stop
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)
    
    def rotate(self, angular_speed=0.5, angle=1.57):
        """Rotate robot (radians)"""
        twist = Twist()
        twist.angular.z = angular_speed
        
        duration = abs(angle) / angular_speed
        end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=duration)
        
        while self.get_clock().now() < end_time:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
    
    def control_loop(self):
        pass
```

### Odometry Feedback

```python
from nav_msgs.msg import Odometry
import numpy as np

class OdometryTracker(rclpy.Node):
    def __init__(self):
        super().__init__('odometry_tracker')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.position = np.array([0.0, 0.0])
        self.orientation = 0.0
    
    def odom_callback(self, msg):
        # Extract position
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        
        # Extract orientation (quaternion to euler)
        from tf_transformations import euler_from_quaternion
        q = msg.pose.pose.orientation
        _, _, self.orientation = euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
        
        self.get_logger().info(
            f"Position: ({self.position[0]:.2f}, {self.position[1]:.2f}), "
            f"Orientation: {np.degrees(self.orientation):.1f}°"
        )
```

## Launch Files for Gazebo

**gazebo.launch.py:**
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', 'worlds/empty.world'],
        output='screen'
    )
    
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open('urdf/robot.urdf').read()}]
    )
    
    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
    ])
```

**Launch with:**
```bash
ros2 launch gazebo.launch.py
```

## Debugging with rviz2

```bash
# Start rviz2
ros2 run rviz2 rviz2

# Publish TF tree
ros2 run tf2_tools view_frames.py
ros2 tool tf tree

# View messages
ros2 topic echo /odom
ros2 topic info /cmd_vel
```

## Performance Tips

1. **Reduce Update Rates**: Lower sensor update rates for faster simulation
2. **Simplify Geometry**: Use primitive shapes instead of complex meshes
3. **Disable Visualization**: Run headless with gzserver only
4. **Adjust Physics**: Increase max_step_size for faster (less accurate) simulation
5. **Limit Contacts**: Use collision filtering to reduce collision checks
