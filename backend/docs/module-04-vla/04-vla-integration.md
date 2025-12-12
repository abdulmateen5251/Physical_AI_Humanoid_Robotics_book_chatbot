# Chapter 4: Complete VLA Integration and Execution

## Full VLA Pipeline Architecture

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np

class VLAController(Node):
    """Complete Vision-Language-Action controller"""
    
    def __init__(self):
        super().__init__('vla_controller')
        
        # Initialize components
        self.recognizer = WhisperRecognizer()
        self.planner = RobotPlanner(api_key='your-key')
        self.validator = SafetyValidator()
        self.vision_model = VisionLanguageModel(api_key='your-key')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/vla_status', 10)
        self.debug_pub = self.create_publisher(String, '/vla_debug', 10)
        
        # Subscribers
        self.create_subscription(String, '/voice_commands', 
                                self.command_callback, 10)
        self.create_subscription(Image, '/camera/rgb/image_raw',
                                self.vision_callback, 10)
        
        # State
        self.robot_state = {}
        self.current_action_idx = 0
        self.current_plan = []
        self.last_image = None
        
        self.get_logger().info("VLA Controller initialized")
    
    def command_callback(self, msg: String):
        """Process incoming voice command"""
        command = msg.data
        self.get_logger().info(f"Processing command: {command}")
        
        # Step 1: Get visual context
        if self.last_image is not None:
            visual_context = self.vision_model.understand_scene(
                self.last_image,
                query=f"What is relevant to this task: {command}?"
            )
            self.get_logger().info(f"Visual context: {visual_context}")
        
        # Step 2: Generate plan
        plan = self.planner.plan(command)
        self.current_plan = plan
        
        # Step 3: Validate plan
        is_valid, violations = self.validator.validate_plan(
            plan, 
            self.robot_state
        )
        
        if not is_valid:
            self.get_logger().warn(f"Constraint violations: {violations}")
            plan = self.validator.recover_plan(plan, violations)
        
        # Step 4: Execute plan
        self.execute_plan(plan)
    
    def vision_callback(self, msg: Image):
        """Store current image for vision-language reasoning"""
        from cv_bridge import CvBridge
        bridge = CvBridge()
        self.last_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    
    def execute_plan(self, plan: List[Dict]):
        """Execute action sequence"""
        for action_idx, action in enumerate(plan):
            self.current_action_idx = action_idx
            
            # Publish status
            status = String()
            status.data = f"Executing action {action_idx + 1}/{len(plan)}: {action}"
            self.status_pub.publish(status)
            
            self.get_logger().info(f"Executing: {action}")
            
            try:
                self.execute_action(action)
            except Exception as e:
                self.get_logger().error(f"Error executing action: {e}")
                break
    
    def execute_action(self, action: Dict):
        """Execute single action"""
        action_type = action.get('action')
        
        if action_type == 'move_forward':
            self.move_forward(action.get('distance', 1.0))
        
        elif action_type == 'turn_left':
            self.turn(angle=1.57)  # 90 degrees
        
        elif action_type == 'turn_right':
            self.turn(angle=-1.57)
        
        elif action_type == 'grasp':
            self.grasp(action.get('force', 10))
        
        elif action_type == 'release':
            self.release()
    
    def move_forward(self, distance: float):
        """Move robot forward"""
        twist = Twist()
        twist.linear.x = 0.5  # m/s
        
        # Duration based on distance
        duration = distance / 0.5  # seconds
        
        end_time = self.get_clock().now() + \
                   rclpy.duration.Duration(seconds=duration)
        
        while self.get_clock().now() < end_time:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        # Stop
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Moved forward: {distance}m")
    
    def turn(self, angle: float):
        """Rotate robot (radians)"""
        twist = Twist()
        twist.angular.z = 0.5 if angle > 0 else -0.5
        
        duration = abs(angle) / 0.5
        end_time = self.get_clock().now() + \
                   rclpy.duration.Duration(seconds=duration)
        
        while self.get_clock().now() < end_time:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Rotated: {np.degrees(angle):.1f}°")
    
    def grasp(self, force: float):
        """Grasp object with end effector"""
        self.get_logger().info(f"Grasping with force: {force}N")
        # Send grasp command to manipulator
        # (implementation depends on robot gripper API)
    
    def release(self):
        """Release object"""
        self.get_logger().info("Releasing object")
        # Send release command to manipulator

def main():
    rclpy.init()
    controller = VLAController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## State Management

**Track robot state throughout execution:**

```python
class RobotState:
    """Maintains robot state during execution"""
    
    def __init__(self):
        self.position = [0.0, 0.0]
        self.orientation = 0.0
        self.joint_angles = {}
        self.gripper_state = "open"  # open, closed, grasping
        self.battery_level = 1.0
        self.last_update = None
    
    def update_from_sensors(self, odometry, joint_states, gripper_status):
        """Update state from sensor readings"""
        
        self.position[0] = odometry.pose.pose.position.x
        self.position[1] = odometry.pose.pose.position.y
        
        # Extract orientation
        from tf_transformations import euler_from_quaternion
        q = odometry.pose.pose.orientation
        _, _, self.orientation = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # Update joint angles
        for joint, angle in joint_states.items():
            self.joint_angles[joint] = angle
        
        # Update gripper
        self.gripper_state = gripper_status
    
    def to_dict(self):
        """Convert to dictionary for logging"""
        return {
            'position': self.position,
            'orientation': np.degrees(self.orientation),
            'joints': self.joint_angles,
            'gripper': self.gripper_state,
            'battery': self.battery_level
        }
```

## Error Recovery

**Handle failures gracefully:**

```python
class ErrorRecoveryManager:
    def __init__(self, controller: VLAController):
        self.controller = controller
        self.recovery_strategies = {
            'obstacle_detected': self.recover_obstacle,
            'gripper_failed': self.recover_gripper,
            'timeout': self.recover_timeout,
            'battery_low': self.recover_battery_low
        }
    
    def handle_error(self, error_type: str):
        """Route error to appropriate recovery strategy"""
        strategy = self.recovery_strategies.get(error_type)
        if strategy:
            strategy()
        else:
            self.recovery_fallback()
    
    def recover_obstacle(self):
        """Recover from obstacle detection"""
        self.controller.get_logger().warn("Obstacle detected, backing up")
        
        # Reverse direction
        twist = Twist()
        twist.linear.x = -0.2  # Back up
        self.controller.cmd_vel_pub.publish(twist)
        
        # Wait
        import time
        time.sleep(1)
        
        # Try alternate path
        self.controller.turn(angle=1.57)
    
    def recover_gripper(self):
        """Recover from gripper failure"""
        self.controller.get_logger().warn("Gripper failed, releasing")
        self.controller.release()
    
    def recover_timeout(self):
        """Recover from action timeout"""
        self.controller.get_logger().warn("Action timeout, emergency stop")
        twist = Twist()
        self.controller.cmd_vel_pub.publish(twist)
    
    def recover_battery_low(self):
        """Recover from low battery"""
        self.controller.get_logger().warn("Battery low, returning to dock")
        # Navigate to charging dock
    
    def recovery_fallback(self):
        """Default recovery: stop all motion"""
        twist = Twist()
        self.controller.cmd_vel_pub.publish(twist)
```

## Debugging and Logging

**Comprehensive logging for debugging:**

```python
import logging
from datetime import datetime

class VLALogger:
    def __init__(self, log_file='vla_execution.log'):
        self.logger = logging.getLogger('VLA')
        self.logger.setLevel(logging.DEBUG)
        
        # File handler
        fh = logging.FileHandler(log_file)
        fh.setLevel(logging.DEBUG)
        
        # Console handler
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        
        # Formatter
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        fh.setFormatter(formatter)
        ch.setFormatter(formatter)
        
        self.logger.addHandler(fh)
        self.logger.addHandler(ch)
    
    def log_command(self, command: str):
        self.logger.info(f"Command received: {command}")
    
    def log_plan(self, plan: List[Dict]):
        self.logger.debug(f"Generated plan:\n{json.dumps(plan, indent=2)}")
    
    def log_action(self, action_idx: int, action: Dict, status: str):
        self.logger.info(f"Action {action_idx}: {action['action']} - {status}")
    
    def log_state(self, state: Dict):
        self.logger.debug(f"Robot state: {json.dumps(state, indent=2)}")
```

## Performance Monitoring

**Track execution metrics:**

```python
import time

class PerformanceMonitor:
    def __init__(self):
        self.metrics = {
            'planning_time': [],
            'validation_time': [],
            'execution_time': [],
            'total_time': None
        }
    
    def record_planning_time(self, duration: float):
        self.metrics['planning_time'].append(duration)
    
    def record_validation_time(self, duration: float):
        self.metrics['validation_time'].append(duration)
    
    def record_execution_time(self, duration: float):
        self.metrics['execution_time'].append(duration)
    
    def report(self):
        """Generate performance report"""
        return {
            'avg_planning_ms': np.mean(self.metrics['planning_time']) * 1000,
            'avg_validation_ms': np.mean(self.metrics['validation_time']) * 1000,
            'avg_execution_ms': np.mean(self.metrics['execution_time']) * 1000,
            'total_actions': len(self.metrics['execution_time'])
        }
```

## Key Takeaways

✓ VLA integrates voice, vision, language, and action
✓ Multi-modal understanding improves task success
✓ Safety validation is critical for real robots
✓ Error recovery enables robust operation
✓ Comprehensive logging aids debugging
