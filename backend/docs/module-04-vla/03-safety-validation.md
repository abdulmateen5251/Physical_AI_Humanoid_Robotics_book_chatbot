# Chapter 3: Safety Validation and Constraint Checking

## Constraint Validator Architecture

Ensure that generated plans satisfy safety constraints:

```python
from typing import List, Dict, Tuple

class SafetyValidator:
    def __init__(self):
        self.constraints = {
            'max_speed': 2.0,  # m/s
            'max_acceleration': 1.0,  # m/s^2
            'max_angular_velocity': 1.57,  # rad/s
            'min_distance_to_obstacle': 0.3,  # meters
            'max_joint_angles': {
                'shoulder': 1.57,
                'elbow': 2.0,
                'wrist': 1.57
            },
            'max_grasp_force': 50,  # Newtons
            'min_grasp_force': 5,  # Newtons
        }
    
    def validate_plan(self, plan: List[Dict], state: Dict) -> Tuple[bool, List[str]]:
        """
        Validate plan against constraints
        
        Returns: (is_valid: bool, violations: List[str])
        """
        violations = []
        
        for action_idx, action in enumerate(plan):
            action_type = action.get('action')
            
            # Validate movement actions
            if action_type == 'move_forward':
                distance = action.get('distance', 0)
                if distance > 10.0:
                    violations.append(
                        f"Action {action_idx}: Movement too large: "
                        f"{distance}m > 10m limit"
                    )
                
                # Check for obstacles
                if state.get('lidar_min_distance', 1.0) < \
                   self.constraints['min_distance_to_obstacle']:
                    violations.append(
                        f"Action {action_idx}: Obstacle too close, "
                        f"cannot move forward"
                    )
            
            # Validate grasp actions
            elif action_type == 'grasp':
                force = action.get('force', 0)
                max_force = self.constraints['max_grasp_force']
                min_force = self.constraints['min_grasp_force']
                
                if force > max_force:
                    violations.append(
                        f"Action {action_idx}: Grasp force too high: "
                        f"{force} > {max_force} limit"
                    )
                elif force < min_force:
                    violations.append(
                        f"Action {action_idx}: Grasp force too low: "
                        f"{force} < {min_force} minimum"
                    )
            
            # Validate joint movements
            elif action_type == 'move_arm':
                angles = {
                    'shoulder': action.get('shoulder', 0),
                    'elbow': action.get('elbow', 0),
                    'wrist': action.get('wrist', 0)
                }
                
                for joint, angle in angles.items():
                    limit = self.constraints['max_joint_angles'].get(joint)
                    if limit and abs(angle) > limit:
                        violations.append(
                            f"Action {action_idx}: Joint {joint} angle out of limits: "
                            f"{angle} rad > {limit} rad"
                        )
            
            # Validate rotation
            elif action_type == 'turn_right' or action_type == 'turn_left':
                # Extract angle from action
                angle = action.get('angle', 1.57)  # Default 90 degrees
                
                if abs(angle) > 3.14:  # 180 degrees
                    violations.append(
                        f"Action {action_idx}: Rotation angle too large: {angle} rad"
                    )
        
        is_valid = len(violations) == 0
        return is_valid, violations
    
    def recover_plan(self, plan: List[Dict], violations: List[str]) -> List[Dict]:
        """Attempt to fix constraint violations"""
        fixed_plan = []
        
        for action in plan:
            fixed_action = action.copy()
            
            # Cap movement distances
            if action.get('action') == 'move_forward':
                fixed_action['distance'] = min(action.get('distance', 0), 2.0)
            
            # Cap grasp force
            elif action.get('action') == 'grasp':
                fixed_action['force'] = min(action.get('force', 0), 50)
                fixed_action['force'] = max(fixed_action['force'], 5)
            
            # Cap joint angles
            elif action.get('action') == 'move_arm':
                for joint in ['shoulder', 'elbow', 'wrist']:
                    if joint in fixed_action:
                        limit = self.constraints['max_joint_angles'].get(joint)
                        fixed_action[joint] = min(
                            max(fixed_action[joint], -limit),
                            limit
                        )
            
            fixed_plan.append(fixed_action)
        
        return fixed_plan
    
    def explain_violations(self, violations: List[str]) -> str:
        """Generate human-readable explanation"""
        explanation = "Safety violations detected:\n\n"
        for violation in violations:
            explanation += f"• {violation}\n"
        return explanation
```

## Real-Time Monitoring

**Monitor execution and abort if unsafe:**

```python
class ExecutionMonitor:
    def __init__(self, validator: SafetyValidator):
        self.validator = validator
        self.current_state = {}
        self.should_abort = False
    
    def update_state(self, sensor_data: Dict):
        """Update current robot state from sensors"""
        self.current_state = {
            'position': sensor_data.get('odometry'),
            'velocity': sensor_data.get('velocity'),
            'lidar_min_distance': sensor_data.get('lidar_min_distance'),
            'joint_angles': sensor_data.get('joint_angles'),
            'battery_level': sensor_data.get('battery_level')
        }
    
    def check_safety(self, next_action: Dict) -> bool:
        """Check if next action is safe given current state"""
        
        # Check distance to obstacles
        if self.current_state['lidar_min_distance'] < 0.2:
            self.should_abort = True
            return False
        
        # Check battery
        if self.current_state['battery_level'] < 0.1:  # 10% threshold
            self.should_abort = True
            return False
        
        # Check joint limits
        for joint, angle in self.current_state['joint_angles'].items():
            if abs(angle) > 1.5:  # Near limit
                if next_action.get('action') == 'move_arm':
                    return False
        
        return True
    
    def emergency_stop(self):
        """Trigger emergency stop"""
        self.should_abort = True
        self.stop_all_motors()
    
    def stop_all_motors(self):
        """Stop all robot motors"""
        import rclpy
        from geometry_msgs.msg import Twist
        
        # Publish zero velocity
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # ... publish ...
```

## Constraint Specification in DSL

**Domain-Specific Language for constraints:**

```python
class ConstraintDSL:
    """Define constraints in natural language"""
    
    constraints_str = """
    # Robot Safety Constraints
    
    max_speed: 2.0 m/s
    max_acceleration: 1.0 m/s^2
    
    joint_limits:
      shoulder: [-90, 90] degrees
      elbow: [-120, 120] degrees
      wrist: [-180, 180] degrees
    
    grasp_force:
      min: 5 N
      max: 50 N
    
    collision_avoidance:
      min_distance: 0.3 m
      check_frequency: 20 Hz
    
    emergency_stop:
      trigger_distance: 0.1 m
      trigger_tilt_angle: 45 degrees
    """
    
    def parse_constraints(self):
        """Parse DSL into constraint dictionary"""
        import yaml
        
        # Remove comments
        clean_str = '\n'.join([
            line for line in self.constraints_str.split('\n')
            if not line.strip().startswith('#')
        ])
        
        # Parse YAML
        constraints = yaml.safe_load(clean_str)
        return constraints
```

## Testing Safety

**Unit tests for constraints:**

```python
import unittest

class TestSafetyValidator(unittest.TestCase):
    def setUp(self):
        self.validator = SafetyValidator()
    
    def test_excessive_speed(self):
        plan = [{'action': 'move_forward', 'distance': 100.0}]
        is_valid, violations = self.validator.validate_plan(plan, {})
        self.assertFalse(is_valid)
        self.assertTrue(len(violations) > 0)
    
    def test_excessive_grasp_force(self):
        plan = [{'action': 'grasp', 'force': 100.0}]
        is_valid, violations = self.validator.validate_plan(plan, {})
        self.assertFalse(is_valid)
    
    def test_joint_limits(self):
        plan = [{'action': 'move_arm', 'shoulder': 5.0}]
        is_valid, violations = self.validator.validate_plan(plan, {})
        self.assertFalse(is_valid)
    
    def test_safe_plan(self):
        plan = [
            {'action': 'move_forward', 'distance': 1.0},
            {'action': 'grasp', 'force': 20.0}
        ]
        is_valid, violations = self.validator.validate_plan(plan, {})
        self.assertTrue(is_valid)
    
    def test_recovery(self):
        plan = [{'action': 'move_forward', 'distance': 100.0}]
        recovered = self.validator.recover_plan(plan, [])
        self.assertLessEqual(recovered[0]['distance'], 10.0)

if __name__ == '__main__':
    unittest.main()
```

## Key Principles

✓ **Fail-safe**: Default to stopping on any safety concern
✓ **Conservative**: Err on side of caution
✓ **Transparent**: Explain violations clearly
✓ **Recoverable**: Attempt to fix rather than reject
✓ **Testable**: Comprehensive safety tests
