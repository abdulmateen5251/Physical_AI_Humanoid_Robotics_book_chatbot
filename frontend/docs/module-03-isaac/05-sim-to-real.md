# Chapter 5: Sim-to-Real Transfer

## The Sim-to-Real Gap

**Common Challenges:**
- Physics differences (friction, damping, stiffness)
- Sensor noise and calibration differences
- Latency variations
- Model uncertainties

**Solutions:**
- Domain randomization
- System identification
- Model calibration
- Conservative control policies

## Domain Randomization Strategy

**Randomize All Variability:**

```python
class DomainRandomizer:
    def __init__(self):
        self.randomization_config = {
            # Physics
            'friction_coefficient': (0.3, 0.8),
            'mass_multiplier': (0.9, 1.1),
            'damping': (0.0, 0.5),
            
            # Sensors
            'camera_noise_std': (0.001, 0.01),
            'depth_noise_std': (0.01, 0.05),
            'lidar_noise_std': (0.02, 0.1),
            
            # Lighting
            'light_intensity': (500, 3000),
            'ambient_brightness': (0.3, 1.0),
            
            # Textures
            'object_texture': ['metal', 'plastic', 'rubber', 'wood'],
            
            # Scene
            'num_distractor_objects': (0, 5),
            'background_clutter': (0, 1),
        }
    
    def randomize_physics(self, world):
        """Randomize physical properties"""
        import random
        
        friction = random.uniform(
            self.randomization_config['friction_coefficient'][0],
            self.randomization_config['friction_coefficient'][1]
        )
        
        mass_mult = random.uniform(
            self.randomization_config['mass_multiplier'][0],
            self.randomization_config['mass_multiplier'][1]
        )
        
        damping = random.uniform(
            self.randomization_config['damping'][0],
            self.randomization_config['damping'][1]
        )
        
        # Apply to all objects
        for obj in world.scene.get_objects():
            obj.set_friction(friction)
            obj.set_mass(obj.mass * mass_mult)
            # Set damping via physics handle
    
    def randomize_sensors(self):
        """Add noise to sensor readings"""
        import random
        import numpy as np
        
        noise_config = {
            'camera': random.gauss(0, self.randomization_config['camera_noise_std'][1]),
            'depth': random.gauss(0, self.randomization_config['depth_noise_std'][1]),
            'lidar': random.gauss(0, self.randomization_config['lidar_noise_std'][1]),
        }
        
        return noise_config
    
    def randomize_lighting(self, scene):
        """Randomize scene illumination"""
        import random
        
        light_intensity = random.uniform(
            self.randomization_config['light_intensity'][0],
            self.randomization_config['light_intensity'][1]
        )
        
        brightness = random.uniform(
            self.randomization_config['ambient_brightness'][0],
            self.randomization_config['ambient_brightness'][1]
        )
        
        # Update scene lights
        lights = scene.get_lights()
        for light in lights:
            light.set_intensity(light_intensity)
            light.set_brightness(brightness)
```

## System Identification and Calibration

**Calibrate Robot Model:**

```python
class RobotCalibrator:
    def __init__(self, robot):
        self.robot = robot
        self.calibration_data = {}
    
    def calibrate_friction(self):
        """Measure friction coefficient from motion"""
        from scipy.optimize import minimize
        
        # Apply known force, measure deceleration
        test_results = []
        
        for test_velocity in [0.5, 1.0, 1.5, 2.0]:
            # Apply velocity
            self.robot.set_velocity(test_velocity)
            
            # Measure deceleration
            velocities = []
            times = []
            
            for t in range(100):  # 5 seconds at 20Hz
                vel = self.robot.get_velocity()
                velocities.append(vel)
                times.append(t * 0.05)
            
            # Fit exponential decay: v(t) = v0 * exp(-friction * t)
            from numpy.polynomial import polynomial as P
            
            test_results.append({
                'initial_velocity': test_velocity,
                'velocities': velocities,
                'times': times
            })
        
        # Estimate friction coefficient
        def error_fn(friction_coef):
            total_error = 0
            for result in test_results:
                predicted = [
                    result['initial_velocity'] * np.exp(-friction_coef * t)
                    for t in result['times']
                ]
                actual = result['velocities']
                total_error += np.sum((np.array(predicted) - np.array(actual)) ** 2)
            return total_error
        
        result = minimize(error_fn, x0=0.1, bounds=[(0, 1)])
        self.calibration_data['friction'] = result.x[0]
        
        print(f"Estimated friction coefficient: {result.x[0]:.4f}")
    
    def calibrate_dynamics(self):
        """Identify robot inertia and damping"""
        import numpy as np
        
        # Apply step input, measure response
        step_force = 10.0  # Newtons
        self.robot.apply_force(step_force)
        
        accelerations = []
        velocities = []
        
        for t in range(500):  # 25 seconds
            accel = self.robot.get_acceleration()
            vel = self.robot.get_velocity()
            
            accelerations.append(accel)
            velocities.append(vel)
        
        # Fit second-order model: m*a + c*v = F
        # [m, c] from least squares
        
        A_matrix = np.column_stack([accelerations[:-1], velocities[:-1]])
        b_vector = np.ones(len(accelerations) - 1) * step_force
        
        params, residuals, rank, s = np.linalg.lstsq(A_matrix, b_vector)
        
        self.calibration_data['inertia'] = params[0]
        self.calibration_data['damping'] = params[1]
        
        print(f"Estimated inertia: {params[0]:.4f}, Damping: {params[1]:.4f}")

# Usage in training loop
calibrator = RobotCalibrator(robot)
calibrator.calibrate_friction()
calibrator.calibrate_dynamics()

# Use calibration in sim for better accuracy
```

## Conservative Control Policies

**Safety-First Design:**

```python
class ConservativeController:
    def __init__(self, max_velocity=0.5, max_acceleration=0.5):
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.previous_command = 0.0
    
    def compute_safe_command(self, desired_velocity):
        """Apply conservative constraints"""
        import numpy as np
        
        # Limit velocity
        constrained_vel = np.clip(
            desired_velocity,
            -self.max_velocity,
            self.max_velocity
        )
        
        # Limit acceleration
        max_step = self.max_acceleration * 0.05  # 50ms timestep
        constrained_vel = np.clip(
            constrained_vel,
            self.previous_command - max_step,
            self.previous_command + max_step
        )
        
        self.previous_command = constrained_vel
        
        return constrained_vel
```

## Transfer to Real Robot

**Workflow:**

```
1. Train in Simulation
   ↓
2. Evaluate with Domain Randomization
   ↓
3. Calibrate Real Robot Parameters
   ↓
4. Fine-tune Policy (if needed)
   ↓
5. Deploy on Real Hardware
   ↓
6. Monitor & Adapt
```

**Deployment Script:**

```python
import rclpy
from geometry_msgs.msg import Twist

class RealRobotDeployer:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('robot_deployer')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Load trained policy
        import torch
        self.policy = torch.jit.load('policy.pt')
        self.policy.eval()
    
    def run(self):
        """Main deployment loop"""
        rate = self.node.create_rate(20)  # 20Hz
        
        while rclpy.ok():
            # Get sensor readings
            observations = self.get_observations()
            
            # Inference
            with torch.no_grad():
                action = self.policy(observations)
            
            # Apply safety constraints
            safe_action = self.apply_safety(action)
            
            # Publish command
            twist = Twist()
            twist.linear.x = safe_action[0]
            twist.angular.z = safe_action[1]
            self.cmd_pub.publish(twist)
            
            rate.sleep()
    
    def apply_safety(self, action):
        """Apply conservative constraints"""
        import numpy as np
        
        max_vel = 0.5
        max_angular = 1.0
        
        safe_action = np.array([
            np.clip(action[0], -max_vel, max_vel),
            np.clip(action[1], -max_angular, max_angular)
        ])
        
        return safe_action
```

## Validation Metrics

**Success Criteria for Transfer:**

| Metric | Simulation | Real Robot | Target |
|--------|-----------|-----------|--------|
| Task Success Rate | 95 percent or higher | 80 percent or higher | 85 percent or higher |
| Mean Time to Task | 5-10 seconds | 6-12 seconds | ±20% |
| Collision Rate | 1 percent or less | 2 percent or less | 2 percent or less |
| Energy Efficiency | Baseline | ±15% | ±20% |

**Key Learnings:**
✓ Domain randomization essential for robustness
✓ Calibration reduces reality gap significantly
✓ Conservative policies ensure safety during transfer
✓ Gradual deployment reduces risk
