# Module 3: The AI-Robot Brain (NVIDIA Isaac)

## Learning Objectives

By the end of this module, you will be able to:
- Understand the NVIDIA Isaac Sim ecosystem and USD (Universal Scene Description)
- Create synthetic datasets with realistic rendering and automatic labeling
- Train perception models on synthetic data and transfer them to real robots
- Integrate NVIDIA Isaac ROS components with ROS 2
- Implement Nav2-based path planning for autonomous navigation
- Optimize perception and control pipelines with hardware acceleration (CUDA)
- Build a complete sim-to-real transfer pipeline for humanoid robots

## Module Overview

Module 3 focuses on leveraging NVIDIA's AI technologies for robot intelligence. This module teaches:
- **Isaac Sim**: High-fidelity physics and rendering for simulation
- **Synthetic Data Generation**: Creating labeled datasets without manual annotation
- **Isaac ROS**: Optimized ROS 2 nodes for perception and planning
- **Perception ML**: Training and deploying vision models
- **Path Planning**: Nav2 stack for autonomous navigation

## Chapter 1: Introduction to NVIDIA Isaac Sim

### What is Isaac Sim?

Isaac Sim is a digital twin platform built on Omniverse that provides:
- **High-Fidelity Rendering**: Photo-realistic graphics using NVIDIA RTX
- **Physics Simulation**: Accurate physics with NVIDIA PhysX
- **Synthetic Data**: Automatic labeling for computer vision training
- **ROS 2 Integration**: Native support for ROS topics and services
- **RTX Rendering**: Real-time ray tracing for realistic sensor simulation
- **Digital Twins**: Create virtual versions of physical environments

### Installation

**System Requirements:**
- NVIDIA GPU with compute capability 6.0+ (RTX series recommended)
- 16 GB+ VRAM for comfortable development
- 100 GB+ SSD storage
- Ubuntu 20.04 or 22.04

**Installation Steps:**

```bash
# Add NVIDIA's package repository
wget -qO - https://repo.nvidia.com/nvidia-gaming/KEY.gpg | sudo apt-key add -
sudo apt-add-repository 'deb https://repo.nvidia.com/nvidia-gaming ubuntu focal main'

# Update package list
sudo apt-get update

# Install Isaac Sim
sudo apt-get install isaac-sim

# Set environment
export PATH=$PATH:$HOME/.local/share/ov/pkg/isaac-sim-version/bin
```

### First Steps: Create a Simple Scene

**Open Isaac Sim and create a basic scene:**

1. Launch Isaac Sim
2. Create new scene (File → New)
3. Add a ground plane (right-click → Add → Models → Ground Plane)
4. Add a robot (right-click → Add → Models → Robot)
5. Add a camera sensor (right-click → Add → Camera)

**Save as: `scenes/my_first_scene.usd`**

### USD (Universal Scene Description)

USD is the underlying format for Isaac Sim scenes. You can edit scenes programmatically:

```python
from omni.isaac.kit import SimulationApp

# Initialize simulation
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World

# Create a world
world = World(stage_units_in_meters=1.0)

# Add objects
from omni.isaac.core.objects import DynamicCuboid

cube = world.scene.add(
    DynamicCuboid(
        name="cube",
        position=[0, 0, 1.0],
        scale=[0.1, 0.1, 0.1],
    )
)

# Step simulation
world.step(render=True)

# Cleanup
simulation_app.close()
```

### Connecting to ROS 2

**Enable ROS 2 in Isaac Sim:**

```yaml
# config/isaac_sim_config.yaml
ros2:
  enabled: true
  namespace: /isaac_sim
  domain_id: 0
```

**Access topic data in ROS 2:**

```bash
# List available topics from Isaac Sim
ros2 topic list

# Subscribe to camera data
ros2 topic echo /isaac_sim/camera/rgb
```

## Chapter 2: Synthetic Data Generation

### Why Synthetic Data?

| Aspect | Real Data | Synthetic Data |
|--------|-----------|----------------|
| **Collection Time** | Weeks/Months | Hours |
| **Cost** | High (equipment, labor) | Low (computation) |
| **Annotation** | Manual (time-consuming) | Automatic |
| **Variety** | Limited by real scenarios | Unlimited diversity |
| **Privacy** | Concerns with real places | No privacy issues |
| **Edge Cases** | Hard to capture | Easy to generate |

### Creating a Synthetic Dataset

**Step 1: Define the Scenario**

```python
from omni.isaac.synthetic_recorder import SyntheticRecorder

class DatasetGenerator:
    def __init__(self):
        self.recorder = SyntheticRecorder()
        self.output_dir = "datasets/synthetic_data"
    
    def generate_scene(self):
        """Create a scene with objects to detect"""
        # Add ground
        # Add table
        # Add objects (cups, blocks, toys)
        # Add lighting variations
        pass
    
    def randomize_poses(self):
        """Randomly place objects for diversity"""
        import random
        
        num_objects = random.randint(1, 5)
        for i in range(num_objects):
            x = random.uniform(-0.5, 0.5)
            y = random.uniform(-0.5, 0.5)
            z = 1.0  # On table
            # Set object pose
        pass
```

**Step 2: Configure Sensors**

```python
# RGB Camera
camera_rgb = {
    "resolution": [1920, 1080],
    "fov": 60,
    "focal_length": 24,
}

# Depth Camera
camera_depth = {
    "resolution": [640, 480],
    "fov": 69,
    "min_depth": 0.1,
    "max_depth": 5.0,
}
```

**Step 3: Generate Frames**

```python
def generate_frames(num_frames=1000):
    for frame in range(num_frames):
        # Randomize scene
        randomize_poses()
        randomize_lighting()
        randomize_camera_pose()
        
        # Capture data
        rgb = capture_rgb()
        depth = capture_depth()
        segmentation = capture_segmentation()
        bounding_boxes = extract_bounding_boxes()
        
        # Save frame
        save_frame({
            'rgb': rgb,
            'depth': depth,
            'segmentation': segmentation,
            'bboxes': bounding_boxes,
            'frame_id': frame
        })
```

**Step 4: Output Format**

```
datasets/synthetic_data/
├── rgb/
│   ├── frame_0000.png
│   ├── frame_0001.png
│   └── ...
├── depth/
│   ├── frame_0000.png
│   └── ...
├── annotations/
│   ├── frame_0000.json
│   ├── frame_0001.json
│   └── ...
└── metadata.json
```

**metadata.json example:**
```json
{
  "dataset_name": "synthetic_objects_v1",
  "num_frames": 1000,
  "classes": [
    {"id": 0, "name": "cup"},
    {"id": 1, "name": "block"},
    {"id": 2, "name": "ball"}
  ],
  "split": {"train": 0.8, "val": 0.1, "test": 0.1},
  "camera": {
    "resolution": [1920, 1080],
    "fov": 60
  }
}
```

## Chapter 3: Isaac ROS Integration

### What is Isaac ROS?

Isaac ROS provides optimized ROS 2 nodes for robotics perception:
- **NVIDIA GPU accelerated**: CUDA-optimized processing
- **ROS 2 Native**: Standard ROS topic/service interface
- **Pre-built Models**: Object detection, pose estimation, etc.
- **Real-time Performance**: Inference at 30+ FPS

### Common Isaac ROS Nodes

**Object Detection:**
```python
from isaac_ros_detectnet import DetectNetNode

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('detector')
        self.detector = DetectNetNode(model='peoplenet')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb',
            self.image_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )
    
    def image_callback(self, msg):
        detections = self.detector.infer(msg)
        self.publisher.publish(detections)
```

**Pose Estimation:**
```python
from isaac_ros_posenet import PoseEstimationNode

class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        self.pose_estimator = PoseEstimationNode(model='resnet18')
        
        self.publisher = self.create_publisher(
            PoseEstimationResults,
            '/pose_estimates',
            10
        )
```

### Integrating with Nav2

**Launch file: `launch/perception_nav2.launch.py`**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Isaac ROS detection node
        Node(
            package='isaac_ros_detectnet',
            executable='detectnet_node',
            name='detector',
            parameters=[{'model': 'peoplenet'}]
        ),
        
        # Nav2 stack
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            parameters=[{'autostart': True}]
        ),
        
        # Custom costmap updater
        Node(
            package='my_robot',
            executable='costmap_updater',
            name='costmap_updater'
        ),
    ])
```

## Chapter 4: Navigation with Nav2

### What is Nav2?

Nav2 (Navigation 2) is a ROS 2 framework for autonomous navigation:
- **Path Planning**: Global planner (Dijkstra, A*, etc.)
- **Local Navigation**: DWB (Dynamic Window Approach)
- **Behavior Trees**: Task execution and recovery
- **Costmaps**: Occupancy grids for obstacle avoidance

### Basic Nav2 Configuration

**config/nav2_params.yaml:**

```yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: base_link
    beam_search_angle: 0.545
    lambda_short: 0.1
    laser_likelihood_max_desc_occ_dist: 2.0
    laser_max_range: 12.0
    laser_min_range: 0.1
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: odom
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.3
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: nav2_amcl::DifferentialMotionModel
    save_pose_rate: 0.5
    sigma_hit: 0.2
    sigma_short: 0.05
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    default_bt_xml_filename: behavior_trees/navigate_w_replanning_and_recovery.xml
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
```

### Autonomous Navigation Example

```python
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator

def navigate_to_goal():
    rclpy.init()
    
    nav = BasicNavigator()
    nav.waitUntilNav2Active()
    
    # Define goal pose
    from geometry_msgs.msg import PoseStamped
    
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = rclpy.clock.Clock().now().to_msg()
    goal_pose.pose.position.x = 5.0
    goal_pose.pose.position.y = 5.0
    goal_pose.pose.orientation.w = 1.0
    
    # Send goal
    nav.goToPose(goal_pose)
    
    # Monitor navigation
    while not nav.isNavComplete():
        feedback = nav.getFeedback()
        if feedback:
            print(f"Remaining distance: {feedback.distance_remaining:.1f}m")
    
    print("Navigation complete!")
    rclpy.shutdown()

if __name__ == '__main__':
    navigate_to_goal()
```

## Chapter 5: Sim-to-Real Transfer

### Domain Randomization

Techniques to make trained models work on real robots:

**1. Visual Randomization**
```python
def randomize_visual_appearance():
    # Random object colors
    # Random lighting conditions
    # Random textures
    # Random camera distortion
    pass
```

**2. Physics Randomization**
```python
def randomize_physics():
    # Random friction coefficients
    # Random mass variations
    # Random joint stiffness
    # Random actuator delays
    pass
```

**3. Environmental Randomization**
```python
def randomize_environment():
    # Random obstacle placement
    # Random floor friction
    # Random object poses
    # Random noise in sensor readings
    pass
```

### Transfer Pipeline

```python
class TransferPipeline:
    def __init__(self):
        self.sim_model = load_model('sim_trained_model.pth')
        self.real_adapter = load_adapter('adapter.pth')
    
    def forward(self, real_input):
        # Adapter network bridges sim-to-real gap
        adapted_features = self.real_adapter(real_input)
        
        # Run inference
        output = self.sim_model(adapted_features)
        
        return output
```

## Exercises

### Lab 3.1: Generate Synthetic Dataset

**Objective**: Create a dataset of 100 synthetic images with object annotations

**Steps**:
1. Create a scene in Isaac Sim with a table and objects
2. Configure RGB and depth cameras
3. Implement randomization (pose, lighting, background)
4. Generate 100 frames with bounding box annotations
5. Export dataset in COCO format

**Expected Output**: 
```
datasets/synthetic_objects/
├── images/     (100 PNG files)
├── annotations.json  (COCO format)
└── metadata.json
```

### Lab 3.2: Configure Nav2 Navigation

**Objective**: Set up Nav2 on a simulated robot and navigate to goal poses

**Steps**:
1. Create Nav2 config files (amcl, bt_navigator, costmap)
2. Launch Nav2 with simulated robot
3. Send multiple navigation goals programmatically
4. Record and visualize navigation path

**Expected Output**:
```
Robot reaches goal pose with < 0.1m error
Navigation completes in ~30 seconds
Obstacle avoidance prevents collisions
```

## Key Takeaways

✓ Isaac Sim enables high-fidelity physics and rendering
✓ Synthetic data solves annotation and privacy challenges
✓ Isaac ROS provides GPU-accelerated perception nodes
✓ Nav2 enables autonomous navigation with ROS 2
✓ Domain randomization bridges simulation and reality

## References

- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/)
- [Isaac ROS GitHub](https://github.com/NVIDIA-ISAAC-ROS)
- [Nav2 Documentation](https://nav2.org/)
- [Domain Randomization Paper](https://arxiv.org/abs/1703.06907)
- [Synthetic Data Best Practices](https://developer.nvidia.com/blog/synthetic-data-what-it-is-and-why-its-important/)
