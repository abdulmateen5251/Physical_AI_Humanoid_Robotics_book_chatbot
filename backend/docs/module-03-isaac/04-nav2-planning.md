# Chapter 4: Nav2 Path Planning Integration

## What is Nav2?

Nav2 provides navigation stack for autonomous robots:
- **Global Planning**: Compute paths avoiding obstacles
- **Local Planning**: Real-time obstacle avoidance
- **Behavior Trees**: Complex navigation behaviors
- **ROS 2 Native**: Full ROS 2 ecosystem integration

## Nav2 Launch Configuration

**nav2_launch.py:**
```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get paths
    nav2_launch_dir = os.path.join(
        FindPackageShare('nav2_bringup').find('nav2_bringup'),
        'launch'
    )
    
    # Include Nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'True',
            'map': '',
            'use_sim_time': 'true',
            'params_file': 'nav2_params.yaml'
        }.items()
    )
    
    return LaunchDescription([nav2_launch])
```

**nav2_params.yaml:**
```yaml
amcl:
  ros__parameters:
    use_sim_time: true
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: base_footprint
    global_frame_id: map
    odom_frame_id: odom
    beam_search_angle: 0.545
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: differential
    save_pose_rate: 0.5
    sigma_hit: 0.2
    sigma_short: 0.05
    do_beamskip: false
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: likelihood_field
    init_pose_x: 0.0
    init_pose_y: 0.0
    init_pose_a: 0.0
    initial_pose_covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.016]
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_footprint
    odom_topic: /odom
    bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    default_nav_through_poses_bt_xml: "navigate_through_poses_w_replanning_and_recovery.xml"
    default_nav_to_pose_bt_xml: "navigate_to_pose_w_replanning_and_recovery.xml"

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: nav2_core/SimpleProgressChecker
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    general_goal_checker:
      plugin: nav2_core/SimpleGoalChecker
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: true

    FollowPath:
      plugin: nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
      desired_linear_vel: 0.5
      manifold_radius: 0.25
      inflation_radius: 0.55
      use_velocity_scaled_lookahead_dist: false
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.8
      transform_tolerance: 0.1
      use_collision_detection: true
      max_allowed_time_error_ratio: 0.1
      use_regulated_linear_velocity_scaling: true
      use_fixed_curvature_scaling: false
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_mult: 6.0
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785
      max_angular_accel: 3.2
      max_robot_pose_search_dist: 10.0
      use_interpolation: false
```

## Autonomous Navigation Example

**BasicNavigator with Isaac Sim:**
```python
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import rclpy
from math import pi

class IsaacNavigator:
    def __init__(self):
        rclpy.init()
        self.nav = BasicNavigator()
        
        # Create initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        
        self.nav.setInitialPose(initial_pose)
        self.nav.waitUntilNav2Active(localizer='amcl')
    
    def navigate_to_goal(self, x, y, theta):
        """Navigate to a goal position"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        
        # Convert theta to quaternion
        from tf_transformations import quaternion_from_euler
        qx, qy, qz, qw = quaternion_from_euler(0, 0, theta)
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw
        
        self.nav.goToPose(goal_pose)
    
    def navigate_through_poses(self, poses_list):
        """Navigate through multiple waypoints"""
        goal_poses = []
        
        for x, y, theta in poses_list:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.nav.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            
            from tf_transformations import quaternion_from_euler
            qx, qy, qz, qw = quaternion_from_euler(0, 0, theta)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            
            goal_poses.append(pose)
        
        self.nav.goThroughPoses(goal_poses)
    
    def wait_for_navigation(self):
        """Wait for navigation to complete"""
        i = 0
        while not self.nav.isNavComplete():
            i += 1
            feedback = self.nav.getFeedback()
            if feedback and i % 5 == 0:
                print(f"Distance remaining: {feedback.distance_remaining:.2f}m")
            rclpy.spin_once(self.nav, timeout_sec=0.1)
        
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            return True
        else:
            return False

# Usage
if __name__ == '__main__':
    navigator = IsaacNavigator()
    
    # Navigate to single goal
    navigator.navigate_to_goal(x=5.0, y=5.0, theta=0.0)
    success = navigator.wait_for_navigation()
    print(f"Navigation {'successful' if success else 'failed'}")
    
    # Navigate through waypoints
    waypoints = [(1.0, 1.0, 0.0), (3.0, 3.0, pi/2), (5.0, 1.0, pi)]
    navigator.navigate_through_poses(waypoints)
    success = navigator.wait_for_navigation()
    print(f"Waypoint navigation {'successful' if success else 'failed'}")
```

## SLAM (Simultaneous Localization and Mapping)

**SLAM Setup:**
```bash
# Install SLAM Toolbox
sudo apt-get install ros-humble-slam-toolbox

# Launch with SLAM enabled
ros2 launch nav2_bringup bringup_launch.py slam:=True
```

**Saving Maps:**
```bash
# Map server saves map to file
ros2 run nav2_map_server map_saver_cli -f ~/my_map

# Load saved map
ros2 launch nav2_bringup bringup_launch.py map:=~/my_map.yaml
```

## Performance Metrics

**Navigation Performance:**
- Planning time: 50-100ms
- Control frequency: 20Hz
- Max velocity: configurable (default 0.5 m/s)
- Rotation time to target: ~2-5 seconds
