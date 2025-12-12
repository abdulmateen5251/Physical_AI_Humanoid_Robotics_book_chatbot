/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

module.exports = {
  tutorialSidebar: {
    'Module 1: ROS 2 Fundamentals': [
      'module-01-ros2/introduction',
      'module-01-ros2/nodes-topics-services',
    ],
    'Module 2: The Digital Twin (Gazebo & Unity)': [
      'module-02-gazebo/gazebo-setup',
      'module-02-gazebo/urdf-sdf',
      'module-02-gazebo/sensor-simulation',
      'module-02-gazebo/ros2-integration',
      'module-02-gazebo/labs-exercises',
    ],
    'Module 3: AI-Robot Brain (NVIDIA Isaac)': [
      'module-03-isaac/isaac-ecosystem',
      'module-03-isaac/synthetic-data',
      'module-03-isaac/isaac-ros2',
      'module-03-isaac/nav2-planning',
      'module-03-isaac/sim-to-real',
    ],
    'Module 4: Vision-Language-Action (VLA)': [
      'module-04-vla/whisper-integration',
      'module-04-vla/llm-planning',
      'module-04-vla/safety-validation',
      'module-04-vla/vla-integration',
      'module-04-vla/vision-language',
    ],
  },
};
