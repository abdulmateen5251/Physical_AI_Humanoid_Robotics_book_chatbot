# Module 4: Vision-Language-Action (VLA)

## Learning Objectives

By the end of this module, you will be able to:
- Convert voice input to text using Whisper and other speech recognition systems
- Use Large Language Models (LLMs) to translate natural language into robot action sequences
- Implement vision-language models for multi-modal understanding
- Design safety validators to prevent unsafe robot commands
- Build a closed-loop control system that executes LLM-generated plans
- Integrate speech recognition, language models, and robot control in ROS 2
- Handle edge cases and improve robustness through feedback loops

## Module Overview

Module 4 brings together multiple AI modalities (Vision, Language, Action) to create intelligent robots that understand human commands and execute them safely.

### Key Components
- **Whisper**: Speech-to-text (OpenAI)
- **LLMs**: Plan generation (GPT-4, Claude, LLaMA)
- **Vision Models**: Multi-modal understanding (CLIP, GPT-4V)
- **Validators**: Safety checking and constraint enforcement
- **Executors**: Convert plans to ROS 2 actions

## Chapter 1: Speech Recognition with Whisper

### What is Whisper?

Whisper is a speech recognition model that:
- **Handles multiple languages**: 99 languages supported
- **Robust to noise**: Works in noisy environments
- **Free and open**: Available via OpenAI API or locally
- **Fast**: Real-time transcription possible with quantization

### Installation and Setup

**Install required packages:**

```bash
pip install openai-whisper
pip install librosa  # Audio processing
pip install pyaudio  # Microphone input
```

**Download model:**

```bash
# Small model (774M)
whisper tiny

# Medium model (1.5GB)  
whisper base

# Large model (2.9GB)
whisper small
```

### Real-time Speech Recognition

**Capture and transcribe microphone input:**

```python
import whisper
import pyaudio
import numpy as np
from threading import Thread

class WhisperRecognizer:
    def __init__(self, model_name='base'):
        self.model = whisper.load_model(model_name)
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paFloat32
        self.CHANNELS = 1
        self.RATE = 16000
        self.recording = False
        self.audio_data = []
    
    def record_audio(self, duration=5):
        """Record audio for specified duration (seconds)"""
        p = pyaudio.PyAudio()
        stream = p.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK
        )
        
        print(f"Recording for {duration} seconds...")
        
        frames = []
        for _ in range(0, int(self.RATE / self.CHUNK * duration)):
            data = stream.read(self.CHUNK)
            frames.append(np.frombuffer(data, dtype=np.float32))
        
        stream.stop_stream()
        stream.close()
        p.terminate()
        
        return np.concatenate(frames)
    
    def transcribe(self, audio_data):
        """Transcribe audio to text"""
        result = self.model.transcribe(audio_data, language="en")
        return result['text']
    
    def recognize(self, duration=5):
        """Record and transcribe in one call"""
        audio = self.record_audio(duration)
        text = self.transcribe(audio)
        return text

# Example usage
if __name__ == '__main__':
    recognizer = WhisperRecognizer()
    command = recognizer.recognize(duration=5)
    print(f"You said: {command}")
```

### Integration with ROS 2

**Create a ROS 2 node for voice commands:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.recognizer = WhisperRecognizer(model_name='base')
        
        # Publisher for recognized commands
        self.publisher = self.create_publisher(
            String,
            '/voice_commands',
            10
        )
        
        # Timer to continuously listen
        self.timer = self.create_timer(
            10.0,  # Listen every 10 seconds
            self.listen_callback
        )
    
    def listen_callback(self):
        try:
            command = self.recognizer.recognize(duration=5)
            self.get_logger().info(f"Command: {command}")
            
            msg = String()
            msg.data = command
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Recognition error: {e}")

def main():
    rclpy.init()
    node = VoiceCommandNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Chapter 2: LLM-Based Planning

### Converting Commands to Plans

An LLM planner converts natural language into executable action sequences:

```python
import openai
from typing import List, Dict

class RobotPlanner:
    def __init__(self, api_key: str):
        self.client = openai.OpenAI(api_key=api_key)
        
        # Define allowed actions
        self.allowed_actions = [
            'move_forward', 'turn_left', 'turn_right',
            'grasp', 'release', 'move_arm'
        ]
    
    def plan(self, command: str) -> List[Dict]:
        """Convert natural language command to action plan"""
        
        system_prompt = f"""You are a robot motion planner. Your task is to convert
natural language commands into a sequence of robot actions.

Available actions:
{json.dumps(self.allowed_actions, indent=2)}

Return a JSON array of actions with parameters.

Example:
Input: "Pick up the red block on the table"
Output: [
  {{"action": "move_forward", "distance": 1.0}},
  {{"action": "grasp", "force": 10.0}},
  {{"action": "move_arm", "x": 0.5, "y": 0.5, "z": 1.0}}
]

Be concise and return only valid JSON."""
        
        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": command}
            ],
            temperature=0.3  # Lower temperature for deterministic output
        )
        
        plan_text = response.choices[0].message.content
        
        # Parse JSON from response
        import json
        import re
        
        json_match = re.search(r'\[.*\]', plan_text, re.DOTALL)
        if json_match:
            plan = json.loads(json_match.group())
            return plan
        else:
            return []

# Example
if __name__ == '__main__':
    planner = RobotPlanner(api_key='your-api-key')
    
    command = "Move forward one meter and then turn left"
    plan = planner.plan(command)
    
    print("Generated plan:")
    for action in plan:
        print(f"  - {action}")
```

### Selection-Mode Constraint Enforcement

When operating in "selection mode" (limited context), enforce that actions only reference information in the selected text:

```python
class ConstrainedPlanner(RobotPlanner):
    def plan_with_selection(self, command: str, selected_text: str) -> List[Dict]:
        """Plan using only information in selected text"""
        
        system_prompt = f"""You are a constrained robot planner. IMPORTANT:
You may ONLY use information present in the selected text below.
Do not use any external knowledge or assumptions.

SELECTED TEXT:
{selected_text}

Natural language command:
{command}

If the selected text does not contain enough information to plan, 
respond with: {{"error": "insufficient_information"}}

Otherwise, return a JSON array of actions."""
        
        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": command}
            ],
            temperature=0.1
        )
        
        return self._parse_response(response)
```

## Chapter 3: Safety Validation

### Constraint Checker

Ensure that generated plans satisfy safety constraints:

```python
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
            }
        }
    
    def validate_plan(self, plan: List[Dict], state: Dict) -> tuple:
        """
        Validate plan against constraints
        
        Returns: (is_valid: bool, violations: List[str])
        """
        violations = []
        
        for action in plan:
            action_type = action.get('action')
            
            # Validate movement actions
            if action_type == 'move_forward':
                distance = action.get('distance', 0)
                if distance > 10.0:
                    violations.append(
                        f"Movement too large: {distance}m > 10m limit"
                    )
            
            # Validate grasp actions
            elif action_type == 'grasp':
                force = action.get('force', 0)
                if force > 100:
                    violations.append(
                        f"Grasp force too high: {force} > 100 limit"
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
                            f"Joint {joint} angle out of limits: {angle} rad"
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
            
            fixed_plan.append(fixed_action)
        
        return fixed_plan
```

## Chapter 4: Integration and Execution

### Complete VLA Pipeline

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class VLAController(Node):
    def __init__(self):
        super().__init__('vla_controller')
        
        # Initialize components
        self.recognizer = WhisperRecognizer()
        self.planner = RobotPlanner(api_key='your-key')
        self.validator = SafetyValidator()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/vla_status', 10)
        
        # Subscribers
        self.create_subscription(String, '/voice_commands', 
                                self.command_callback, 10)
        
        # Current state
        self.robot_state = {}
        self.current_action_idx = 0
        self.current_plan = []
    
    def command_callback(self, msg: String):
        """Process incoming voice command"""
        command = msg.data
        self.get_logger().info(f"Processing: {command}")
        
        # Step 1: Generate plan
        plan = self.planner.plan(command)
        self.current_plan = plan
        
        # Step 2: Validate plan
        is_valid, violations = self.validator.validate_plan(plan, self.robot_state)
        
        if not is_valid:
            self.get_logger().warn(f"Constraint violations: {violations}")
            # Attempt recovery
            plan = self.validator.recover_plan(plan, violations)
        
        # Step 3: Execute plan
        self.execute_plan(plan)
    
    def execute_plan(self, plan: List[Dict]):
        """Execute action sequence"""
        for action in plan:
            self.execute_action(action)
    
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
    
    def move_forward(self, distance: float):
        """Move robot forward"""
        twist = Twist()
        twist.linear.x = 0.5  # m/s
        
        # Duration based on distance
        duration = distance / 0.5  # seconds
        
        for _ in range(int(duration * 10)):
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f"Moving forward: {distance}m")
        
        # Stop
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def turn(self, angle: float):
        """Rotate robot"""
        twist = Twist()
        twist.angular.z = 0.5  # rad/s
        
        duration = abs(angle) / 0.5
        
        sign = 1 if angle > 0 else -1
        twist.angular.z = 0.5 * sign
        
        for _ in range(int(duration * 10)):
            self.cmd_vel_pub.publish(twist)
        
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def grasp(self, force: float):
        """Grasp object with end effector"""
        self.get_logger().info(f"Grasping with force: {force}")
        # Send grasp command to manipulator
```

## Chapter 5: Vision-Language Integration

### Multi-Modal Understanding

Combine vision and language for grounded understanding:

```python
import base64
from pathlib import Path

class VisionLanguageModel:
    def __init__(self, api_key: str):
        self.client = openai.OpenAI(api_key=api_key)
    
    def understand_scene(self, image_path: str, query: str) -> str:
        """Understand an image in context of a query"""
        
        # Encode image to base64
        with open(image_path, 'rb') as image_file:
            image_data = base64.standard_b64encode(image_file.read()).decode('utf-8')
        
        response = self.client.chat.completions.create(
            model="gpt-4-vision",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{image_data}"
                            }
                        },
                        {
                            "type": "text",
                            "text": query
                        }
                    ]
                }
            ]
        )
        
        return response.choices[0].message.content
    
    def ground_objects(self, image_path: str) -> List[Dict]:
        """Identify and locate objects in image"""
        
        query = """List all objects you see in this image with their approximate
locations (e.g., "top-left", "center", "bottom-right"). 
Return as JSON array with fields: object, location, confidence."""
        
        response_text = self.understand_scene(image_path, query)
        
        import json
        import re
        
        json_match = re.search(r'\[.*\]', response_text, re.DOTALL)
        if json_match:
            return json.loads(json_match.group())
        return []
```

## Exercises

### Lab 4.1: Build a Voice-Controlled Robot

**Objective**: Create a complete VLA system that listens to voice commands and executes robot actions

**Steps**:
1. Implement WhisperRecognizer node
2. Create RobotPlanner with 5 basic actions
3. Add SafetyValidator for constraints
4. Integrate with a simulated robot (Gazebo)
5. Test with voice commands

**Expected Commands**:
- "Move forward 2 meters"
- "Turn around and go back"
- "Pick up the object"

**Expected Output**: Robot executes commanded actions safely

### Lab 4.2: Implement Selection-Mode Planning

**Objective**: Create a planner that only uses information from selected text

**Steps**:
1. Implement ConstrainedPlanner
2. Create test cases with sample text passages
3. Test that planner refuses to use external knowledge
4. Add error handling for insufficient information

**Test Case**:
```
Selected Text: "The robot can move forward or backward at 1 m/s."
Query: "Turn the robot 90 degrees"

Expected: Error (turning not mentioned in selected text)
```

## Key Takeaways

✓ Whisper enables real-time voice-to-text conversion
✓ LLMs can translate natural language to structured action plans
✓ Safety validators prevent unsafe robot behaviors
✓ Vision-language models enable grounded understanding
✓ Complete integration creates intuitive robot control

## References

- [OpenAI Whisper](https://openai.com/research/whisper)
- [GPT-4 API Documentation](https://platform.openai.com/docs/guides/gpt-4)
- [GPT-4 Vision](https://openai.com/research/gpt-4v-system-card)
- [ROS 2 Bridging](https://docs.ros.org/en/humble/Concepts/About-ROS-2.html)
- [Safety in Autonomous Systems](https://arxiv.org/abs/2106.07535)
- [Planning with LLMs](https://arxiv.org/abs/2305.14992)
