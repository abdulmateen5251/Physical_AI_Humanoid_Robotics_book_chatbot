# Chapter 5: Vision-Language Models and Multi-Modal Reasoning

## Vision-Language Model Fundamentals

Multi-modal models that understand images and text:

```python
import base64
from pathlib import Path
import openai

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
For each object, provide:
- name: object name
- location: position in image
- confidence: 0-1 confidence score

Return as JSON array."""
        
        response_text = self.understand_scene(image_path, query)
        
        # Parse JSON
        import json
        import re
        
        json_match = re.search(r'\[.*\]', response_text, re.DOTALL)
        if json_match:
            return json.loads(json_match.group())
        return []
    
    def scene_understanding(self, image_path: str) -> Dict:
        """Get comprehensive scene understanding"""
        
        queries = {
            'objects': "What objects are in this image?",
            'spatial_relations': "What are the spatial relationships between objects?",
            'hazards': "Are there any potential safety hazards visible?",
            'robot_context': "In what ways could a robot interact with objects in this image?"
        }
        
        understanding = {}
        
        for key, query in queries.items():
            understanding[key] = self.understand_scene(image_path, query)
        
        return understanding

# Usage
if __name__ == '__main__':
    vlm = VisionLanguageModel(api_key='your-key')
    
    # Get scene understanding
    scene = vlm.scene_understanding('robot_scene.jpg')
    print(f"Scene understanding:\n{json.dumps(scene, indent=2)}")
    
    # Ground objects
    objects = vlm.ground_objects('robot_scene.jpg')
    print(f"Objects:\n{json.dumps(objects, indent=2)}")
```

## Visual Grounding with Coordinates

**Get precise object locations:**

```python
class VisualGrounder:
    def __init__(self, model):
        self.model = model
    
    def get_bounding_boxes(self, image_path: str) -> List[Dict]:
        """Get bounding boxes for detected objects"""
        
        query = """Analyze this image and for each object provide:
- object_name: name of object
- bounding_box: [x_min, y_min, x_max, y_max] as percentage (0-100)
- category: (robot, gripper, table, object, etc)
- graspable: true/false

Return as JSON array with fields exactly as specified above."""
        
        response = self.model.understand_scene(image_path, query)
        
        import json
        import re
        json_match = re.search(r'\[.*\]', response, re.DOTALL)
        
        if json_match:
            return json.loads(json_match.group())
        return []
    
    def pixel_to_world_coordinates(self, image_shape, bbox_percent, camera_calib):
        """Convert image coordinates to world coordinates"""
        import numpy as np
        
        img_h, img_w = image_shape
        x_min, y_min, x_max, y_max = bbox_percent
        
        # Convert percentage to pixels
        x_min_px = int(x_min * img_w / 100)
        y_min_px = int(y_min * img_h / 100)
        x_max_px = int(x_max * img_w / 100)
        y_max_px = int(y_max * img_h / 100)
        
        # Center of bbox
        center_x = (x_min_px + x_max_px) / 2
        center_y = (y_min_px + y_max_px) / 2
        
        # Use camera calibration to convert to world coordinates
        # This requires camera intrinsics and depth information
        
        # Placeholder: simple projection
        world_x = center_x * camera_calib['focal_length'] / img_w
        world_y = center_y * camera_calib['focal_length'] / img_h
        
        return (world_x, world_y)
```

## CLIP for Semantic Search

**Use CLIP for zero-shot vision-language understanding:**

```python
class CLIPSemanticSearch:
    def __init__(self):
        import clip
        import torch
        
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model, self.preprocess = clip.load("ViT-B/32", device=self.device)
    
    def encode_text(self, text_queries: List[str]):
        """Encode text queries"""
        import clip
        import torch
        
        text_tokens = clip.tokenize(text_queries).to(self.device)
        with torch.no_grad():
            text_features = self.model.encode_text(text_tokens)
        
        return text_features
    
    def encode_image(self, image_path: str):
        """Encode image"""
        import torch
        from PIL import Image
        
        image = Image.open(image_path)
        image_tensor = self.preprocess(image).unsqueeze(0).to(self.device)
        
        with torch.no_grad():
            image_features = self.model.encode_image(image_tensor)
        
        return image_features
    
    def find_similar_objects(self, image_path: str, object_names: List[str]):
        """Find objects in image matching descriptions"""
        import torch
        
        image_features = self.encode_image(image_path)
        text_features = self.encode_text(object_names)
        
        # Normalize features
        image_features /= image_features.norm(dim=-1, keepdim=True)
        text_features /= text_features.norm(dim=-1, keepdim=True)
        
        # Compute similarities
        similarities = 100.0 * image_features @ text_features.T
        
        return {
            'object_names': object_names,
            'similarities': similarities.squeeze().cpu().numpy().tolist()
        }
```

## Reasoning with Multi-Modal Context

**Combine vision and language for complex reasoning:**

```python
class MultiModalReasoner:
    def __init__(self, vlm, clip_model):
        self.vlm = vlm
        self.clip = clip_model
    
    def reason_about_task(self, image_path: str, task_description: str) -> Dict:
        """Reason about task execution in given scene"""
        
        # Step 1: Get scene understanding
        scene_understanding = self.vlm.scene_understanding(image_path)
        
        # Step 2: Find relevant objects using CLIP
        clip_results = self.clip.find_similar_objects(
            image_path,
            task_description.split()
        )
        
        # Step 3: Generate grounding query
        grounding_query = f"""Given this scene with objects: {scene_understanding['objects']}
        
Task: {task_description}

Which objects should the robot interact with and in what order?
Provide reasoning and step-by-step plan."""
        
        reasoning = self.vlm.understand_scene(image_path, grounding_query)
        
        return {
            'scene': scene_understanding,
            'task': task_description,
            'relevant_objects': clip_results,
            'reasoning': reasoning
        }
    
    def generate_grounded_plan(self, image_path: str, command: str) -> List[Dict]:
        """Generate plan grounded in visual scene"""
        
        reasoning = self.reason_about_task(image_path, command)
        
        # Extract plan from reasoning
        plan_query = f"""Based on this reasoning: {reasoning['reasoning']}
        
        Generate a step-by-step action plan as JSON:
        [
          {{"action": "action_name", "target_object": "name", "params": {{}}},
          ...
        ]"""
        
        plan_response = self.vlm.understand_scene(image_path, plan_query)
        
        import json
        import re
        json_match = re.search(r'\[.*\]', plan_response, re.DOTALL)
        
        if json_match:
            return json.loads(json_match.group())
        return []
```

## Temporal Reasoning

**Understand action sequences over time:**

```python
class TemporalVisualReasoner:
    def __init__(self, vlm):
        self.vlm = vlm
    
    def analyze_sequence(self, image_sequence: List[str], task: str) -> Dict:
        """Analyze sequence of images for task understanding"""
        
        # Encode all images
        analysis = {
            'task': task,
            'frames': [],
            'temporal_changes': []
        }
        
        for idx, image_path in enumerate(image_sequence):
            frame_analysis = {
                'frame': idx,
                'objects': self.vlm.ground_objects(image_path),
                'description': self.vlm.understand_scene(
                    image_path,
                    "Describe what is happening in this image."
                )
            }
            analysis['frames'].append(frame_analysis)
        
        # Analyze temporal changes
        for i in range(len(image_sequence) - 1):
            change_analysis = self.vlm.understand_scene(
                image_sequence[i],
                f"Compare this to the next frame. What changed? "
                f"Is the task '{task}' progressing as expected?"
            )
            analysis['temporal_changes'].append(change_analysis)
        
        return analysis
```

## Integration with Robot Control

**Use vision-language understanding for robot control:**

```python
class VisionLanguageController:
    def __init__(self, vlm, robot_controller):
        self.vlm = vlm
        self.robot = robot_controller
    
    def execute_task_with_vision(self, image_path: str, task: str):
        """Execute task with continuous visual feedback"""
        
        # Understand task in visual context
        plan = self.vlm_reasoner.generate_grounded_plan(image_path, task)
        
        # Execute with visual tracking
        for action_idx, action in enumerate(plan):
            target = action.get('target_object')
            
            # Get current image
            current_image = self.robot.get_camera_feed()
            
            # Find target in current image
            objects = self.vlm.ground_objects(current_image)
            target_obj = next((o for o in objects if target in o['name'].lower()), None)
            
            if target_obj:
                # Extract coordinates
                bbox = target_obj.get('bounding_box', [0, 0, 100, 100])
                
                # Execute action
                self.robot.execute_action(action, target_bbox=bbox)
            else:
                self.robot.get_logger().warn(f"Target {target} not found in image")
```

## Performance and Limitations

**Performance Metrics:**
- GPT-4V inference: 1-5 seconds per image
- CLIP embedding: 100-500ms per image
- Max images for batch analysis: 10-20

**Limitations:**
- Hallucinations possible (false detections)
- Requires clear, well-lit images
- Context window limits for long sequences
- Expensive API calls for continuous monitoring

**Best Practices:**
✓ Always validate VLM outputs with robot sensors
✓ Use conservative plans when confidence is low
✓ Cache embeddings for repeated queries
✓ Fall back to simpler rules if VLM fails
✓ Test with diverse lighting conditions
