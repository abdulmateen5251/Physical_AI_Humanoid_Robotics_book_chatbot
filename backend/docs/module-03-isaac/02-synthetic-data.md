# Chapter 2: Synthetic Data Generation Pipeline

## Why Synthetic Data?

Synthetic data solves key challenges:
- **Data Volume**: Generate unlimited labeled data
- **Safety**: Test with unlimited failure scenarios
- **Cost**: No physical wear and tear
- **Privacy**: No real-world recording required
- **Reproducibility**: Exact scene control

## Data Generation Pipeline

**Complete Pipeline:**
```python
import numpy as np
from isaacsim import SimulationApp
from omni.isaac.core import World
from omni.isaac.synthetic_utils import SyntheticDataHelper
import json
from pathlib import Path

class SyntheticDataGenerator:
    def __init__(self, output_dir='./synthetic_data'):
        self.simulation_app = SimulationApp({"headless": True})
        self.world = World(stage_units_in_meters=1.0)
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # Image counter
        self.frame_count = 0
    
    def setup_scene(self):
        """Load scene with robot and objects"""
        from omni.isaac.core.utils.stage import add_reference_to_stage
        
        # Add robot
        add_reference_to_stage(
            usd_path="omniverse://localhost/NVIDIA/Assets/Robots/UR/ur10/ur10.usd",
            prim_path="/World/UR10"
        )
        
        # Add camera
        add_reference_to_stage(
            usd_path="omniverse://localhost/NVIDIA/Assets/Sensors/Camera/realsense_d435.usd",
            prim_path="/World/Camera"
        )
        
        self.world.reset()
    
    def randomize_scene(self):
        """Apply domain randomization"""
        import random
        
        # Randomize lighting
        light = self.world.scene.get_object("/World/Light")
        light.set_intensity(random.uniform(1000, 3000))
        
        # Randomize object poses
        from omni.isaac.core.utils.prims import get_prim_at_path
        object_prim = get_prim_at_path("/World/Objects")
        
        for child in object_prim.GetChildren():
            # Random rotation and translation
            position = [
                random.uniform(-1, 1),
                random.uniform(-1, 1),
                random.uniform(0.5, 1.5)
            ]
            object_prim.GetAttribute("xformOp:translate").Set(position)
    
    def capture_frame(self, camera_path="/World/Camera"):
        """Capture RGB and depth"""
        from omni.isaac.sensor import Camera
        
        camera = Camera(prim_path=camera_path)
        
        # Get RGB
        rgb = camera.get_rgb()
        
        # Get depth
        depth = camera.get_depth()
        
        return rgb, depth
    
    def get_instance_segmentation(self, camera_path="/World/Camera"):
        """Get semantic segmentation"""
        from omni.isaac.sensor import Camera
        
        camera = Camera(prim_path=camera_path)
        segmentation = camera.get_instance_segmentation()
        return segmentation
    
    def get_bounding_boxes(self, camera_path="/World/Camera"):
        """Get 2D bounding boxes"""
        from omni.isaac.sensor import Camera
        
        camera = Camera(prim_path=camera_path)
        bboxes = camera.get_bounding_box_2d()
        return bboxes
    
    def generate_dataset(self, num_frames=1000):
        """Generate full dataset"""
        self.setup_scene()
        
        annotations = []
        
        for frame_idx in range(num_frames):
            # Randomize scene
            self.randomize_scene()
            
            # Step simulation
            self.world.step(render=False)
            
            # Capture data
            rgb, depth = self.capture_frame()
            segmentation = self.get_instance_segmentation()
            bboxes = self.get_bounding_boxes()
            
            # Save frame
            frame_name = f"frame_{frame_idx:06d}"
            self.save_frame(frame_name, rgb, depth, segmentation)
            
            # Record annotation
            annotation = {
                "frame_id": frame_idx,
                "rgb": f"{frame_name}_rgb.png",
                "depth": f"{frame_name}_depth.npy",
                "segmentation": f"{frame_name}_seg.npy",
                "bboxes": bboxes.tolist(),
                "timestamp": self.world.current_time
            }
            annotations.append(annotation)
            
            if (frame_idx + 1) % 100 == 0:
                print(f"Generated {frame_idx + 1}/{num_frames} frames")
        
        # Save annotations
        with open(self.output_dir / "annotations.json", 'w') as f:
            json.dump(annotations, f, indent=2)
        
        self.simulation_app.close()
    
    def save_frame(self, frame_name, rgb, depth, segmentation):
        """Save frame data"""
        import cv2
        
        # RGB
        rgb_path = self.output_dir / f"{frame_name}_rgb.png"
        cv2.imwrite(str(rgb_path), rgb)
        
        # Depth
        depth_path = self.output_dir / f"{frame_name}_depth.npy"
        np.save(depth_path, depth)
        
        # Segmentation
        seg_path = self.output_dir / f"{frame_name}_seg.npy"
        np.save(seg_path, segmentation)

# Usage
if __name__ == '__main__':
    generator = SyntheticDataGenerator()
    generator.generate_dataset(num_frames=1000)
```

## COCO Format Conversion

Convert to COCO for compatibility with popular ML frameworks:

```python
def convert_to_coco_format(annotations_file):
    """Convert Isaac Sim annotations to COCO format"""
    
    coco_data = {
        "info": {
            "description": "Isaac Sim Synthetic Dataset",
            "version": "1.0",
            "year": 2024
        },
        "licenses": [],
        "images": [],
        "annotations": [],
        "categories": []
    }
    
    # Define categories
    categories = {
        0: "background",
        1: "object_1",
        2: "object_2"
    }
    
    for cat_id, cat_name in categories.items():
        coco_data["categories"].append({
            "id": cat_id,
            "name": cat_name
        })
    
    # Load Isaac annotations
    with open(annotations_file, 'r') as f:
        isaac_annotations = json.load(f)
    
    annotation_id = 0
    
    for frame_data in isaac_annotations:
        image_id = frame_data["frame_id"]
        
        coco_data["images"].append({
            "id": image_id,
            "file_name": frame_data["rgb"],
            "height": 480,
            "width": 640
        })
        
        # Convert bboxes
        for bbox in frame_data["bboxes"]:
            coco_data["annotations"].append({
                "id": annotation_id,
                "image_id": image_id,
                "category_id": 1,  # Object class
                "bbox": bbox,  # [x, y, width, height]
                "area": bbox[2] * bbox[3],
                "iscrowd": 0
            })
            annotation_id += 1
    
    # Save COCO format
    with open('dataset_coco.json', 'w') as f:
        json.dump(coco_data, f)
    
    return coco_data
```

## Domain Randomization Parameters

**Common Randomization:**
```python
class DomainRandomizer:
    def __init__(self):
        self.params = {
            # Lighting
            'light_intensity': (500, 3000),
            'light_color': [(0.8, 0.8, 1.0), (1.0, 0.9, 0.8)],
            
            # Camera
            'camera_noise_std': 0.001,
            'camera_blur': (0, 5),
            
            # Object
            'object_scale': (0.8, 1.2),
            'object_texture': ['metal', 'plastic', 'rubber'],
            
            # Environmental
            'background': ['warehouse', 'office', 'lab']
        }
    
    def randomize(self):
        import random
        randomized = {}
        for param, value_range in self.params.items():
            if isinstance(value_range[0], tuple):
                randomized[param] = random.choice(value_range)
            elif isinstance(value_range[0], (int, float)):
                randomized[param] = random.uniform(value_range[0], value_range[1])
        return randomized
```

## Performance Metrics

**Generation Speed:**
- 2000+ FPS headless (no rendering)
- ~100-500 frames per minute with full annotation capture
- 1000-frame dataset in ~5-10 minutes
