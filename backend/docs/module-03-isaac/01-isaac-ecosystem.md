# Chapter 1: NVIDIA Isaac Sim Ecosystem

## What is Isaac Sim?

NVIDIA Isaac Sim is an advanced robotics simulator that provides:
- **Physically Accurate Simulation**: GPU-accelerated physics with PhysX
- **Synthetic Data Generation**: Create labeled datasets for ML training
- **ROS 2 Native Integration**: Direct ROS 2 topic/service publishing
- **USD Foundation**: Industry-standard Universal Scene Description format
- **Real-time Performance**: 10x+ faster than traditional simulators
- **Digital Twins**: Sim-to-real transfer with domain randomization

## Installation

**System Requirements:**
- NVIDIA GPU (RTX 3070 or better)
- 24GB+ VRAM recommended
- Ubuntu 22.04 LTS
- NVIDIA Driver 550+

**Install via pip:**
```bash
# Create virtual environment
python -m venv isaac_env
source isaac_env/bin/activate

# Install Isaac Sim
pip install isaacsim

# Install ROS 2 bridge
pip install isaac-ros-bridge
```

**Verify Installation:**
```bash
python -c "from isaacsim import SimulationApp; print('Isaac Sim installed!')"
```

## USD (Universal Scene Description)

USD is the foundation format for Isaac Sim scenes:

```python
from pxr import Usd, UsdGeom, UsdPhysics
from isaacsim import SimulationApp

# Create USD stage
stage = Usd.Stage.CreateNew("scene.usd")
root = stage.GetRootLayer()

# Create a cube
cube_path = "/World/Cube"
cube = UsdGeom.Cube.Define(stage, cube_path)
cube.GetSizeAttr().Set(1.0)

# Add physics
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(cube_path))

# Save
stage.GetRootLayer().Save()
```

## Isaac Sim Python API

**Basic Scene Setup:**
```python
from isaacsim import SimulationApp
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage

# Initialize simulation
simulation_app = SimulationApp({"headless": False})

# Create world
world = World(stage_units_in_meters=1.0)

# Add robot
add_reference_to_stage(usd_path="omniverse://localhost/NVIDIA/Assets/Robots/UR/ur10/ur10.usd",
                       prim_path="/World/UR10")

# Play simulation
world.play()

# Simulation loop
for i in range(1000):
    world.step(render=True)

simulation_app.close()
```

## Scene Composition

**Creating a Warehouse Scene:**
```python
from omni.isaac.core.utils.stage import add_reference_to_stage

# Add ground plane
add_reference_to_stage(
    usd_path="omniverse://localhost/NVIDIA/Assets/Environments/Simple_Warehouse/warehouse.usd",
    prim_path="/World/Warehouse"
)

# Add robot
add_reference_to_stage(
    usd_path="omniverse://localhost/NVIDIA/Assets/Robots/UR/ur10/ur10.usd",
    prim_path="/World/UR10"
)

# Add table
add_reference_to_stage(
    usd_path="omniverse://localhost/NVIDIA/Assets/Props/Furniture/DiningTable/DiningTable.usd",
    prim_path="/World/Table"
)
```

## Rendering and Visualization

**Capture Images:**
```python
from omni.isaac.synthetic_utils import SyntheticDataHelper

def capture_frame():
    viewport = omni.kit.viewport_legacy.get_default_viewport_window()
    # Render and save
    rgb_image = viewport.get_image()
    return rgb_image

# In simulation loop
for i in range(100):
    world.step(render=True)
    if i % 10 == 0:
        frame = capture_frame()
        # Process frame
```

## Key Differences from Gazebo

| Feature | Gazebo | Isaac Sim |
|---------|--------|-----------|
| **Physics Engine** | ODE/Bullet | PhysX (GPU) |
| **Rendering** | Ignition | Nvidia RTX |
| **Scene Format** | URDF/SDF | USD |
| **Speed** | Real-time | 10x+ faster |
| **Synthetic Data** | Limited | Native support |
| **ROS 2** | Native | Native |
| **GPU Acceleration** | No | Yes |
| **Learning Curve** | Easy | Medium |

## Performance Metrics

**Typical Performance:**
- 2000-5000 FPS with headless simulation
- 200-500 FPS with rendering
- 10+ parallel simulations in parallel domains
