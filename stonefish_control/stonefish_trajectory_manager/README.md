# stonefish_trajectory_manager

Path generation and following for marine vehicles in Stonefish simulator.

**Version**: 0.3.0
**Coordinate System**: NED (North-East-Down)
**Frame ID**: `world_ned`

---

## Overview

This package provides complete trajectory generation and path following capabilities for underwater vehicles. It includes multiple interpolation methods (linear, LIPB, cubic) and implements Line-of-Sight (LOS) guidance algorithms for path following.

**Key Capabilities**:
- **Path Generation**: Convert waypoints to smooth trajectories
- **Path Following**: LOS guidance for trajectory tracking
- **Velocity Profiling**: Automatic speed adjustment based on path curvature
- **RViz Integration**: Real-time path visualization

---

## Features

### Path Generation
- **Linear**: Piecewise linear segments (sharp corners)
- **LIPB**: Log-Interpolated Polynomial Bezier (smooth corners) ⭐ **Recommended**
- **Cubic**: Cubic spline interpolation (fully smooth curves)

### Path Following
- **Simple LOS Guidance**: Literature-based Line-of-Sight guidance (Fossen 2011)
- Cross-track error convergence
- Monotonic path parameter tracking
- Automatic velocity profiling

### Coordinate Systems
- **World Frame**: `world_ned` (NED: X=North, Y=East, Z=Down)
- **Body Frame**: `base_link` (FRD: X=Forward, Y=Right, Z=Down)
- **Static TF**: `world` (ENU for RViz) ↔ `world_ned` (NED for Stonefish)

---

## Installation

### Build from Source

```bash
cd /workspace/colcon_ws
colcon build --packages-select stonefish_trajectory_manager
source install/setup.bash
```

### Dependencies

**ROS2 Packages**:
- `geometry_msgs`
- `nav_msgs`
- `stonefish_control_msgs`
- `visualization_msgs`

**Python**:
- `numpy`
- `scipy`
- `pyyaml`

---

## Quick Start

### 1. Path Generator (Standalone)

Generate and visualize paths from waypoints:

```bash
ros2 launch stonefish_trajectory_manager path_generator.launch.py \
    waypoint_file:=/path/to/waypoints.yaml \
    interpolation_method:=lipb \
    sample_step:=0.01
```

**Published Topics**:
- `/path_generator_node/path` (nav_msgs/Path) - For path following + RViz
- `/path_generator_node/waypoint_markers` (MarkerArray) - Waypoint visualization

### 2. Path Following (with BlueROV2)

Follow generated paths using Simple LOS guidance:

```bash
ros2 launch stonefish_trajectory_manager path_following.launch.py \
    waypoint_file:=/path/to/waypoints.yaml \
    vehicle_name:=bluerov2 \
    lookahead_distance:=2.5 \
    acceptance_radius:=0.5
```

**Subscribes**: `/{vehicle_name}/odometry`
**Publishes**: `/{vehicle_name}/cmd_pose` (for position controller)

### 3. Complete Mission (Simulator + Path Following)

Run full system:

```bash
# Terminal 1: Simulator
ros2 launch stonefish_ros2 bluerov2.launch.py

# Terminal 2: Controller
ros2 launch stonefish_control controller.launch.py controller_type:=position

# Terminal 3: Path following
ros2 launch stonefish_trajectory_manager path_following.launch.py \
    waypoint_file:=config/example_waypoints.yaml \
    vehicle_name:=bluerov2
```

---

## System Architecture

```
┌─────────────────────┐
│ Path Generator Node │  YAML → Path interpolation
└──────────┬──────────┘
           │ publishes /path (nav_msgs/Path)
           ↓
┌──────────┴──────────────┐
│ Path Following Node     │  Simple LOS Guidance
│ (Simple LOS)            │
└──────────┬──────────────┘
           │ publishes cmd_pose
           ↓
    ┌──────────┐
    │ PID Ctrl │
    └──────────┘
```

---

## Interpolation Methods

| Method | LABEL | Description | Use Case |
|--------|-------|-------------|----------|
| Linear | `linear` | Piecewise linear, sharp corners | Simple missions, debugging |
| LIPB | `lipb` | Smooth Bezier corners | **General use (Recommended)** |
| Cubic | `cubic` | Cubic spline, fully smooth | Complex smooth trajectories |

**Parameters**:
- `sample_step`: Path resolution (default: 0.01)
  - Smaller (0.001): More points, smoother, more memory
  - Larger (0.1): Fewer points, less memory

---

## Waypoint File Format

```yaml
# config/example_waypoints.yaml
inertial_frame_id: 'world_ned'

waypoints:
  - position: [0.0, 0.0, 2.0]      # NED: underwater 2m
    use_fixed_heading: false        # Auto-calculate heading

  - position: [5.0, 0.0, 1.5]
    use_fixed_heading: false

  # Fixed heading example
  - position: [5.0, 5.0, 1.3]
    use_fixed_heading: true
    heading_deg: 45                 # Fixed heading in degrees
```

**Coordinate Convention**:
- `position: [X, Y, Z]` in NED
  - X: North (meters)
  - Y: East (meters)
  - Z: Down (meters, positive = underwater)

**Speed Control**:
- Speed is controlled globally via `robot_max_speed` parameter (not per-waypoint)
- LOS guidance automatically reduces speed at corners based on path curvature
- See [Velocity Profiling](#velocity-profiling) for details

---

## RViz Visualization

### Setup

1. Start Stonefish simulator
2. Launch path generator:
   ```bash
   ros2 launch stonefish_trajectory_manager path_generator.launch.py
   ```
3. Open RViz
4. **Fixed Frame**: `world` (keep as-is)
5. Add displays:
   - Path: `/path_generator_node/path`
   - MarkerArray: `/path_generator_node/waypoint_markers`

### Why Fixed Frame = 'world'?

- Stonefish uses NED (`world_ned`)
- RViz expects ENU (`world`)
- Static TF transform automatically converts: `world_ned` → `world`
- Result: Paths display correctly underwater ✅

---

## Parameters

### Path Generator

| Parameter | Default | Description |
|-----------|---------|-------------|
| `waypoint_file` | required | Path to YAML waypoint file |
| `interpolation_method` | `lipb` | Path interpolation: linear, lipb, cubic |
| `sample_step` | `0.01` | Path resolution (smaller = more points) |
| `publish_rate` | `1.0` | Hz for visualization topics |

### Path Following

| Parameter | Default | Description |
|-----------|---------|-------------|
| `waypoint_file` | required | Path to YAML waypoint file |
| `vehicle_name` | `bluerov2` | Vehicle namespace |
| `lookahead_distance` | `2.5` | LOS lookahead distance (m) |
| `acceptance_radius` | `0.5` | Waypoint acceptance radius (m) |
| `robot_max_speed` | `1.0` | Maximum robot speed (m/s) |
| `max_lateral_accel` | `0.3` | Maximum lateral acceleration (m/s²) |
| `min_speed_factor` | `0.3` | Minimum speed as factor of max (0-1) |
| `update_rate` | `20.0` | Guidance update rate (Hz) |

---

## Velocity Profiling

The LOS path following system includes **automatic velocity profiling** based on path curvature:

### How It Works

1. **Curvature Estimation**: Uses 3-point Menger curvature method at waypoints
   ```
   κ = 4 × Area(triangle) / (|P1P2| × |P2P3| × |P3P1|)
   ```

2. **Safe Speed Calculation**: Based on lateral acceleration constraint
   ```
   v_safe = sqrt(a_lateral_max / κ)
   ```

3. **Speed Constraints**:
   ```
   v_final = max(v_min, min(v_safe, robot_max_speed))
   ```
   where `v_min = robot_max_speed × min_speed_factor`

### Example

```bash
ros2 launch stonefish_trajectory_manager path_following.launch.py \
    waypoint_file:=config/example_square_path.yaml \
    robot_max_speed:=1.5 \
    max_lateral_accel:=0.3 \
    min_speed_factor:=0.3
```

**Result**:
- Straight segments: 1.5 m/s (max speed)
- 90° corners: ~0.6 m/s (automatically reduced based on curvature)
- Never below: 0.45 m/s (30% of max speed)

### Parameters Tuning

| Scenario | `robot_max_speed` | `max_lateral_accel` | `min_speed_factor` |
|----------|-------------------|---------------------|--------------------|
| Slow & Safe | 0.5 m/s | 0.2 m/s² | 0.5 |
| **Default (Recommended)** | **1.0 m/s** | **0.3 m/s²** | **0.3** |
| Fast & Aggressive | 1.5 m/s | 0.5 m/s² | 0.2 |

---

## API Usage

### Python API

```python
from stonefish_trajectory_manager.common import WaypointSet, WPTrajectoryGenerator
from stonefish_trajectory_manager.path_generator import LIPBInterpolator
from stonefish_trajectory_manager.path_following import SimpleLOSGuidance

# Load waypoints
waypoints = WaypointSet()
waypoints.read_from_file('waypoints.yaml')

# Generate path
traj_gen = WPTrajectoryGenerator(interpolation_method='lipb')
traj_gen.init_waypoints(waypoints)
traj_gen.interpolator.init_interpolator()

# Get samples
path_points = traj_gen.get_samples(step=0.01)

# Path following
guidance = SimpleLOSGuidance(
    lookahead_distance=2.5,
    robot_max_speed=1.0,
    max_lateral_accel=0.3
)
guidance.set_waypoints(waypoint_list)
guidance.update(dt=0.05)
cmd = guidance.get_guidance_command()
```

---

## File Structure

```
stonefish_trajectory_manager/
├── common/              # Common data structures
├── path_generator/      # Path interpolation algorithms
├── path_following/      # Path following algorithms (Simple LOS)
├── nodes/               # ROS2 nodes
│   ├── path_generator_node.py
│   ├── path_following_node.py
│   └── _legacy/         # Deprecated nodes
├── launch/              # Launch files
│   ├── path_generator.launch.py
│   └── path_following.launch.py
└── config/              # Example waypoint files
```

---

## Launch Files

### path_generator.launch.py

Standalone path generator for visualization and testing.

**Parameters**:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `waypoint_file` | string | (required) | Path to waypoint YAML file |
| `interpolation_method` | string | `lipb` | Interpolation: `linear`, `lipb`, `cubic` |
| `sample_step` | float | `0.01` | Path resolution (smaller = more points) |
| `publish_rate` | float | `1.0` | Visualization update rate (Hz) |

**Example**:
```bash
ros2 launch stonefish_trajectory_manager path_generator.launch.py \
    waypoint_file:=/workspace/config/mission1.yaml \
    interpolation_method:=cubic \
    sample_step:=0.005
```

---

### path_following.launch.py

LOS guidance-based path following.

**Parameters**:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `waypoint_file` | string | (required) | Path to waypoint YAML file |
| `vehicle_name` | string | `bluerov2` | Vehicle namespace |
| `interpolation_method` | string | `lipb` | Path interpolation method |
| `lookahead_distance` | float | `2.5` | LOS lookahead distance (m) |
| `acceptance_radius` | float | `0.5` | Waypoint acceptance radius (m) |
| `robot_max_speed` | float | `1.0` | Maximum speed (m/s) |
| `max_lateral_accel` | float | `0.3` | Max lateral acceleration (m/s²) |
| `min_speed_factor` | float | `0.3` | Min speed as factor of max (0-1) |
| `update_rate` | float | `20.0` | Guidance update rate (Hz) |

**Example**:
```bash
ros2 launch stonefish_trajectory_manager path_following.launch.py \
    waypoint_file:=config/square_path.yaml \
    vehicle_name:=bluerov2 \
    robot_max_speed:=1.5 \
    lookahead_distance:=3.0
```

---

## Topics

### Path Generator Node

#### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/path_generator_node/path` | `nav_msgs/Path` | Generated path for visualization and following |
| `/path_generator_node/waypoint_markers` | `visualization_msgs/MarkerArray` | Waypoint visualization markers |

---

### Path Following Node

#### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/{vehicle_name}/odometry` | `nav_msgs/Odometry` | Vehicle state feedback |

#### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/{vehicle_name}/cmd_pose` | `geometry_msgs/PoseStamped` | Desired pose for position controller |
| `/{vehicle_name}/guidance_command` | `stonefish_control_msgs/GuidanceCommand` | Full guidance state (pose, speed, errors) |
| `/path_following_node/path` | `nav_msgs/Path` | Current path being followed |
| `/path_following_node/current_waypoint` | `visualization_msgs/Marker` | Active waypoint marker |

---

## Services

### Path Following Node

| Service | Type | Description |
|---------|------|-------------|
| `/path_following_node/reset_trajectory` | `stonefish_control_msgs/ResetTrajectory` | Reset path following state |

**Example**:
```bash
ros2 service call /path_following_node/reset_trajectory \
    stonefish_control_msgs/srv/ResetTrajectory
```

---

## Related Packages

- **stonefish_control**: Position and hybrid controllers for path following
- **stonefish_control_msgs**: Message definitions (Waypoint, GuidanceCommand, etc.)
- **stonefish_ros2**: Stonefish simulator integration

---

## References

### Path Following
- Fossen, T. I. (2011). "Handbook of Marine Craft Hydrodynamics and Motion Control"
  - Chapter 10: Path Following
  - Section 10.4: Line-of-Sight Guidance
- Lekkas, A. M., & Fossen, T. I. (2014). "Line-of-Sight Guidance for Path Following of Marine Vehicles"

### Path Generation
- Based on UUV Simulator framework (Apache 2.0 license)
- LIPB: Log-Interpolated Polynomial Bezier curves
- Cubic Spline: Hermite spline interpolation

---

## License

Apache 2.0

Based on UUV Simulator (Copyright 2016 The UUV Simulator Authors)

---

## Notes

### DOF (Degrees of Freedom)

**This package generates 4DOF paths only** (X, Y, Z, Yaw):
- ✅ Roll = 0, Pitch = 0 (fixed)
- ✅ Designed for underactuated vehicles (BlueROV2)
- ✅ Roll and pitch determined by vehicle dynamics

**Important**:
- Path generator creates orientation (heading) for visualization
- **Path following (LOS) ignores orientation** and calculates heading independently
- This separation follows the "Separation of Concerns" principle

### Velocity Profiler (Optional)

Automatically reduces speed in sharp corners based on path curvature.

**Enable**:
```bash
ros2 launch ... --ros-args -p use_velocity_profiler:=true
```

**Parameters**:
- `max_lateral_accel`: Maximum lateral acceleration (m/s²)
- `speed_reduction_factor`: Corner speed reduction (0-1)

---

## Troubleshooting

### Path appears above water in RViz

**Solution**: Ensure RViz Fixed Frame = `world` (not `world_ned`)
- The static TF transform handles NED↔ENU conversion automatically

### Waypoints not loading

**Check**:
1. YAML file has `inertial_frame_id: 'world_ned'`
2. Position values use NED convention (Z+ = underwater)

### Path too jagged

**Solution**: Reduce `sample_step` parameter
```bash
sample_step:=0.005  # More points, smoother
```

---

For more details, see `ARCHITECTURE.md`
