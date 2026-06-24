# Stonefish Trajectory Manager - Architecture

**Version**: 0.2.0
**Last Updated**: 2025-11-12

---

## Overview

This package provides **path generation** and **path following** capabilities for marine vehicles in Stonefish simulator. It implements a clean separation between path planning and guidance layers.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Path Generation Layer                     │
└─────────────────────────────────────────────────────────────┘
                               ↓
                    ┌──────────────────────┐
                    │  path_generator_node │
                    │                      │
                    │  Waypoints (YAML)    │
                    │         ↓            │
                    │  Interpolator        │
                    │  (linear/lipb/cubic) │
                    │         ↓            │
                    │  nav_msgs/Path       │
                    └──────────┬───────────┘
                               │ /path topic
                               ↓
┌─────────────────────────────────────────────────────────────┐
│                   Path Following Layer                       │
└─────────────────────────────────────────────────────────────┘
                               ↓
                    ┌──────────────────────┐
                    │ path_following_node  │
                    │                      │
                    │  Simple LOS Guidance │
                    │  (Fossen 2011)       │
                    │         ↓            │
                    │  cmd_pose            │
                    └──────────┬───────────┘
                               │
                               ↓
                    ┌──────────────────────┐
                    │   PID Controller     │
                    └──────────────────────┘
```

---

## Module Structure

### 1. common/

**Common data structures and utilities** (UUV Simulator based)

```
common/
├── waypoint.py           # Individual waypoint data
├── waypoint_set.py       # Waypoint collection + YAML loading
├── trajectory_point.py   # Trajectory point (pos, vel, acc, orientation)
└── trajectory_generator.py  # Waypoint → Trajectory conversion
```

**Key Classes**:
- `Waypoint`: Stores position, speed, heading, acceptance radius
- `WaypointSet`: Manages waypoint collection, loads from YAML
- `TrajectoryPoint`: Represents a point on trajectory with full state
- `WPTrajectoryGenerator`: Generates trajectories using interpolators

### 2. path_generator/

**Path interpolation algorithms**

```
path_generator/
├── path_generator.py       # Base class for interpolators
├── linear_interpolator.py  # Linear interpolation (LABEL='linear')
├── lipb_interpolator.py    # LIPB interpolation (LABEL='lipb') ⭐
├── cs_interpolator.py      # Cubic spline (LABEL='cubic')
├── bezier_curve.py         # Bezier curve utilities
├── line_segment.py         # Line segment representation
└── velocity_profiler.py    # Curvature-based speed adjustment
```

**Interpolation Methods**:

| Method | Sharp Corners | Smooth Curves | Computation | Use Case |
|--------|---------------|---------------|-------------|----------|
| `linear` | Yes | No | Fast | Simple paths, debugging |
| `lipb` | No | Yes (corners) | Medium | **General use** ⭐ |
| `cubic` | No | Yes (entire path) | Medium | Complex smooth paths |

**Velocity Profiler** (Optional):
- Adjusts speed based on path curvature
- Reduces speed in sharp corners
- Maintains stability during turns
- Enable with `use_velocity_profiler: true`

### 3. path_following/

**Path following algorithms**

```
path_following/
└── simple_los_guidance.py  # Simple LOS guidance (Fossen 2011)
```

**Simple LOS Guidance**:
- Cross-track error minimization
- Lookahead-based heading control
- Monotonic path parameter tracking (prevents backwards motion)
- Literature-based implementation (Lekkas & Fossen 2014)

**Key Improvements** (vs. Complex LOS):
- ✅ Desired position: Closest point on path (not lookahead point)
- ✅ Velocity reference: Path tangent direction
- ✅ Path parameter: Monotonic advancement
- ✅ Simple and robust (250 lines vs 1305 lines)

### 4. nodes/

**ROS2 node implementations**

```
nodes/
├── path_generator_node.py   # ✅ Active: Path generation
├── path_following_node.py   # ✅ Active: Path following
├── utils.py                  # Shared utilities
└── _legacy/                  # Deprecated nodes
    ├── trajectory_publisher_node.py
    └── trajectory_follower_node.py
```

---

## Coordinate Systems

### NED (North-East-Down) - Stonefish World Frame

```
     N (X+)
     ↑
     |
W ← -+- → E (Y+)
     |
     ↓
     D (Z+, underwater)
```

**Frame ID**: `world_ned`

### FRD (Forward-Right-Down) - Vehicle Body Frame

```
   Forward (X+)
     ↑
     |
L ← -+- → R (Y+)
     |
     ↓
   Down (Z+)
```

**Frame ID**: `base_link`

### ENU (East-North-Up) - RViz Convention

```
     N (Y+)
     ↑
     |
W ← -+- → E (X+)
     |
     ↑
     U (Z+, above water)
```

**Frame ID**: `world` (RViz Fixed Frame)

### TF Transform

```
world (ENU) --[180° X rotation]--> world_ned (NED)
```

This allows RViz to properly display NED-based paths when Fixed Frame = 'world'.

---

## Design Principles

### 1. Separation of Concerns

```
Path Generation (offline)  ← Independent →  Path Following (runtime)
```

- **Path Generator**: Creates path from waypoints (independent of vehicle state)
- **Path Following**: Tracks path using current vehicle state
- **Communication**: Standard ROS topic (`nav_msgs/Path`)

### 2. Path Representation Agnostic

LOS guidance is independent of path type:
- Works with linear paths
- Works with LIPB paths
- Works with cubic spline paths
- **No modification needed** when changing path type

### 3. Literature-Based

All algorithms follow established marine robotics literature:
- LOS guidance: Fossen (2011), Lekkas & Fossen (2014)
- Path interpolation: UUV Simulator framework
- Coordinate systems: Marine robotics standard (NED/FRD)

---

## Data Flow

### Path Generation

```
YAML Waypoints
    ↓
WaypointSet.read_from_file()
    ↓
WPTrajectoryGenerator
    ↓
Interpolator (linear/lipb/cubic)
    ↓
[Optional] VelocityProfiler
    ↓
List[TrajectoryPoint]
    ↓
nav_msgs/Path topic
```

### Path Following

```
nav_msgs/Path (from generator)
    ↓
SimpleLOSGuidance.set_waypoints()
    ↓
Odometry input
    ↓
SimpleLOSGuidance.update(dt)
    ↓
- Closest point on path (desired position)
- LOS heading (desired yaw)
- Path tangent velocity (desired velocity)
    ↓
TrajectoryPoint (cmd_pose)
    ↓
PID Controller
```

---

## Key Algorithms

### LOS Guidance (Fossen 2011, Eq. 10.12)

```
χ_d = χ_p - arctan(y_e / Δ)

where:
  χ_d: desired heading
  χ_p: path tangent angle
  y_e: cross-track error
  Δ: lookahead distance
```

**Implementation**: `path_following/simple_los_guidance.py`

### Path Parameter Tracking

```python
# Monotonic advancement (prevent backwards motion)
current_s = projection_on_path / path_length
if current_s > path_parameter:
    path_parameter = current_s  # Only advance forward
```

**Benefit**: Prevents oscillation and backwards motion on dynamic paths

---

## Performance Characteristics

### Path Generation

| Method | Points (step=0.01) | Generation Time | Memory |
|--------|-------------------|-----------------| ------|
| linear | ~100 | <10ms | Low |
| lipb | ~100 | ~50ms | Medium |
| cubic | ~100 | ~50ms | Medium |

### Path Following

- **Update Rate**: 20 Hz (configurable)
- **Cross-track Error**: < 0.5m (typical)
- **Computational Cost**: Very low (~0.1ms per update)

---

## Future Enhancements

### Path Generator

1. **Dynamic Waypoint Updates** (planned, framework ready)
   - Service: `AddWaypoint`, `UpdatePath`
   - Topic: `waypoint_stream` for continuous updates
   - Real-time path regeneration

2. **Additional Interpolators**
   - Dubins path (minimum turning radius)
   - B-spline with obstacle avoidance

### Path Following

1. **Adaptive Lookahead**
   - Adjust lookahead based on cross-track error
   - Better performance in tight corners

2. **Integral LOS** (ILOS)
   - Compensate for ocean currents
   - Lekkas & Fossen (2014) extension

3. **Continuous Path Support**
   - Remove segment-based logic
   - Single continuous parametrization (s: 0→1)

---

## Migration from Legacy

### From trajectory_publisher → path_generator_node

```bash
# Old
ros2 run stonefish_trajectory_manager trajectory_publisher

# New
ros2 run stonefish_trajectory_manager path_generator_node
```

### From trajectory_follower → path_following_node

```bash
# Old
ros2 run stonefish_trajectory_manager trajectory_follower

# New
ros2 run stonefish_trajectory_manager path_following_node
```

**Key Changes**:
- Topics: `/trajectory` → `/path` (nav_msgs/Path)
- Frame: `world` → `world_ned` (NED explicit)
- Guidance: Complex LOS → Simple LOS (literature-based)

---

## Development

### Building

```bash
colcon build --packages-select stonefish_trajectory_manager
```

### Testing

```bash
# Test the path stack: generator + following (following loop requires a simulator)
ros2 launch stonefish_trajectory_manager path.launch.py
```

---

**For questions or issues, refer to inline code documentation or contact the development team.**
