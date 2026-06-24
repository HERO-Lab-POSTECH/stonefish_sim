# stonefish_control_msgs

ROS2 message and service definitions for the Stonefish control system.

## Overview

This package provides custom message and service definitions used by the Stonefish control stack. It includes trajectory messages, waypoint definitions, guidance commands, and controller service interfaces.

**Package Type**: Message/Service definitions only (no nodes)

## Features

- Trajectory and waypoint data structures
- Guidance command messages for path following
- PID controller service interfaces
- Controller reset and hold services

## Message Definitions

### Core Messages

| Message | Description | Fields |
|---------|-------------|--------|
| `Trajectory.msg` | Complete trajectory with timestamped points | `header`, `points[]` |
| `TrajectoryPoint.msg` | Single trajectory point with pose, velocity, acceleration | `header`, `pose`, `velocity`, `acceleration` |
| `Waypoint.msg` | 4DOF waypoint with heading and speed constraints | `header`, `point`, `heading`, `max_forward_speed`, `use_fixed_heading`, `radius_of_acceptance` |
| `WaypointSet.msg` | Collection of waypoints with start time | `header`, `start_time`, `waypoints[]` |
| `GuidanceCommand.msg` | Guidance layer output for control layer | `header`, `desired_pose`, `desired_speed`, `cross_track_error`, `along_track_error`, `path_progress` |

### Message Details

#### Trajectory.msg

Container for a sequence of trajectory points.

```
std_msgs/Header header
stonefish_control_msgs/TrajectoryPoint[] points  # Array of trajectory samples
```

**Usage**: Path visualization, trajectory tracking

---

#### TrajectoryPoint.msg

Single point on a trajectory with full kinematic state.

```
std_msgs/Header header
geometry_msgs/Pose pose            # Position and orientation
geometry_msgs/Twist velocity       # Linear and angular velocity
geometry_msgs/Accel acceleration   # Linear and angular acceleration
```

**Usage**: Trajectory interpolation, feedforward control

---

#### Waypoint.msg

4DOF waypoint definition for marine vehicles (X, Y, Z, Yaw).

```
std_msgs/Header header
geometry_msgs/Point point          # Position [X, Y, Z] in world frame
float64 heading                    # Yaw angle in radians
float64 max_forward_speed          # Speed limit to this waypoint (m/s)
bool use_fixed_heading             # True: use specified heading, False: auto-calculate
float64 radius_of_acceptance       # Acceptance radius (meters)
```

**Coordinate Convention**:
- World frame: NED (North-East-Down)
- Position: `[X, Y, Z]` where Z > 0 = underwater
- Heading: Yaw angle in radians (0 = North, π/2 = East)

**Usage**: Path generation, waypoint navigation

---

#### WaypointSet.msg

Collection of waypoints for mission planning.

```
std_msgs/Header header
builtin_interfaces/Time start_time     # Mission start time
stonefish_control_msgs/Waypoint[] waypoints
```

**Usage**: Mission definition, path generator input

---

#### GuidanceCommand.msg

Output from guidance layer (e.g., LOS guidance) to control layer.

```
std_msgs/Header header

# Desired pose (4DOF for marine vehicles: x, y, z, yaw)
geometry_msgs/Pose desired_pose

# Desired forward speed (m/s)
float64 desired_speed

# Cross-track error (m)
# Positive: vehicle is to the right of the path
# Negative: vehicle is to the left of the path
float64 cross_track_error

# Along-track error (m)
# Positive: vehicle is ahead of the virtual target
# Negative: vehicle is behind the virtual target
float64 along_track_error

# Path progress parameter (0.0 to 1.0)
float64 path_progress
```

**Usage**: Path following, guidance-control interface

**Typical Flow**:
```
Path Following Node (LOS Guidance)
  ↓ publishes GuidanceCommand
Controller (Position/Hybrid)
  ↓ subscribes and generates wrench
Thruster Manager
```

---

## Service Definitions

### Controller Services

| Service | Description | Request | Response |
|---------|-------------|---------|----------|
| `ResetTrajectory.srv` | Reset trajectory following | (empty) | `success`, `message` |

### Service Details

#### ResetTrajectory.srv

Reset trajectory following state (used by path following nodes).

**Request**: Empty

**Response**:
```
bool success
string message  # Status message
```

**Example**:
```bash
ros2 service call /path_following_node/reset_trajectory stonefish_control_msgs/srv/ResetTrajectory
```

---

## Installation

### Build from Source

```bash
cd /workspace/colcon_ws
colcon build --packages-select stonefish_control_msgs
source install/setup.bash
```

### Dependencies

**ROS2 Packages**:
- `std_msgs`
- `geometry_msgs`
- `builtin_interfaces`

**Build Tools**:
- `rosidl_default_generators`

---

## Usage

### In Python

```python
from stonefish_control_msgs.msg import Waypoint, WaypointSet, GuidanceCommand

# Create waypoint
waypoint = Waypoint()
waypoint.point.x = 5.0
waypoint.point.y = 0.0
waypoint.point.z = 2.0  # 2m underwater (NED convention)
waypoint.heading = 0.0
waypoint.use_fixed_heading = False
waypoint.max_forward_speed = 1.0
waypoint.radius_of_acceptance = 0.5

# Create guidance command
cmd = GuidanceCommand()
cmd.desired_pose.position.x = 10.0
cmd.desired_speed = 1.5
cmd.cross_track_error = 0.2
cmd.path_progress = 0.5  # 50% complete
```

### In C++

```cpp
#include "stonefish_control_msgs/msg/waypoint.hpp"

// Create waypoint
auto waypoint = stonefish_control_msgs::msg::Waypoint();
waypoint.point.x = 5.0;
waypoint.point.y = 0.0;
waypoint.point.z = 2.0;
waypoint.heading = 0.0;
waypoint.use_fixed_heading = false;
waypoint.max_forward_speed = 1.0;
```

---

## Package Structure

```
stonefish_control_msgs/
├── msg/
│   ├── Trajectory.msg         # Trajectory container
│   ├── TrajectoryPoint.msg    # Single trajectory point
│   ├── Waypoint.msg           # 4DOF waypoint
│   ├── WaypointSet.msg        # Waypoint collection
│   └── GuidanceCommand.msg    # Guidance output
├── srv/
│   └── ResetTrajectory.srv    # Reset trajectory
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## Related Packages

- **stonefish_control**: Controllers using these messages
- **stonefish_trajectory_manager**: Path generation and following
- **stonefish_thruster_manager**: Thruster allocation

---

## Design Notes

### 4DOF Convention

All messages use **4DOF control** (X, Y, Z, Yaw) for underactuated marine vehicles:
- Roll and Pitch are **not controlled** (passive stability via buoyancy)
- Suitable for BlueROV2, most AUVs, and USVs
- Orientation in messages may include Roll/Pitch for state representation, but control acts only on Yaw

### Coordinate System

**NED (North-East-Down)** world frame:
- X-axis: North
- Y-axis: East
- Z-axis: Down (positive = underwater)

Example: Position `[0, 0, 2.0]` = 2 meters underwater at origin

---

## License

GPL-3.0

Based on UUV Simulator (Copyright 2016 The UUV Simulator Authors)
