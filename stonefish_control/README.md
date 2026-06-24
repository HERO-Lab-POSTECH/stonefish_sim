# Stonefish Control - ROS2 Control Packages for Marine Robotics

ROS2-based control system for underwater vehicles (UUVs) and surface vehicles, ported from UUV Simulator and adapted for the Stonefish marine robotics simulator.

## Overview

This meta-package provides a complete control stack for marine robotics simulation, including thrust allocation, trajectory generation, path following, and various control algorithms. It follows a modular architecture where each sub-package handles a specific aspect of the control pipeline.

## Package List

### Core Packages (Production-Ready)

#### 1. **stonefish_control_msgs**
Message and service definitions for control systems.

**Contents**:
- 4 messages: `Waypoint`, `WaypointSet`, `Trajectory`, `TrajectoryPoint`
- 22 services: Controller configuration, waypoint management, trajectory generation

**Location**: `stonefish_control_msgs/`

#### 2. **stonefish_thruster_manager**
Thrust allocation using TAM (Thruster Allocation Matrix).

**Features**:
- Converts 6DOF wrench to individual thruster forces
- Supports multiple thruster configurations (BlueROV2, Girona500, etc.)
- Thruster models: Proportional, Custom
- TAM-based least-squares allocation

**Location**: `stonefish_thruster_manager/`

#### 3. **stonefish_trajectory_manager**
Path generation and following with LOS guidance.

**Features**:
- Waypoint-based path generation
- Interpolation methods: Linear, Cubic, LIPB (Log-Interpolated Polynomial Bezier)
- LOS (Line-of-Sight) guidance law
- Automatic velocity profiling based on curvature
- 4DOF paths (X, Y, Z, Yaw)

**Location**: `stonefish_trajectory_manager/`

### Development Packages

#### 4. **stonefish_control**
Main control algorithms package.

**Planned Controllers**:
- PID controllers (standard, nonlinear, underactuated)
- Cascaded control (position → velocity → acceleration)
- Advanced control (Sliding Mode, Feedback Linearization, Model Predictive Control)

**Current Status**: Package structure complete, controllers under development

**Location**: `stonefish_control/`

## Quick Start

### 1. Build

```bash
cd /workspace/colcon_ws

# Build all control packages
colcon build --packages-select \
    stonefish_control_msgs \
    stonefish_thruster_manager \
    stonefish_trajectory_manager \
    stonefish_control

# Or build individually
colcon build --packages-select stonefish_thruster_manager
```

### 2. Source Workspace

```bash
source /workspace/colcon_ws/install/setup.bash
```

### 3. Launch Simulation with Control

#### Basic Simulation with Thruster Manager

```bash
# BlueROV2 with thruster manager (included by default)
ros2 launch stonefish_ros2 bluerov2.launch.py
```

#### Thruster Manager Only

```bash
# Launch thruster manager separately
ros2 launch stonefish_thruster_manager thruster_manager.launch.py \
    vehicle_name:=bluerov2

# Custom TAM file
ros2 launch stonefish_thruster_manager thruster_manager.launch.py \
    vehicle_name:=bluerov2 \
    tam_file:=/path/to/custom/TAM.yaml \
    max_thrust:=150.0 \
    timeout:=1.0
```

#### Path Following

```bash
# Generate and visualize path
ros2 launch stonefish_trajectory_manager path_generator.launch.py \
    waypoint_file:=/workspace/colcon_ws/src/stonefish_control/stonefish_trajectory_manager/config/example_waypoints.yaml \
    interpolation_method:=lipb

# Path following with LOS guidance
ros2 launch stonefish_trajectory_manager path_following.launch.py \
    waypoint_file:=/workspace/colcon_ws/src/stonefish_control/stonefish_trajectory_manager/config/example_waypoints.yaml \
    vehicle_name:=bluerov2 \
    lookahead_distance:=2.5 \
    robot_max_speed:=1.0
```

### 4. Send Control Commands

```bash
# Send wrench command (10N forward, 5N⋅m yaw)
ros2 topic pub /bluerov2/thruster_manager/input geometry_msgs/msg/Wrench \
    "{force: {x: 10.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 5.0}}" \
    --once

# Check thruster outputs
ros2 topic echo /bluerov2/setpoint/pwm

# Monitor odometry
ros2 topic echo /bluerov2/odometry
```

## Testing

### TAM (Thruster Allocation Matrix) Testing

#### Python API Test

```python
from stonefish_thruster_manager.thruster_manager import ThrusterManager
import numpy as np

# Load TAM
tam_mgr = ThrusterManager(
    tam_file_path='/workspace/colcon_ws/src/stonefish_description/data/robots/bluerov2/config/TAM.yaml'
)

# Wrench → Thrust conversion
wrench = np.array([10, 0, 20, 0, 0, 5])  # [Fx, Fy, Fz, Tx, Ty, Tz]
thrust_forces = tam_mgr.compute_thrust_forces(wrench)
print(f"Thrust forces: {thrust_forces}")

# Verify: Thrust → Wrench
wrench_check = tam_mgr.compute_wrench(thrust_forces)
print(f"Recovered wrench: {wrench_check}")
```

**Expected Output**:
```
Thrust forces: [3.536 3.536 3.536 3.536 5.0 5.0 5.0 5.0]
Recovered wrench: [10.  0. 20.  0.  0.  5.]
```

#### ROS2 Runtime Test

```bash
# Start simulator with thruster manager
ros2 launch stonefish_ros2 bluerov2.launch.py

# Send test wrench (in another terminal)
ros2 topic pub /bluerov2/thruster_manager/input geometry_msgs/msg/Wrench \
    "{force: {x: 10.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}" \
    --once

# Monitor thruster outputs
ros2 topic echo /bluerov2/setpoint/pwm
```

### Path Following Test

```bash
# Terminal 1: Start simulation
ros2 launch stonefish_ros2 bluerov2.launch.py

# Terminal 2: Launch path following
ros2 launch stonefish_trajectory_manager path_following.launch.py \
    vehicle_name:=bluerov2

# Monitor progress
ros2 topic echo /bluerov2/odometry
ros2 topic echo /bluerov2/path_following/current_waypoint
```

## Architecture

### Control System Flow

```
User Command (Waypoint/Pose/Wrench)
    ↓
Controller (PID/Cascaded/Hybrid)
    ↓ 6DOF Wrench: [Fx, Fy, Fz, Tx, Ty, Tz]
Thruster Allocator (TAM-based)
    ↓ Thrust Array: [F1, F2, ..., FN]
Stonefish Simulator
    ↓ State Feedback
Back to Controller
```

### TAM (Thruster Allocation Matrix)

#### BlueROV2 Configuration

BlueROV2 uses 8 thrusters:
- **Horizontal Thrusters (T1-T4)**: 45° angle, control Surge/Sway/Yaw
- **Vertical Thrusters (T5-T8)**: Downward, control Heave/Roll/Pitch

**TAM File Location**:
```
/workspace/colcon_ws/src/stonefish_description/data/robots/bluerov2/config/TAM.yaml
```

#### TAM Formula

**Forward Mapping** (Thrust → Wrench):
```
[Fx, Fy, Fz, Tx, Ty, Tz]ᵀ = TAM × [F1, F2, F3, F4, F5, F6, F7, F8]ᵀ
```

**Inverse Mapping** (Wrench → Thrust):
```
[F1, F2, ..., F8]ᵀ = pinv(TAM) × [Fx, Fy, Fz, Tx, Ty, Tz]ᵀ
```

Where `pinv(TAM)` is the Moore-Penrose pseudo-inverse (least-squares solution).

#### Adding New Robots

1. **Create TAM Configuration Directory**:
```bash
mkdir -p /workspace/colcon_ws/src/stonefish_description/data/robots/my_robot/config
```

2. **Write TAM.yaml**:
```yaml
tam:
  # 6 rows (DOF) x N columns (thrusters)
  - [t1_fx, t2_fx, ..., tN_fx]  # Row 0: X force contribution
  - [t1_fy, t2_fy, ..., tN_fy]  # Row 1: Y force contribution
  - [t1_fz, t2_fz, ..., tN_fz]  # Row 2: Z force contribution
  - [t1_tx, t2_tx, ..., tN_tx]  # Row 3: Roll torque contribution
  - [t1_ty, t2_ty, ..., tN_ty]  # Row 4: Pitch torque contribution
  - [t1_tz, t2_tz, ..., tN_tz]  # Row 5: Yaw torque contribution
```

**Example** (4-thruster configuration):
```yaml
tam:
  - [0.707, 0.707, -0.707, -0.707]  # Fx (surge)
  - [0.707, -0.707, -0.707, 0.707]  # Fy (sway)
  - [0.0, 0.0, 0.0, 0.0]            # Fz (heave, no vertical thrusters)
  - [0.0, 0.0, 0.0, 0.0]            # Tx (roll)
  - [0.0, 0.0, 0.0, 0.0]            # Ty (pitch)
  - [0.2, -0.2, 0.2, -0.2]          # Tz (yaw)
```

3. **Launch Thruster Manager**:
```bash
ros2 launch stonefish_thruster_manager thruster_manager.launch.py \
    vehicle_name:=my_robot \
    tam_file:=/workspace/colcon_ws/src/stonefish_description/data/robots/my_robot/config/TAM.yaml
```

## Topics

### Thruster Manager

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/{vehicle_name}/thruster_manager/input` | `geometry_msgs/Wrench` | Wrench command (6DOF) |
| `/{vehicle_name}/thruster_manager/input_stamped` | `geometry_msgs/WrenchStamped` | Timestamped wrench command |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/{vehicle_name}/setpoint/pwm` | `std_msgs/Float64MultiArray` | Thruster PWM commands [-1.0, 1.0] |

### Path Following (Trajectory Manager)

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/{vehicle_name}/odometry` | `nav_msgs/Odometry` | Robot state feedback |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/{vehicle_name}/reference/trajectory` | `nav_msgs/Path` | Generated trajectory path |
| `/{vehicle_name}/path_following/current_waypoint` | `geometry_msgs/PoseStamped` | Current target waypoint |
| `/{vehicle_name}/cmd_vel` | `geometry_msgs/Twist` | Velocity command (LOS guidance output) |

### Controllers (Future)

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/{vehicle_name}/odometry` | `nav_msgs/Odometry` | Robot state |
| `/{vehicle_name}/cmd_pose` | `geometry_msgs/PoseStamped` | Position setpoint |
| `/{vehicle_name}/reference/trajectory` | `stonefish_control_msgs/Trajectory` | Trajectory reference |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/{vehicle_name}/thruster_manager/input` | `geometry_msgs/Wrench` | Control wrench output |

## Development Status

### Completed Components
- ✅ ROS2 message/service system
- ✅ TAM-based thrust allocation
- ✅ Thruster models (proportional, custom)
- ✅ Path generation (Linear, Cubic, LIPB)
- ✅ LOS guidance law
- ✅ Package structure and build system

### In Progress
- ⏳ Control interfaces (vehicle dynamics, controller base classes)
- ⏳ PID controllers (standard, nonlinear, underactuated)
- ⏳ Cascaded control (position → velocity → acceleration)
- ⏳ Teleoperation interfaces

### Roadmap
1. Complete control interface base classes
2. Implement basic PID controllers
3. Add cascaded control architecture
4. Integrate trajectory manager with controllers
5. Full-stack integration testing

## Package-Specific Documentation

Individual packages have their own detailed READMEs:

- **stonefish_thruster_manager**: See `stonefish_thruster_manager/README.md` (pending)
- **stonefish_trajectory_manager**: See `stonefish_trajectory_manager/README.md` ✅
- **stonefish_control**: See `stonefish_control/README.md` (pending)

## Related Packages

- **stonefish_ros2**: Core simulator interface
- **stonefish_msgs**: Basic sensor/actuator messages
- **stonefish_description**: Robot models and TAM configurations
- **stonefish_slam**: SLAM using DVL/sonar data

## References

### Documentation

- **Stonefish Simulator**: https://stonefish.readthedocs.io
- **ROS2 Humble**: https://docs.ros.org/en/humble/
- **UUV Simulator** (original control system): https://github.com/uuvsimulator/uuv_simulator
- **Fossen's Handbook**: "Handbook of Marine Craft Hydrodynamics and Motion Control" (2011)

### Key Papers

- **LOS Guidance**: Fossen, T. I. (2011). "Handbook of Marine Craft Hydrodynamics and Motion Control"
- **TAM Allocation**: Fossen, T. I., & Johansen, T. A. (2006). "A Survey of Control Allocation Methods for Ships and Underwater Vehicles"

## License

**Apache License 2.0**

Original UUV Simulator code:
- Copyright (c) 2016-2019 The UUV Simulator Authors
- Licensed under Apache License 2.0

ROS2 port and enhancements:
- Copyright 2025 HERO Lab, POSTECH
- Licensed under Apache License 2.0

---

**Current Status**: Core infrastructure complete, control algorithms under development
