# stonefish_control

PID-based controllers for underwater vehicles in the Stonefish simulator.

## Overview

This package provides 4DOF (X, Y, Z, Yaw) controllers for underactuated marine vehicles such as BlueROV2. Controllers are designed for station-keeping, waypoint navigation, and path following with passive roll/pitch stability.

**Key Features**:
- **Position Controller**: Precise position hold with anti-windup PID
- **Hybrid Controller**: Switches between velocity and position modes
- **Velocity Controller**: Fast velocity tracking for path following
- **Feedforward support**: Optional velocity feedforward from path planners
- **Anti-windup**: Back-calculation method prevents integral windup

## Architecture

```
Waypoint/Setpoint
    ↓
Controller (Position/Hybrid/Velocity)
    ↓ publishes geometry_msgs/WrenchStamped
Thruster Manager (TAM allocation)
    ↓ publishes std_msgs/Float64MultiArray
Stonefish Simulator
```

**Input**: Desired pose and optional velocity
**Output**: 6DOF wrench (forces + torques) in body frame
**Feedback**: Vehicle odometry (position + velocity)

---

## Installation

### Build from Source

```bash
cd /workspace/colcon_ws
colcon build --packages-select stonefish_control
source install/setup.bash
```

### Dependencies

**ROS2 Packages**:
- `geometry_msgs`
- `nav_msgs`
- `stonefish_control_msgs`
- `stonefish_description` (for dynamics parameters)

**Python**:
- `numpy`
- `scipy`

---

## Controllers

### 1. Position Controller

4DOF PID controller for precise position hold and waypoint tracking.

**Control Law**:
```
τ = Kp·e + Kd·(-v) + Ki·∫e + M·v_ff
```

Where:
- `e` = Position error (world frame → body frame)
- `v` = Current velocity (body frame)
- `v_ff` = Feedforward velocity (optional)
- `M` = Mass matrix (for feedforward scaling)

**Features**:
- Anti-windup with back-calculation method
- Automatic integral limit calculation
- Feedforward velocity support
- Saturation on forces and torques

**Use Cases**:
- Station-keeping
- Waypoint navigation
- Precise positioning tasks

---

### 2. Hybrid Controller

Combines velocity and position controllers with mode switching.

**Modes**:
- **Velocity Mode**: Fast path following (accepts drift, high responsiveness)
- **Position Mode**: Precise position hold (zero drift, high stability)

**Mode Switching**:
- Controlled by `set_mode()` service or parameter
- Automatic reset of integral terms on mode switch
- Typical usage: Velocity mode during path following → Position mode at waypoint arrival

**Features**:
- Independent PID gains for each mode
- Separate saturation limits per mode
- Bumpless transfer on mode switch

**Use Cases**:
- Path following missions (switch at waypoints)
- Dynamic tasks requiring both speed and precision

---

### 3. Velocity Controller

PID controller optimized for velocity tracking.

**Control Law**:
```
τ = Kp·e_vel + Kd·a + Ki·∫e_vel
```

Where:
- `e_vel` = Velocity error
- `a` = Acceleration (numerical derivative)

**Features**:
- High gains for fast response
- Lower integral safety factor (prevents windup during aggressive maneuvers)
- Suitable for path following with LOS guidance

**Use Cases**:
- Fast path tracking
- Velocity-based teleoperation
- Dynamic maneuvers

---

## Usage

### Launch Files

#### controller.launch.py

Main launch file for all controller types.

**Parameters**:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `vehicle_name` | string | `bluerov2` | Vehicle namespace |
| `controller_type` | string | `hybrid` | Controller type: `position`, `hybrid`, `velocity` |
| `controller_config` | string | (auto) | Path to controller YAML config |
| `dynamics_config` | string | (auto) | Path to vehicle dynamics YAML |
| `start_simulator` | bool | `false` | Start Stonefish simulator automatically |

**Examples**:

```bash
# Position controller only
ros2 launch stonefish_control controller.launch.py \
    vehicle_name:=bluerov2 \
    controller_type:=position

# Hybrid controller (default)
ros2 launch stonefish_control controller.launch.py

# With simulator
ros2 launch stonefish_control controller.launch.py \
    start_simulator:=true

# Custom config
ros2 launch stonefish_control controller.launch.py \
    controller_config:=/path/to/custom_controller.yaml \
    dynamics_config:=/path/to/custom_dynamics.yaml
```

---

## Topics

### Position Controller

#### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/{vehicle_name}/odometry` | `nav_msgs/Odometry` | Current vehicle state (position + velocity) |
| `/{vehicle_name}/cmd_pose` | `geometry_msgs/PoseStamped` | Desired position and orientation |

#### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/{vehicle_name}/thruster_manager/input_stamped` | `geometry_msgs/WrenchStamped` | 6DOF control wrench (body frame) |

---

### Hybrid Controller

#### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/{vehicle_name}/odometry` | `nav_msgs/Odometry` | Current vehicle state |
| `/{vehicle_name}/cmd_pose` | `geometry_msgs/PoseStamped` | Desired pose (position mode) |
| `/{vehicle_name}/cmd_vel` | `geometry_msgs/Twist` | Desired velocity (velocity mode) |
| `/{vehicle_name}/control_mode` | `std_msgs/String` | Mode switch command: `"velocity"` or `"position"` |

#### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/{vehicle_name}/thruster_manager/input_stamped` | `geometry_msgs/WrenchStamped` | 6DOF control wrench |
| `/{vehicle_name}/controller/current_mode` | `std_msgs/String` | Active control mode |

---

## Services

All controllers provide these services:

| Service | Type | Description |
|---------|------|-------------|
| `/{vehicle_name}/{controller_name}/get_pid_params` | `stonefish_control_msgs/GetPIDParams` | Retrieve current PID gains |
| `/{vehicle_name}/{controller_name}/set_pid_params` | `stonefish_control_msgs/SetPIDParams` | Update PID gains dynamically |
| `/{vehicle_name}/{controller_name}/reset` | `stonefish_control_msgs/ResetController` | Reset controller state |
| `/{vehicle_name}/{controller_name}/hold` | `stonefish_control_msgs/Hold` | Hold current position |

**Example**:

```bash
# Get current gains
ros2 service call /bluerov2/position_controller/get_pid_params \
    stonefish_control_msgs/srv/GetPIDParams

# Update gains
ros2 service call /bluerov2/position_controller/set_pid_params \
    stonefish_control_msgs/srv/SetPIDParams \
    "{kp: [300.0, 300.0, 400.0, 200.0], kd: [150.0, 150.0, 200.0, 100.0], ki: [10.0, 10.0, 20.0, 5.0]}"

# Reset controller
ros2 service call /bluerov2/position_controller/reset \
    stonefish_control_msgs/srv/ResetController

# Hold position
ros2 service call /bluerov2/position_controller/hold \
    stonefish_control_msgs/srv/Hold
```

---

## Parameters

### Position Controller

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `vehicle_name` | string | `bluerov2` | Vehicle namespace |
| `control_rate` | float | `50.0` | Control loop rate (Hz) |
| `Kp` | float[4] | `[300, 300, 400, 200]` | Proportional gains [surge, sway, heave, yaw] |
| `Kd` | float[4] | `[150, 150, 200, 100]` | Derivative gains |
| `Ki` | float[4] | `[10, 10, 20, 5]` | Integral gains |
| `Kb` | float[4] | `[0.8, 0.8, 0.8, 0.8]` | Anti-windup back-calculation gains |
| `max_force` | float | `800.0` | Maximum force per axis (N) |
| `max_torque` | float | `160.0` | Maximum torque for yaw (Nm) |
| `integral_safety_factor` | float | `2.0` | Integral limit multiplier |
| `position_error_threshold` | float | `0.1` | Position acceptance radius (m) |
| `angle_error_threshold` | float | `0.087` | Angle acceptance threshold (rad, ~5°) |

---

### Hybrid Controller

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `vehicle_name` | string | `bluerov2` | Vehicle namespace |
| `control_rate` | float | `50.0` | Control loop rate (Hz) |
| `initial_mode` | string | `velocity` | Starting mode: `velocity` or `position` |
| **Velocity Mode** | | | |
| `velocity_mode.Kp` | float[4] | `[200, 200, 250, 150]` | Velocity mode proportional gains |
| `velocity_mode.Kd` | float[4] | `[0, 100, 100, 80]` | Velocity mode derivative gains |
| `velocity_mode.Ki` | float[4] | `[50, 50, 60, 10]` | Velocity mode integral gains |
| `velocity_mode.Kb` | float[4] | `[0.8, 0.8, 0.8, 0.8]` | Velocity mode anti-windup gains |
| `velocity_mode.max_force` | float | `800.0` | Velocity mode force limit (N) |
| `velocity_mode.max_torque` | float | `160.0` | Velocity mode torque limit (Nm) |
| `velocity_mode.integral_safety_factor` | float | `0.5` | Velocity mode integral safety |
| **Position Mode** | | | |
| `position_mode.Kp` | float[4] | `[300, 300, 400, 200]` | Position mode proportional gains |
| `position_mode.Kd` | float[4] | `[150, 150, 200, 100]` | Position mode derivative gains |
| `position_mode.Ki` | float[4] | `[10, 10, 20, 5]` | Position mode integral gains |
| `position_mode.Kb` | float[4] | `[0.8, 0.8, 0.8, 0.8]` | Position mode anti-windup gains |
| `position_mode.max_force` | float | `800.0` | Position mode force limit (N) |
| `position_mode.max_torque` | float | `160.0` | Position mode torque limit (Nm) |
| `position_mode.integral_safety_factor` | float | `2.0` | Position mode integral safety |

---

## Configuration Files

### Example: position_controller.yaml

```yaml
position_controller_4dof:
  ros__parameters:
    vehicle_name: 'bluerov2'
    control_rate: 50.0

    # PID Gains
    Kp: [300.0, 300.0, 400.0, 200.0]  # [surge, sway, heave, yaw]
    Kd: [150.0, 150.0, 200.0, 100.0]
    Ki: [10.0, 10.0, 20.0, 5.0]
    Kb: [0.8, 0.8, 0.8, 0.8]

    # Saturation
    max_force: 800.0
    max_torque: 160.0

    # Integral limits
    integral_safety_factor: 2.0
```

### Example: hybrid_controller.yaml

```yaml
hybrid_controller_4dof:
  ros__parameters:
    vehicle_name: 'bluerov2'
    control_rate: 50.0
    initial_mode: 'velocity'

    velocity_mode:
      Kp: [200.0, 200.0, 250.0, 150.0]
      Kd: [0.0, 100.0, 100.0, 80.0]
      Ki: [50.0, 50.0, 60.0, 10.0]
      Kb: [0.8, 0.8, 0.8, 0.8]
      max_force: 800.0
      max_torque: 160.0
      integral_safety_factor: 0.5

    position_mode:
      Kp: [300.0, 300.0, 400.0, 200.0]
      Kd: [150.0, 150.0, 200.0, 100.0]
      Ki: [10.0, 10.0, 20.0, 5.0]
      Kb: [0.8, 0.8, 0.8, 0.8]
      max_force: 800.0
      max_torque: 160.0
      integral_safety_factor: 2.0
```

### Dynamics Configuration (dynamics_params.yaml)

Required for all controllers. Located in `stonefish_description`:

```yaml
# /path/to/stonefish_description/data/robots/bluerov2/config/dynamics_params.yaml
vehicle_dynamics:
  ros__parameters:
    mass: 11.5          # kg
    inertia_zz: 0.6     # kg·m² (yaw inertia)
    center_of_mass: [0.0, 0.0, 0.0]
    center_of_buoyancy: [0.0, 0.0, 0.0]
```

---

## Integration Examples

### With Stonefish Simulator

```bash
# Terminal 1: Simulator
ros2 launch stonefish_ros2 bluerov2.launch.py

# Terminal 2: Position controller
ros2 launch stonefish_control controller.launch.py controller_type:=position

# Terminal 3: Send pose command
ros2 topic pub /bluerov2/cmd_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world_ned'}, pose: {position: {x: 5.0, y: 0.0, z: 2.0}}}" --once
```

### With Path Following

```bash
# Terminal 1: Simulator
ros2 launch stonefish_ros2 bluerov2.launch.py

# Terminal 2: Hybrid controller
ros2 launch stonefish_control controller.launch.py controller_type:=hybrid

# Terminal 3: Path following (publishes cmd_pose and mode switches)
ros2 launch stonefish_trajectory_manager path_following.launch.py \
    waypoint_file:=config/example_waypoints.yaml \
    vehicle_name:=bluerov2
```

---

## Testing

### Manual Pose Command

```bash
# Move to 5m North, 2m depth
ros2 topic pub /bluerov2/cmd_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world_ned'}, \
    pose: {position: {x: 5.0, y: 0.0, z: 2.0}, \
           orientation: {x: 0, y: 0, z: 0, w: 1}}}" --once
```

### Monitor Control Output

```bash
# Check wrench output
ros2 topic echo /bluerov2/thruster_manager/input_stamped

# Check odometry feedback
ros2 topic echo /bluerov2/odometry
```

### Verify Controller Activity

```bash
# List nodes
ros2 node list | grep controller

# Check node info
ros2 node info /bluerov2/position_controller
```

---

## PID Tuning Guide

### Step-by-Step Tuning

1. **Start with Kp only** (Kd = 0, Ki = 0):
   - Increase Kp until oscillations appear
   - Reduce to 70% of oscillation threshold

2. **Add Kd for damping**:
   - Increase Kd until oscillations disappear
   - Typical ratio: Kd ≈ 0.5 × Kp

3. **Add Ki for steady-state accuracy**:
   - Start with small Ki (1-5% of Kp)
   - Increase until steady-state error < 0.05m

4. **Adjust saturation limits**:
   - Monitor `/thruster_manager/input_stamped`
   - Ensure wrench values stay below saturation

### Tuning Parameters by DOF

| DOF | Characteristics | Tuning Strategy |
|-----|-----------------|-----------------|
| **Surge (X)** | Slow dynamics | Moderate Kp, low Kd |
| **Sway (Y)** | Fast coupling with yaw | High Kp, moderate Kd |
| **Heave (Z)** | Buoyancy restoring force | High Kp, high Kd, moderate Ki |
| **Yaw** | Low inertia, fast response | Moderate Kp, low Kd |

### Common Issues

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Oscillations | Kp too high or Kd too low | Reduce Kp or increase Kd |
| Slow convergence | Kp too low | Increase Kp |
| Steady-state error | Ki too low | Increase Ki |
| Integral windup | Ki too high or limits too loose | Reduce Ki or `integral_safety_factor` |
| Overshoot | Kd too low | Increase Kd |

---

## Package Structure

```
stonefish_control/
├── stonefish_control/
│   ├── controllers/
│   │   ├── position_controller.py     # Position PID controller
│   │   ├── hybrid_controller.py       # Hybrid mode controller
│   │   └── velocity_controller_node.py  # Velocity controller node
│   ├── control_interfaces/
│   │   ├── dp_controller_base.py      # Base class for controllers
│   │   ├── dp_pid_controller_base.py  # PID controller base
│   │   ├── vehicle.py                 # Vehicle model
│   │   └── dynamics_loader.py         # Dynamics parameter loader
│   └── nodes/
│       ├── position_controller_node.py  # Position controller ROS2 node
│       └── hybrid_controller_node.py    # Hybrid controller ROS2 node
├── launch/
│   └── controller.launch.py           # Main launch file
├── config/
│   └── bluerov2/
│       ├── position_controller.yaml   # Position PID config
│       ├── hybrid_controller.yaml     # Hybrid controller config
│       └── velocity_controller.yaml   # Velocity controller config
├── package.xml
├── setup.py
└── README.md
```

---

## Related Packages

- **stonefish_control_msgs**: Message and service definitions
- **stonefish_thruster_manager**: TAM-based thruster allocation
- **stonefish_trajectory_manager**: Path generation and following
- **stonefish_description**: Vehicle models and dynamics parameters

---

## References

### Control Theory

- Fossen, T.I. (2011). "Handbook of Marine Craft Hydrodynamics and Motion Control"
  - Section 8.2.1: PID Control for Station-Keeping
  - Section 12.4: Path Following Control
- Åström, K.J., & Hägglund, T. (1995). "PID Controllers: Theory, Design, and Tuning"
  - Chapter 3: Anti-Windup and Bumpless Transfer

### Implementation

- Based on UUV Simulator framework (Apache 2.0 license)
- Adapted for ROS2 Humble and Stonefish simulator

---

## License

Apache 2.0

Based on UUV Simulator (Copyright 2016 The UUV Simulator Authors)

---

## Notes

### 4DOF Control

This package implements **4DOF control** for underactuated vehicles:
- **Controlled**: X, Y, Z (position), Yaw (heading)
- **Passive**: Roll, Pitch (stabilized by buoyancy/gravity)

**Design Assumption**: Vehicle has positive metacentric height (GM ≥ 0.15m) for passive roll/pitch stability.

### Coordinate System

**NED (North-East-Down)** convention:
- World frame: `world_ned` (X=North, Y=East, Z=Down)
- Body frame: `base_link` (FRD: X=Forward, Y=Right, Z=Down)
- Control wrench: Always in body frame
- Pose commands: In world frame (automatically transformed)

### Anti-Windup

All controllers use **back-calculation anti-windup** (Åström & Hägglund 1995):

```
integral -= (u - u_sat) / Ki * Kb
```

Where:
- `u` = Unsaturated control signal
- `u_sat` = Saturated control signal
- `Kb` = Back-calculation gain (typically 0.5-1.0)

This prevents integral windup during saturation while maintaining good tracking performance.
