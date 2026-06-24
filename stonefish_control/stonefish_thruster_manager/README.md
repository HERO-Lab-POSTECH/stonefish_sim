# stonefish_thruster_manager

Thruster allocation system using the Thruster Allocation Matrix (TAM) method for marine vehicles.

## Overview

The thruster manager converts 6DOF wrench commands (forces and torques) into individual thruster forces using the **Thruster Allocation Matrix (TAM)**. This package provides both a ROS2 node for real-time allocation and a Python API for offline calculations.

**Key Concept**: TAM defines the mapping from individual thruster forces to the resulting 6DOF wrench on the vehicle body.

## Features

- **TAM-based allocation**: Pseudo-inverse method for overdetermined systems
- **Configurable TAM**: Load from YAML files for different vehicles
- **Timeout safety**: Automatic thruster shutdown on command timeout
- **PWM normalization**: Converts forces to PWM setpoints for Stonefish
- **Python API**: Use thruster allocation in custom scripts

---

## How TAM Works

### Theory

The **Thruster Allocation Matrix (TAM)** relates individual thruster forces to the resulting wrench on the vehicle:

```
τ = T · f
```

Where:
- `τ` = 6DOF wrench [Fx, Fy, Fz, Tx, Ty, Tz] (forces and torques on vehicle body)
- `T` = TAM matrix (6 × N, where N = number of thrusters)
- `f` = Thruster forces [F1, F2, ..., FN]

For **control**, we need the **inverse** mapping (wrench → thruster forces):

```
f = T⁺ · τ
```

Where `T⁺` is the **Moore-Penrose pseudo-inverse** of T.

### Example: BlueROV2

BlueROV2 has **8 thrusters** (N=8):
- 4 horizontal thrusters at 45° angles (surge + sway + yaw)
- 4 vertical thrusters (heave)

The TAM is a **6×8 matrix**:
- Rows: [Surge, Sway, Heave, Roll, Pitch, Yaw]
- Columns: [T1, T2, T3, T4, T5, T6, T7, T8]

Each column represents the contribution of one thruster to each DOF.

### Overdetermined System

Since 8 thrusters > 6 DOFs, the system is **overdetermined**:
- Multiple thruster combinations can produce the same wrench
- Pseudo-inverse finds the **minimum-norm solution** (least total thrust)

---

## Installation

### Build from Source

```bash
cd /workspace/colcon_ws
colcon build --packages-select stonefish_thruster_manager
source install/setup.bash
```

### Dependencies

**ROS2 Packages**:
- `geometry_msgs`
- `std_msgs`
- `stonefish_description` (for default TAM files)

**Python**:
- `numpy`
- `pyyaml`

---

## Usage

### Launch File

#### Basic Usage

```bash
ros2 launch stonefish_thruster_manager thruster_manager.launch.py \
    vehicle_name:=bluerov2
```

This uses the default TAM file: `/path/to/stonefish_description/data/robots/bluerov2/config/TAM.yaml`

#### Custom TAM File

```bash
ros2 launch stonefish_thruster_manager thruster_manager.launch.py \
    vehicle_name:=my_vehicle \
    tam_file:=/path/to/custom_TAM.yaml
```

#### All Parameters

```bash
ros2 launch stonefish_thruster_manager thruster_manager.launch.py \
    vehicle_name:=bluerov2 \
    tam_file:=/custom/path/TAM.yaml \
    update_rate:=50.0 \
    timeout:=1.0 \
    max_thrust:=100.0
```

---

## Launch Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `vehicle_name` | string | `bluerov2` | Vehicle namespace |
| `tam_file` | string | (auto) | Path to TAM YAML file. If empty, auto-detects from `vehicle_name` |
| `update_rate` | float | `50.0` | Update rate (Hz) |
| `timeout` | float | `1.0` | Command timeout (seconds). Set to 0 to disable |
| `max_thrust` | float | `100.0` | Maximum thrust per thruster (N). Used for PWM normalization |

---

## Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/{vehicle_name}/thruster_manager/input_stamped` | `geometry_msgs/WrenchStamped` | 6DOF wrench command (body frame) |

**Wrench Convention**:
```
force.x    → Surge force (N)
force.y    → Sway force (N)
force.z    → Heave force (N)
torque.x   → Roll torque (Nm)
torque.y   → Pitch torque (Nm)
torque.z   → Yaw torque (Nm)
```

Frame: **Body FRD (Forward-Right-Down)**

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/{vehicle_name}/setpoint/pwm` | `std_msgs/Float64MultiArray` | Thruster PWM setpoints (-1.0 to 1.0) |

**PWM Convention**:
- `1.0` = Full forward thrust
- `0.0` = Zero thrust
- `-1.0` = Full reverse thrust

**Array Size**: Matches number of thrusters (e.g., 8 for BlueROV2)

**Index Mapping**: Defined by TAM column order (see TAM file)

---

## Parameters (Node)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `vehicle_name` | string | `bluerov2` | Vehicle name (namespace) |
| `tam_file` | string | (auto) | TAM YAML file path |
| `update_rate` | float | `50.0` | Control loop rate (Hz) |
| `timeout` | float | `1.0` | Command timeout for safety shutdown |
| `max_thrust` | float | `100.0` | Max thrust per thruster for PWM scaling |

---

## TAM File Format

### YAML Structure

```yaml
# TAM.yaml - Thruster Allocation Matrix for BlueROV2
#
# Matrix size: 6 x N (6 DOFs × N thrusters)
# Rows: [Surge, Sway, Heave, Roll, Pitch, Yaw]
# Columns: [T1, T2, T3, T4, T5, T6, T7, T8]

tam:
  # Row 0: Surge (X-axis force)
  - [0.707,  0.707, -0.707, -0.707,  0.0,  0.0,  0.0,  0.0]

  # Row 1: Sway (Y-axis force)
  - [0.707, -0.707,  0.707, -0.707,  0.0,  0.0,  0.0,  0.0]

  # Row 2: Heave (Z-axis force)
  - [0.0,    0.0,    0.0,    0.0,    1.0,  1.0,  1.0,  1.0]

  # Row 3: Roll (torque around X)
  - [0.0,    0.0,    0.0,    0.0,   -0.1,  0.1,  0.1, -0.1]

  # Row 4: Pitch (torque around Y)
  - [0.0,    0.0,    0.0,    0.0,    0.15, 0.15, -0.15, -0.15]

  # Row 5: Yaw (torque around Z)
  - [0.218, -0.218, -0.218,  0.218,  0.0,  0.0,  0.0,  0.0]
```

### How to Read TAM

Each **column** represents one thruster's contribution to all 6 DOFs.

**Example - Thruster 1** (column 0):
```
[0.707, 0.707, 0.0, 0.0, 0.0, 0.218]
```
Means:
- Produces **0.707 N** surge force per N of thrust
- Produces **0.707 N** sway force per N of thrust
- Produces **0.218 Nm** yaw torque per N of thrust
- No heave force (horizontal thruster)

**Physical Meaning**: Thruster 1 is at 45° angle, producing both forward and lateral force.

### Creating TAM for New Vehicles

For each thruster `i`:

1. **Position vector** `r_i = [x, y, z]` (from body center of mass to thruster)
2. **Thrust direction** `d_i = [dx, dy, dz]` (unit vector)
3. **Compute contributions**:
   - Force: `F_i = d_i` (rows 0-2)
   - Torque: `τ_i = r_i × d_i` (rows 3-5)

**Example**: Thruster at position `[0.2, 0.1, 0]` pointing forward-right at 45°:
```python
r = np.array([0.2, 0.1, 0])
d = np.array([0.707, 0.707, 0])
tau = np.cross(r, d)  # Torque contribution

# TAM column: [d[0], d[1], d[2], tau[0], tau[1], tau[2]]
```

---

## Python API

### ThrusterManager Class

```python
from stonefish_thruster_manager.thruster_manager import ThrusterManager
import numpy as np

# Initialize from TAM file
tam_mgr = ThrusterManager(tam_file_path='/path/to/TAM.yaml')

# Or provide TAM matrix directly
tam_matrix = np.array([...])  # 6xN matrix
tam_mgr = ThrusterManager(tam_matrix=tam_matrix)

# Get TAM info
print(f"Number of thrusters: {tam_mgr.n_thrusters}")
print(f"TAM shape: {tam_mgr.tam.shape}")

# Compute thruster forces from wrench
wrench = np.array([10.0, 0.0, 20.0, 0.0, 0.0, 5.0])  # [Fx, Fy, Fz, Tx, Ty, Tz]
thrust_forces = tam_mgr.compute_thrust_forces(wrench)

print(f"Required thrust forces: {thrust_forces}")
```

### Example: Offline Calculation

```python
#!/usr/bin/env python3
from stonefish_thruster_manager.thruster_manager import ThrusterManager
import numpy as np

# Load TAM
tam_path = '/workspace/colcon_ws/src/stonefish_description/data/robots/bluerov2/config/TAM.yaml'
tam_mgr = ThrusterManager(tam_file_path=tam_path)

# Desired wrench (100N forward, 50Nm yaw torque)
wrench = np.array([100.0, 0.0, 0.0, 0.0, 0.0, 50.0])

# Compute allocation
thrust = tam_mgr.compute_thrust_forces(wrench)

# Print results
print("Desired wrench:")
print(f"  Surge: {wrench[0]:.2f} N")
print(f"  Yaw:   {wrench[5]:.2f} Nm")
print("\nThruster allocation:")
for i, f in enumerate(thrust):
    print(f"  T{i+1}: {f:6.2f} N")
```

---

## Integration Example

### With Position Controller

Complete control chain:

```
Position Controller
  ↓ publishes geometry_msgs/WrenchStamped on /bluerov2/thruster_manager/input_stamped
Thruster Manager (this package)
  ↓ publishes std_msgs/Float64MultiArray on /bluerov2/setpoint/pwm
Stonefish Simulator
```

**Launch both**:

```bash
# Terminal 1: Simulator
ros2 launch stonefish_ros2 bluerov2.launch.py

# Terminal 2: Controller
ros2 launch stonefish_control controller.launch.py controller_type:=position

# Thruster manager is automatically started by bluerov2.launch.py
```

**Manual command** (for testing):

```bash
ros2 topic pub /bluerov2/thruster_manager/input_stamped geometry_msgs/msg/WrenchStamped \
  "{header: {frame_id: 'base_link'}, wrench: {force: {x: 50.0, y: 0.0, z: 0.0}}}" --once
```

---

## Verification

### Check Topics

```bash
# List active topics
ros2 topic list | grep thruster

# Monitor wrench input
ros2 topic echo /bluerov2/thruster_manager/input_stamped

# Monitor PWM output
ros2 topic echo /bluerov2/setpoint/pwm
```

### Verify TAM Loading

```bash
# Check node logs
ros2 run stonefish_thruster_manager thruster_allocator --ros-args \
    -p vehicle_name:=bluerov2 \
    -p tam_file:=/path/to/TAM.yaml
```

Expected output:
```
[INFO] [thruster_allocator]: Loading TAM from: /path/to/TAM.yaml
[INFO] [thruster_allocator]: TAM loaded successfully: 8 thrusters
[INFO] [thruster_allocator]: Output topic: /bluerov2/setpoint/pwm
```

---

## Package Structure

```
stonefish_thruster_manager/
├── stonefish_thruster_manager/
│   ├── __init__.py
│   ├── thruster_manager.py             # TAM loader and allocation logic
│   └── nodes/
│       ├── __init__.py
│       └── thruster_allocator_node.py  # ROS2 node
├── launch/
│   └── thruster_manager.launch.py      # Launch file
├── test/
│   └── test_thruster_manager.py
├── package.xml
├── setup.py
└── README.md
```

---

## Related Packages

- **stonefish_control**: Controllers that output wrench commands
- **stonefish_description**: Contains default TAM files for vehicles
- **stonefish_ros2**: Stonefish simulator interface

---

## Troubleshooting

### No thruster output

**Check**:
1. Verify topic connection:
   ```bash
   ros2 topic info /bluerov2/thruster_manager/input_stamped
   ros2 topic info /bluerov2/setpoint/pwm
   ```
2. Check timeout parameter (set to 0 to disable)
3. Monitor logs for TAM loading errors

### Wrong thruster behavior

**Check**:
1. TAM file matches vehicle configuration
2. Frame convention (wrench must be in body FRD frame)
3. Thruster index mapping in TAM file

### TAM file not found

**Solution**: Ensure `stonefish_description` is installed or provide absolute path:
```bash
ros2 launch stonefish_thruster_manager thruster_manager.launch.py \
    tam_file:=/absolute/path/to/TAM.yaml
```

---

## References

- Fossen, T.I. (2011). "Handbook of Marine Craft Hydrodynamics and Motion Control", Section 3.5: Thruster Models
- Johansen, T.A., & Fossen, T.I. (2013). "Control allocation—A survey"

---

## License

Apache 2.0
