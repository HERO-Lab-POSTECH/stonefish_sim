# Stonefish Simulation for Marine Robotics

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)](https://releases.ubuntu.com/22.04/)
[![Stonefish](https://img.shields.io/badge/Stonefish-1.5+-green.svg)](https://github.com/HERO-Lab-POSTECH/stonefish)

A comprehensive ROS2 integration workspace for the Stonefish marine robotics simulator. This workspace provides a complete simulation stack for underwater vehicles with advanced physics, realistic sensors, and integrated control systems.

## Overview

This workspace integrates the [Stonefish](https://github.com/patrykcieslak/stonefish) C++ simulation library with ROS2 Humble, enabling high-fidelity simulation of underwater vehicles including BlueROV2, BlueBoat, and other marine robots. The system combines realistic physics (Bullet Physics), GPU-accelerated rendering (OpenGL), and comprehensive sensor simulation with a full ROS2 control architecture.

**Key Capabilities:**
- Physics-based marine robotics simulation with hydrodynamics and buoyancy
- GPU-accelerated underwater rendering with realistic lighting and water effects
- Comprehensive sensor suite (DVL, IMU, pressure, cameras, FLS, SSS, MSIS)
- Advanced control systems (position, velocity, hybrid controllers)
- Trajectory planning and path following with LOS guidance
- Environment control (ocean currents, waves, wind)
- SLAM capabilities for underwater navigation
- Modular XML-based scenario definition system
- NED (North-East-Down) coordinate system for marine robotics

**Supported Vehicles:**
- **BlueROV2** - Heavy configuration with 8 thrusters (fully implemented)
- **BlueBoat** - Surface vehicle platform
- Legacy vehicles (GIRONA500, SPARUS2, in `_legacy/` directory)

## Features

### Simulation Capabilities
- **Advanced Physics**: Bullet-based rigid body dynamics with marine-specific forces
- **Hydrodynamics**: Added mass, linear/quadratic damping, buoyancy modeling
- **Realistic Rendering**: Underwater light absorption, scattering, caustics
- **Real-time Performance**: Optimized for interactive simulation at 100+ Hz

### Sensor Simulation
- **Navigation Sensors**: DVL, IMU, pressure sensors, GPS, USBL
- **Vision Sensors**: RGB cameras, depth cameras with underwater effects
- **Acoustic Sensors**: Forward-Looking Sonar (FLS), Side-Scan Sonar (SSS), Mechanically-Scanned Imaging Sonar (MSIS)
- **Contact Sensors**: Force-torque sensors, contact detection

### Control System
- **Controllers**: Position, Velocity, Hybrid (position + velocity)
- **Thruster Allocation**: TAM-based (Thruster Allocation Matrix) wrench-to-thrust conversion
- **Trajectory Planning**: Linear, LIPB (Log-Interpolated Polynomial Bezier), Cubic spline interpolation
- **Path Following**: LOS (Line-of-Sight) guidance with adaptive velocity profiling
- **Teleoperation**: Joystick/keyboard control (planned)

### Environment Control
- **Ocean Currents**: Runtime-adjustable water currents (velocity vector control)
- **Wave Simulation**: Dynamic wave height adjustment (0-10m)
- **Wind Simulation**: Atmospheric wind velocity control for surface vehicles

### SLAM Integration
- **Sonar-based SLAM**: Feature extraction and mapping from acoustic sensors
- **Kalman Filtering**: State estimation and sensor fusion
- **Dead Reckoning**: Odometry integration for navigation

## Package Structure

```
stonefish_sim/
├── stonefish_msgs/                      # Core simulation messages and services
│   ├── msg/                             # 7 message definitions (DVL, IMU, etc.)
│   └── srv/                             # 6 service definitions (environment control)
│
├── stonefish_description/               # Robot models, worlds, and scenarios
│   ├── data/
│   │   ├── robots/                      # Robot definitions (BlueROV2, BlueBoat)
│   │   ├── models/                      # Environmental objects
│   │   └── worlds/                      # Base environments (tank, seabed)
│   └── scenarios/                       # Complete simulation scenarios
│
├── stonefish_ros2/                      # Core ROS2 simulator interface
│   ├── include/stonefish_ros2/          # C++ interface library
│   ├── src/                             # Simulator nodes (GPU/NoGPU)
│   └── launch/                          # Launch files
│
└── stonefish_control/                   # Control system packages
    ├── stonefish_control_msgs/          # Control-specific messages (4 msg, 22 srv)
    ├── stonefish_thruster_manager/      # TAM-based thrust allocation
    ├── stonefish_control/               # Controllers (Position, Hybrid, Velocity)
    ├── stonefish_trajectory_manager/    # Path generation and following
    ├── stonefish_control_utils/         # PID optimizer and utilities
    └── stonefish_teleop_manager/        # Teleoperation interface (planned)
```

**Package Details:**
- **[stonefish_msgs](stonefish_msgs/README.md)**: Core simulation messages and environment control services
- **[stonefish_description](stonefish_description/README.md)**: Robot models, world definitions, and scenario compositions
- **[stonefish_ros2](stonefish_ros2/README.md)**: C++ simulator interface with ROS2 bindings
- **[stonefish_control](stonefish_control/README.md)**: Complete control system stack (see subdirectory READMEs for details)

## Requirements

### Hardware Requirements
- **CPU**: Modern multi-core processor (4+ cores recommended for real-time simulation)
- **GPU**: Discrete graphics card with **OpenGL 4.3+** support
  - NVIDIA GTX 960 or higher (recommended)
  - AMD Radeon R9 290 or higher
  - Intel integrated graphics may work but performance will be limited

### Software Requirements
- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS2 Humble Hawksbill
- **Graphics Drivers**: Official manufacturer drivers (NVIDIA/AMD proprietary drivers recommended)

> **⚠️ Important**: Install official GPU drivers before building Stonefish. Generic open-source drivers may not support OpenGL 4.3 and will cause rendering failures.

### Dependencies

**ROS2 Packages:**
```bash
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-std-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-image-transport
```

**External Libraries:**
```bash
sudo apt install -y \
    libglm-dev \
    libsdl2-dev \
    libfreetype6-dev \
    libbullet-dev \
    libopengl-dev
```

> **Note**: If you encounter SDL2 CMake issues, check `/usr/lib/x86_64-linux-gnu/cmake/SDL2/sdl2-config.cmake` and remove any extra spaces after `-lSDL2`.

## Installation

### 1. Install System Dependencies

```bash
# Update package list
sudo apt update

# Install graphics libraries
sudo apt install -y libglm-dev libsdl2-dev libfreetype6-dev libbullet-dev libopengl-dev

# Install ROS2 Humble (if not already installed)
sudo apt install -y ros-humble-desktop ros-humble-tf2-ros ros-humble-image-transport
```

### 2. Install HERO Lab Stonefish Library

**IMPORTANT:** This workspace requires the HERO Lab fork of Stonefish, which includes custom fixes for Forward-Looking Sonar (FLS) and other marine robotics features.

```bash
# Clone HERO Lab Stonefish repository
cd /workspace
git clone https://github.com/HERO-Lab-POSTECH/stonefish.git
cd stonefish

# Build and install
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

**Verify Installation:**
```bash
# Check if library is installed
ls -l /usr/local/lib/libStonefish.so
# Should show the library file

# Check include headers
ls /usr/local/include/Stonefish/
# Should show header files
```

**Build Options:**
- `-DBUILD_TESTS=ON` - Build test applications and examples
- `-DEMBED_RESOURCES=ON` - Embed shader resources into binary

### 3. Build ROS2 Workspace

```bash
# Clone the workspace
cd /workspace/colcon_ws/src
git clone https://github.com/HERO-Lab-POSTECH/stonefish_sim.git

# Build all packages
cd /workspace/colcon_ws
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

colcon build --symlink-install

# Source workspace
source install/setup.bash
```

**Build Specific Packages:**
```bash
# Build only core packages
colcon build --packages-select stonefish_msgs stonefish_description stonefish_ros2

# Build control system
colcon build --packages-select stonefish_control_msgs stonefish_thruster_manager stonefish_control

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## Quick Start

### Basic BlueROV2 Simulation

```bash
# Source workspace
source /workspace/colcon_ws/install/setup.bash

# Launch BlueROV2 in empty environment (GPU-accelerated)
ros2 launch stonefish_ros2 bluerov2.launch.py

# Launch with thruster manager
ros2 launch stonefish_ros2 bluerov2.launch.py start_thruster_manager:=true

# Launch specific scenario
ros2 launch stonefish_ros2 bluerov2.launch.py scenario:=bluerov2_tank
```

**Available Scenarios:**
- `bluerov2_empty` - Empty ocean environment (default)
- `bluerov2_tank` - Indoor water tank with walls
- `bluerov2_seabed` - Ocean floor with rocky terrain

### With Position Controller

```bash
# Launch simulator + thruster manager + position controller
ros2 launch stonefish_ros2 bluerov2.launch.py start_thruster_manager:=true

# In another terminal, launch controller
ros2 launch stonefish_control controller.launch.py \
    vehicle_name:=bluerov2 \
    controller_type:=position

# Send position command (move to [5, 3, 2] meters in NED frame)
ros2 topic pub /bluerov2/cmd_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'world_ned'},
      pose: {position: {x: 5.0, y: 3.0, z: 2.0},
             orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" --once
```

### With Path Following

```bash
# Generate and visualize path
ros2 launch stonefish_trajectory_manager path_generator.launch.py \
    waypoint_file:=/workspace/colcon_ws/src/stonefish_sim/stonefish_control/stonefish_trajectory_manager/config/example_waypoints.yaml \
    interpolation_method:=lipb

# Launch path following with LOS guidance
ros2 launch stonefish_trajectory_manager path_following.launch.py \
    waypoint_file:=/workspace/colcon_ws/src/stonefish_sim/stonefish_control/stonefish_trajectory_manager/config/example_waypoints.yaml \
    vehicle_name:=bluerov2 \
    lookahead_distance:=2.5 \
    robot_max_speed:=1.0
```

### Headless Simulation (No GPU)

For running on servers without displays or GPUs:

```bash
ros2 launch stonefish_ros2 simulator_nogpu.launch.py \
    scenario_description:=/workspace/colcon_ws/src/stonefish_sim/stonefish_description/scenarios/bluerov2_empty.scn \
    simulation_rate:=100.0
```

## Usage Examples

### Environment Control

Control ocean conditions dynamically during simulation:

```bash
# Set wave height to 2.0 meters
ros2 service call /set_wave_height stonefish_msgs/srv/SetWaveHeight "{height: 2.0}"

# Set wind velocity (5 m/s North, 2 m/s East)
ros2 service call /set_wind_velocity stonefish_msgs/srv/SetWindVelocity \
    "{x: 5.0, y: 2.0, z: 0.0}"

# Enable ocean current (1 m/s East)
ros2 service call /set_ocean_current stonefish_msgs/srv/SetOceanCurrent \
    "{current_index: 0, enable: true, velocity: [0.0, 1.0, 0.0]}"
```

### Robot Control

#### Manual Wrench Control (6DOF)

```bash
# Forward thrust (10N) with upward force (5N) and yaw torque (2Nm)
ros2 topic pub /bluerov2/thruster_manager/input geometry_msgs/msg/Wrench \
    "{force: {x: 10.0, y: 0.0, z: 5.0}, torque: {x: 0.0, y: 0.0, z: 2.0}}" --once
```

#### Velocity Control

```bash
# Launch velocity controller
ros2 launch stonefish_control controller.launch.py \
    vehicle_name:=bluerov2 \
    controller_type:=velocity

# Command velocity (0.5 m/s forward, 0.2 m/s right)
ros2 topic pub /bluerov2/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.5, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}" --once
```

#### Hybrid Control (Position + Velocity)

```bash
# Launch hybrid controller (position control in XY, velocity control in Z, yaw)
ros2 launch stonefish_control controller.launch.py \
    vehicle_name:=bluerov2 \
    controller_type:=hybrid
```

### Monitoring Robot State

```bash
# Monitor odometry (ground truth position/velocity)
ros2 topic echo /bluerov2/odometry

# Monitor thruster outputs
ros2 topic echo /bluerov2/setpoint/pwm

# Monitor DVL velocity
ros2 topic echo /bluerov2/dvl

# Monitor IMU data
ros2 topic echo /bluerov2/imu

# Monitor pressure (depth)
ros2 topic echo /bluerov2/pressure

# View camera image (requires image_view or RViz2)
ros2 run image_view image_view --ros-args --remap image:=/bluerov2/camera_front/image_color

# View Forward-Looking Sonar
ros2 run image_view image_view --ros-args --remap image:=/bluerov2/fls/image
```

### SLAM Demonstration

```bash
# Launch simulator with sonar-equipped vehicle
ros2 launch stonefish_ros2 bluerov2.launch.py

# Launch SLAM system (in separate terminal)
ros2 launch stonefish_slam slam_sim.launch.py vehicle_name:=bluerov2

# Monitor SLAM output
ros2 topic echo /slam/map
ros2 topic echo /slam/pose_estimate
```

## ROS2 Interface

### Topic Overview

| Topic Pattern | Type | Description |
|--------------|------|-------------|
| `/{vehicle}/odometry` | `nav_msgs/Odometry` | Ground truth pose and velocity (NED frame) |
| `/{vehicle}/cmd_pose` | `geometry_msgs/PoseStamped` | Position setpoint (for position controller) |
| `/{vehicle}/cmd_vel` | `geometry_msgs/Twist` | Velocity setpoint (for velocity controller) |
| `/{vehicle}/thruster_manager/input` | `geometry_msgs/Wrench` | 6DOF wrench input [Fx, Fy, Fz, Tx, Ty, Tz] |
| `/{vehicle}/setpoint/pwm` | `std_msgs/Float64MultiArray` | Individual thruster outputs (-1.0 to 1.0) |
| `/{vehicle}/dvl` | `stonefish_msgs/DVL` | Doppler Velocity Log measurements |
| `/{vehicle}/imu` | `sensor_msgs/Imu` | Inertial Measurement Unit data |
| `/{vehicle}/pressure` | `sensor_msgs/FluidPressure` | Pressure sensor (depth estimation) |
| `/{vehicle}/camera_*/image_*` | `sensor_msgs/Image` | Camera images |
| `/{vehicle}/fls/image` | `sensor_msgs/Image` | Forward-Looking Sonar image |

### Service Overview

| Service | Type | Description |
|---------|------|-------------|
| `/set_wave_height` | `stonefish_msgs/srv/SetWaveHeight` | Set ocean wave height (0-10m) |
| `/set_wind_velocity` | `stonefish_msgs/srv/SetWindVelocity` | Set wind velocity vector (NED) |
| `/set_ocean_current` | `stonefish_msgs/srv/SetOceanCurrent` | Enable/disable ocean current |

For detailed interface documentation, see individual package READMEs:
- [stonefish_msgs/README.md](stonefish_msgs/README.md) - Message and service definitions
- [stonefish_control_msgs/README.md](stonefish_control/stonefish_control_msgs/README.md) - Control-specific messages

## Coordinate Systems

### NED Convention (North-East-Down)

This workspace strictly follows the **NED** coordinate system, standard in marine robotics:

**World Frame (`world_ned`):**
- **X-axis**: Points North (forward)
- **Y-axis**: Points East (right)
- **Z-axis**: Points Down (into water)
- **Origin**: Typically at water surface

**Body Frame (`base_link_frd`):**
- **X-axis**: Points Forward (bow)
- **Y-axis**: Points Right (starboard)
- **Z-axis**: Points Down

**Important Notes:**
- **Depth**: `Z > 0` means underwater. Example: `[0, 0, 2.0]` = 2 meters underwater
- **Altitude**: Negative Z values are above water (rare for underwater vehicles)
- **RViz Compatibility**: A static TF publishes `world` (ENU) ↔ `world_ned` (NED) for visualization

**Example Position Commands:**
```bash
# Move to 5m North, 3m East, 2m depth
position: {x: 5.0, y: 3.0, z: 2.0}

# Surface position (at water surface)
position: {x: 0.0, y: 0.0, z: 0.0}

# 10m depth, directly below origin
position: {x: 0.0, y: 0.0, z: 10.0}
```

### Frame Transforms

Key TF frames published by the system:
- `world_ned` → `{vehicle}/base_link` (vehicle pose in world)
- `{vehicle}/base_link` → `{vehicle}/base_link_frd` (coordinate frame correction)
- `{vehicle}/base_link` → sensor frames (cameras, sonars, DVL, etc.)

## Creating Custom Scenarios

Scenarios are defined using XML files with a modular `<include>` system.

### Basic Structure

```xml
<?xml version="1.0"?>
<scenario>
    <!-- Include base world environment -->
    <include file="worlds/tank.scn"/>

    <!-- Include robot with configuration -->
    <include file="robots/bluerov2/bluerov2.scn">
        <arg name="vehicle_name" value="bluerov2"/>
        <arg name="xyz" value="0.0 0.0 1.0"/>  <!-- Start position (NED) -->
        <arg name="rpy" value="0.0 0.0 0.0"/>  <!-- Orientation (roll, pitch, yaw) -->
    </include>

    <!-- Optional: Add environmental objects -->
    <!-- <include file="models/pipe/pipe.scn"/> -->
</scenario>
```

### Resource Hierarchy

1. **`worlds/common/`** - Shared configuration (NED convention, ocean properties, materials)
2. **`worlds/`** - Base environments (tank.scn, seabed.scn, ocean.scn)
3. **`robots/`** - Robot definitions with sensors and actuators
4. **`models/`** - Reusable objects (pipes, rocks, structures)
5. **`scenarios/`** - Final compositions combining world + robots + objects

### Adding Custom Robots

Create a new robot directory under `stonefish_description/data/robots/your_robot/`:

```
your_robot/
├── your_robot.scn           # Stonefish robot definition
├── meshes/                  # 3D models (STL, OBJ)
│   ├── body.obj
│   └── thruster.obj
└── config/
    └── TAM.yaml            # Thruster Allocation Matrix (if using thruster manager)
```

**TAM.yaml Format:**
```yaml
# 6×N matrix where N = number of thrusters
# Each column represents one thruster's contribution to [Fx, Fy, Fz, Tx, Ty, Tz]
thruster_allocation_matrix:
  - [1.0, 0.0, 0.0, 0.0, 0.0, 0.5]   # Thruster 1
  - [1.0, 0.0, 0.0, 0.0, 0.0, -0.5]  # Thruster 2
  # ... (add rows for each thruster)
```

For detailed scenario creation, see [stonefish_description/README.md](stonefish_description/README.md).

## Development

### Building Workspace

```bash
# Build all packages
cd /workspace/colcon_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Build specific package
colcon build --packages-select stonefish_ros2

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Clean build
rm -rf build/ install/ log/
colcon build
```

### Testing

```bash
# Run specific scenario
ros2 launch stonefish_ros2 bluerov2.launch.py

# Check published topics
ros2 topic list

# Monitor specific topic
ros2 topic echo /bluerov2/odometry

# Call service
ros2 service call /set_wave_height stonefish_msgs/srv/SetWaveHeight "{height: 1.0}"

# Verify thruster allocation (if TAM test script exists)
python3 /workspace/colcon_ws/test_tam.py
```

### Python Package Development

For faster iteration on Python-based packages (trajectory_manager, control nodes):

```bash
# Use symlink-install to avoid rebuilding after Python changes
colcon build --symlink-install --packages-select stonefish_trajectory_manager

# Changes to .py files take effect immediately (no rebuild needed)
# Only rebuild if you modify package.xml, setup.py, or add new files
```

### Modifying Stonefish Core

If you need to modify the Stonefish C++ library (shaders, physics, sensors):

```bash
# Make changes in /workspace/stonefish/
cd /workspace/stonefish/build
make -j$(nproc)
sudo make install

# Rebuild ROS2 packages that depend on Stonefish
cd /workspace/colcon_ws
colcon build --packages-select stonefish_ros2
```

## Documentation

### Package Documentation
- [stonefish_msgs](stonefish_msgs/README.md) - Core messages and services
- [stonefish_description](stonefish_description/README.md) - Robot models and scenarios
- [stonefish_ros2](stonefish_ros2/README.md) - Simulator interface
- [stonefish_control](stonefish_control/README.md) - Control system overview
  - [stonefish_thruster_manager](stonefish_control/stonefish_thruster_manager/README.md)
  - [stonefish_control](stonefish_control/stonefish_control/README.md)
  - [stonefish_trajectory_manager](stonefish_control/stonefish_trajectory_manager/README.md)

### External Resources
- **Stonefish Library**: https://stonefish.readthedocs.io
- **ROS2 Humble**: https://docs.ros.org/en/humble/
- **Marine Craft Control (Fossen 2011)**: Reference for guidance and control algorithms
- **Original UUV Simulator**: https://github.com/uuvsimulator/uuv_simulator (control system inspiration)

### Internal Documentation
- [CLAUDE.md](/workspace/CLAUDE.md) - Architecture overview and development guidelines

## Troubleshooting

### Build Issues

**Problem**: `libStonefish.so not found`
```bash
# Solution: Install Stonefish library
cd /workspace/stonefish/build
sudo make install

# Verify installation
ls -l /usr/local/lib/libStonefish.so
```

**Problem**: Message dependencies not found
```bash
# Solution: Build messages first
colcon build --packages-select stonefish_msgs stonefish_control_msgs
source install/setup.bash
colcon build
```

**Problem**: SDL2 CMake configuration error
```bash
# Solution: Edit SDL2 config file
sudo nano /usr/lib/x86_64-linux-gnu/cmake/SDL2/sdl2-config.cmake
# Remove space after "-lSDL2" in the file
```

### Runtime Issues

**Problem**: Black screen or rendering failure
- **Cause**: OpenGL 4.3 not supported or driver issue
- **Solution**: Install official GPU drivers (NVIDIA/AMD proprietary)
```bash
# For NVIDIA
sudo ubuntu-drivers autoinstall
sudo reboot

# Verify OpenGL version
glxinfo | grep "OpenGL version"
# Should show 4.3 or higher
```

**Problem**: Simulator crashes immediately
- **Cause**: Missing scenario file or malformed XML
- **Solution**: Verify scenario path exists
```bash
ls /workspace/colcon_ws/src/stonefish_sim/stonefish_description/scenarios/
# Check if your scenario file is listed
```

**Problem**: Thruster allocation not working
- **Cause**: TAM.yaml file missing or malformed
- **Solution**: Check TAM file exists and is valid YAML
```bash
cat /workspace/colcon_ws/src/stonefish_sim/stonefish_description/data/robots/bluerov2/config/TAM.yaml
# Should show valid YAML matrix
```

**Problem**: Topics not publishing
- **Cause**: Node not started or wrong namespace
- **Solution**: Check running nodes
```bash
ros2 node list
ros2 topic list
# Verify your vehicle namespace appears
```

### Performance Issues

**Problem**: Slow simulation (<50 Hz)
- **Cause**: GPU overload or insufficient resources
- **Solution**: Reduce rendering quality
```bash
ros2 launch stonefish_ros2 bluerov2.launch.py rendering_quality:=medium
# Or use nogpu version for headless operation
```

## License

This project is licensed under the **GNU General Public License v3.0**.

The control system packages (`stonefish_control/*`) are derived from the [UUV Simulator](https://github.com/uuvsimulator/uuv_simulator) project and maintain its Apache 2.0 license.

## Maintainer

**HERO Lab, POSTECH**
- Email: luckkim123@gmail.com
- GitHub: https://github.com/HERO-Lab-POSTECH

## Credits

- **Stonefish Simulator**: Patryk Cieslak (University of Girona)
- **UUV Simulator**: IQUA Robotics
- **BlueROV2 Model**: Blue Robotics
- **Control Algorithms**: Based on "Handbook of Marine Craft Hydrodynamics and Motion Control" by Thor I. Fossen (2011)

## References

- Cieslak, P., Ridao, P., & Giergiel, M. (2019). "Stonefish: An Advanced Open-Source Simulation Tool Designed for Marine Robotics." *MTS/IEEE OCEANS Seattle*.
- Fossen, T. I. (2011). "Handbook of Marine Craft Hydrodynamics and Motion Control." John Wiley & Sons.
- Manhães, M. M. M., et al. (2016). "UUV Simulator: A Gazebo-based package for underwater intervention and multi-robot simulation." *MTS/IEEE OCEANS Monterey*.
