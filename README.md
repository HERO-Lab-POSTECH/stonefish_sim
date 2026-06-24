# Stonefish Simulation for Marine Robotics

📖 **Full documentation site (Korean): https://hero-lab-postech.github.io/stonefish_sim/**

A ROS2 Humble integration for the Stonefish marine robotics simulator. Provides physics-based simulation for underwater vehicles with realistic sensors and control systems.

## Requirements

### Hardware
- GPU with OpenGL 4.3+ support (NVIDIA GTX 960+ recommended)
- Install official GPU drivers before use

### Software
- Ubuntu 22.04 LTS
- ROS2 Humble
- Stonefish library v1.3.0+

### Dependencies

#### ROS2 packages (apt)

The C++ simulator interface (`stonefish_ros2`) and the message packages pull in
`rclcpp`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`,
`std_srvs`, `std_msgs`, `image_transport`, `tf2`, `tf2_ros`, `PCL` /
`pcl_conversions`, `ament_index_cpp`, and the `rosidl` message generators. The
Python packages additionally require `rclpy`, `tf2_geometry_msgs`, and
`ros2launch`.

```bash
sudo apt update
sudo apt install \
  ros-humble-desktop \
  ros-humble-rclcpp \
  ros-humble-rclpy \
  ros-humble-ament-index-cpp \
  ros-humble-std-msgs \
  ros-humble-std-srvs \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-sensor-msgs \
  ros-humble-visualization-msgs \
  ros-humble-builtin-interfaces \
  ros-humble-image-transport \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-pcl-conversions \
  ros-humble-pcl-ros \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime
```

> `ros-humble-desktop` already provides most of these (rclcpp, rclpy, the
> standard message packages, tf2, image_transport, PCL conversions). They are
> listed explicitly so the dependency set is unambiguous; installing
> `ros-humble-desktop` alone is sufficient on a typical workstation.

#### Python packages (pip)

The Python control, thruster-manager, and trajectory-manager packages import
`numpy`, `scipy`, `transforms3d`, and `PyYAML`. These are also packaged as
`python3-numpy`, `python3-scipy`, `python3-transforms3d`, and `python3-yaml` via
apt if you prefer system packages.

```bash
pip install numpy scipy transforms3d PyYAML
```

#### System / graphics libraries

Stonefish renders with OpenGL and SDL2, so the following native libraries are
required to build and run the simulator:

```bash
sudo apt install libglm-dev libsdl2-dev libfreetype6-dev libopengl-dev
```

## Installation

### 1. Install the Stonefish C++ Library (v1.3.0+)

`stonefish_ros2` calls `find_package(Stonefish REQUIRED 1.3.0)`, so the core
Stonefish library must be built and installed first:

```bash
cd /workspace
git clone https://github.com/HERO-Lab-POSTECH/stonefish.git
cd stonefish && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) && sudo make install
```

### 2. Build the ROS2 Workspace

```bash
cd /workspace/colcon_ws
source /opt/ros/humble/setup.bash

# (optional) resolve declared ROS dependencies from the manifests
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

## Quick Start

```bash
# Launch BlueROV2 simulation
ros2 launch stonefish_ros2 bluerov2.launch.py

# With thruster manager
ros2 launch stonefish_ros2 bluerov2.launch.py start_thruster_manager:=true

# With the controller
ros2 launch stonefish_control control.launch.py
```

## Package Structure

| Package | Description |
|---------|-------------|
| stonefish_msgs | Message/service definitions (DVL, INS, environment control) |
| stonefish_description | Robot models, worlds, scenarios |
| stonefish_ros2 | Core ROS2 simulator interface (C++) |
| stonefish_control | Controllers (position, velocity, hybrid) |
| stonefish_thruster_manager | TAM-based thrust allocation |
| stonefish_trajectory_manager | Path generation and following |

## ROS2 Topics

### Published (by simulator)

| Topic | Type | Description |
|-------|------|-------------|
| `/{vehicle}/odometry` | nav_msgs/Odometry | Ground truth pose/velocity |
| `/{vehicle}/dvl` | stonefish_msgs/DVL | Doppler Velocity Log |
| `/{vehicle}/imu` | sensor_msgs/Imu | IMU data |
| `/{vehicle}/pressure` | sensor_msgs/FluidPressure | Depth sensor |
| `/{vehicle}/fls/image` | sensor_msgs/Image | Forward-Looking Sonar |
| `/{vehicle}/camera_*/image_color` | sensor_msgs/Image | Camera images |

### Subscribed (commands)

| Topic | Type | Description |
|-------|------|-------------|
| `/{vehicle}/thruster_manager/input` | geometry_msgs/Wrench | 6DOF wrench command |
| `/{vehicle}/cmd_pose` | geometry_msgs/PoseStamped | Position setpoint |
| `/{vehicle}/cmd_vel` | geometry_msgs/Twist | Velocity setpoint |

## ROS2 Services

| Service | Type | Description |
|---------|------|-------------|
| `/set_wave_height` | stonefish_msgs/SetWaveHeight | Set wave height (0-10m) |
| `/set_wind_velocity` | stonefish_msgs/SetWindVelocity | Set wind vector |
| `/set_ocean_current` | stonefish_msgs/SetOceanCurrent | Enable/set current |

## Coordinate System

This workspace uses **NED (North-East-Down)** convention:
- X: North (forward)
- Y: East (right)
- Z: Down (into water)
- Depth: Z > 0 means underwater

## Available Scenarios

Located in `stonefish_description/scenarios/`:
- `bluerov2_empty.scn` - Empty ocean
- `bluerov2_infrastructure.scn` - Underwater structures
- `bluerov2_seabed.scn` - Ocean floor terrain
- `bluerov2_shipwreck.scn` - Shipwreck environment

## License

All packages are licensed under GPL-3.0-or-later. The control packages are
derived from the UUV Simulator (originally Apache-2.0); Apache-2.0 is one-way
compatible with GPL-3.0, and the original copyright holders are preserved in
each source file's SPDX header.

## Credits

- Stonefish Simulator: Patryk Cieslak (University of Girona)
- Control System: Based on UUV Simulator
- HERO Lab, POSTECH
