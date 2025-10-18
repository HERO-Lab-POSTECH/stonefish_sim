# Stonefish ROS Workspace

Stonefish 해양 로봇 시뮬레이터를 위한 ROS 통합 워크스페이스입니다. 물리 기반 해양 로봇 시뮬레이션을 위한 완전한 ROS 인터페이스를 제공합니다.

## Overview

이 워크스페이스는 [Stonefish](https://github.com/patrykcieslak/stonefish) C++ 시뮬레이션 라이브러리를 ROS와 통합하여 GIRONA500, SPARUS2, RexROV2 등 다양한 해양 로봇의 시뮬레이션을 지원합니다.

**주요 기능:**
- 물리 기반 해양 로봇 시뮬레이션 (Bullet Physics)
- OpenGL 기반 그래픽 렌더링
- 다양한 센서 지원 (DVL, IMU, 압력, 카메라, 소나 등)
- 모듈화된 시나리오 정의 (XML)
- ROS 토픽 기반 센서 데이터 및 제어 인터페이스

## Repository Structure

```
catkin_ws/src/
├── stonefish_description/    # 로봇, 월드, 시나리오 정의
├── stonefish_msgs/           # ROS 메시지 및 서비스 정의
└── stonefish_ros/            # ROS 시뮬레이터 노드
```

### Packages

#### stonefish_description
로봇, 월드, 모델, 시나리오 정의를 포함하는 통합 description 패키지입니다.

**지원 로봇:**
- GIRONA500 AUV
- GIRONA500 + ECA 5E Micro (매니퓰레이터)
- SPARUS2 AUV
- RexROV2 ROV

**환경:**
- CIRS 수조 환경
- 해저 환경 (바위 포함)
- 밸브 조작 환경

[자세한 내용은 stonefish_description/README.md 참조](stonefish_description/README.md)

#### stonefish_msgs
Stonefish 시뮬레이터와 COLA2 프로젝트를 위한 ROS 메시지 및 서비스 정의입니다.

**포함 메시지:**
- DVL, IMU, 추진기 상태 등 해양 로봇 전용 메시지
- USBL 비콘 정보, 소나 설정 서비스

[자세한 내용은 stonefish_msgs/README.md 참조](stonefish_msgs/README.md)

#### stonefish_ros
Stonefish C++ 라이브러리와 ROS 간의 인터페이스 패키지입니다.

**제공 노드:**
- `parsed_simulator` - 그래픽 시뮬레이터
- `parsed_simulator_nogpu` - 헤드리스 시뮬레이터

[자세한 내용은 stonefish_ros/README.md 참조](stonefish_ros/README.md)

## Requirements

### Hardware Requirements
- **CPU**: Modern multi-core processor (for realtime physics simulation)
- **GPU**: Discrete graphics card with **OpenGL 4.3+** support (for graphical simulations)

### System Requirements
- **OS**: Ubuntu 20.04 (ROS Noetic)
- **Graphics Drivers**: Official manufacturer drivers for your GPU

> **Warning**: Install official GPU drivers before using Stonefish! Generic drivers may not support OpenGL 4.3.

### Software Dependencies

**ROS Packages:**
- ROS Noetic
- sensor_msgs
- geometry_msgs
- nav_msgs
- std_msgs

**External Libraries:**
- Stonefish library >= 1.5.0
- OpenGL Mathematics (libglm-dev, >= 0.9.9.0)
- SDL2 (libsdl2-dev)
- FreeType (libfreetype6-dev)

## Installation

### 1. Install System Dependencies

```bash
sudo apt-get update
sudo apt-get install -y libglm-dev libsdl2-dev libfreetype6-dev
```

> **Note**: SDL2 library may need a small fix to the CMake configuration file. Remove a space after `-lSDL2` in `/usr/lib/x86_64-linux-gnu/cmake/SDL2/sdl2-config.cmake`.

### 2. Install Stonefish Library

**IMPORTANT:** HERO Lab에서 유지관리하는 Stonefish 라이브러리를 먼저 설치해야 합니다.

```bash
# Clone HERO Lab Stonefish repository
git clone https://github.com/HERO-Lab-POSTECH/stonefish.git
cd stonefish

# Build and install
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

**Build Options:**
- `BUILD_TESTS=ON` - Build test applications and examples
- `EMBED_RESOURCES=ON` - Embed resources into binary (for redistribution)

Example with tests:
```bash
cmake -DBUILD_TESTS=ON ..
make -j$(nproc)
# Tests will be in build/Tests/ directory
```

### 3. Clone and Build ROS Workspace

```bash
# Clone the workspace
git clone https://github.com/HERO-Lab-POSTECH/stonefish_sim.git
cd stonefish_sim

# Build all packages
catkin_make

# Source workspace
source devel/setup.bash
```

## Quick Start

### Basic Simulation

```bash
# Source workspace
source /workspace/catkin_ws/devel/setup.bash

# Launch GIRONA500 in tank environment
roslaunch stonefish_description girona500_tank_simulation.launch

# Launch SPARUS2 in tank environment
roslaunch stonefish_description sparus2_tank_simulation.launch

# Launch GIRONA500 valve turning scenario
roslaunch stonefish_description girona500_valve_turning_simulation.launch
```

### Custom Simulation

```bash
# Launch with custom scenario
roslaunch stonefish_ros simulator.launch \
    simulation_data:=$(rospack find stonefish_description) \
    scenario_description:=$(rospack find stonefish_description)/scenarios/girona500_tank.scn \
    simulation_rate:=500 \
    graphics_quality:=high
```

### Headless Simulation (No GPU)

```bash
roslaunch stonefish_ros simulator_nogpu.launch \
    simulation_data:=$(rospack find stonefish_description) \
    scenario_description:=$(rospack find stonefish_description)/scenarios/girona500_tank.scn \
    simulation_rate:=500
```

## Robot Control Examples

### GIRONA500 Control

```bash
# 전진
rostopic pub /girona500/controller/thruster_setpoints_sim std_msgs/Float64MultiArray \
  "data: [-0.5, -0.5, 0.0, 0.0, 0.0]"

# 위로 (상승)
rostopic pub /girona500/controller/thruster_setpoints_sim std_msgs/Float64MultiArray \
  "data: [0.0, 0.0, 0.5, 0.5, 0.0]"

# 정지
rostopic pub /girona500/controller/thruster_setpoints_sim std_msgs/Float64MultiArray \
  "data: [0.0, 0.0, 0.0, 0.0, 0.0]"
```

### Monitor Robot State

```bash
# 위치 확인
rostopic echo /girona500/dynamics/odometry

# DVL 속도 확인
rostopic echo /girona500/navigator/dvl_sim

# IMU 데이터 확인
rostopic echo /girona500/navigator/imu

# 카메라 영상 확인
rosrun image_view image_view image:=/girona500/camera_front/image_color
```

## ROS Topics

### Published Topics (Sensors)
- `/robot_name/dynamics/odometry` (nav_msgs/Odometry) - Ground truth 위치/속도
- `/robot_name/navigator/dvl_sim` (stonefish_msgs/DVL) - DVL 속도
- `/robot_name/navigator/altitude` (sensor_msgs/Range) - 고도
- `/robot_name/navigator/imu` (sensor_msgs/Imu) - IMU 데이터
- `/robot_name/navigator/pressure` (sensor_msgs/FluidPressure) - 압력 (수심)
- `/robot_name/navigator/gps` (sensor_msgs/NavSatFix) - GPS
- `/robot_name/camera_front/image_color` (sensor_msgs/Image) - 카메라
- `/robot_name/fls/image` (sensor_msgs/Image) - Forward-Looking Sonar

### Subscribed Topics (Actuators)
- `/robot_name/controller/thruster_setpoints_sim` (std_msgs/Float64MultiArray) - 추진기 명령

## Creating Custom Scenarios

시나리오는 XML 형식으로 정의되며, world와 robot을 조합하여 구성합니다.

```xml
<?xml version="1.0"?>
<scenario>
    <!-- World -->
    <include file="worlds/tank.scn"/>

    <!-- Robot -->
    <include file="robots/girona500/girona500.scn">
        <arg name="rpy" value="0.0 0.0 0.0"/>
        <arg name="xyz" value="0.0 0.0 1.0"/>
    </include>
</scenario>
```

자세한 내용은 [stonefish_description/README.md](stonefish_description/README.md)를 참조하세요.

## Development

### Building

```bash
cd /workspace/catkin_ws
catkin_make
```

### Running Tests

```bash
# Source workspace
source devel/setup.bash

# Run specific scenario
roslaunch stonefish_description girona500_tank_simulation.launch
```

## Documentation

- [Stonefish Documentation](https://stonefish.readthedocs.io)
- [Stonefish GitHub](https://github.com/patrykcieslak/stonefish)
- [COLA2 Project](https://github.com/iquarobotics/cola2)

## Version History

- **1.0.2** (2025-10-18) - RexROV2 로봇 지원 추가
- **1.0.1** (2025-10-17) - 패키지 구조 개편
- **1.0.0** (2025-10-17) - 초기 릴리스

## License

GNU General Public License v3.0

## Maintainer

HERO Lab, POSTECH

## References

- Stonefish: Patryk Cieslak <patryk.cieslak@udg.edu>
- COLA2: IQUA Robotics
