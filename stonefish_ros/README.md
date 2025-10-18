# stonefish_ros

Stonefish 물리 시뮬레이션 라이브러리를 위한 ROS 통합 패키지입니다.

## Overview

이 패키지는 Stonefish C++ 시뮬레이션 라이브러리와 ROS 간의 인터페이스를 제공합니다. 해양 로봇의 센서 데이터를 ROS 토픽으로 퍼블리시하고, 액추에이터 명령을 ROS 토픽으로 구독합니다.

## Dependencies

- **Stonefish** >= 1.5.0 (C++ 라이브러리)
- **stonefish_msgs** - ROS 메시지 정의
- ROS Noetic
- OpenGL 4.3+

## Changes (2025-10-17)

### Modified
- 메시지 정의를 stonefish_msgs 패키지로 분리
- msg/, srv/ 폴더 제거
- stonefish_msgs 의존성 추가
- CMakeLists.txt 및 package.xml 업데이트

## Nodes

### `parsed_simulator`
- **Type**: Graphical simulator
- **Description**: XML 시나리오 파일을 파싱하여 그래픽 기반 시뮬레이션 실행

### `parsed_simulator_nogpu`
- **Type**: Headless simulator
- **Description**: GPU 없이 헤드리스 모드로 시뮬레이션 실행

## Launch Files

### `simulator.launch`

```bash
roslaunch stonefish_ros simulator.launch \
    simulation_data:=$(find stonefish_description) \
    scenario_description:=$(find stonefish_description)/scenarios/girona500_tank.scn \
    simulation_rate:=500 \
    graphics_resolution:="1200 800" \
    graphics_quality:=high
```

**Parameters:**
- `simulation_data` - 메시/텍스처 리소스 경로
- `scenario_description` - 시나리오 SCN 파일 경로
- `simulation_rate` - 시뮬레이션 Hz (default: 500)
- `graphics_resolution` - 윈도우 해상도 (default: 1200 800)
- `graphics_quality` - low/medium/high (default: high)

### `simulator_nogpu.launch`

헤드리스 모드 실행 (그래픽 없음)

```bash
roslaunch stonefish_ros simulator_nogpu.launch \
    simulation_data:=$(find stonefish_description) \
    scenario_description:=$(find stonefish_description)/scenarios/girona500_tank.scn \
    simulation_rate:=500
```

## Integration with stonefish_description

이 패키지는 `stonefish_description`과 함께 사용됩니다:

```bash
# 1. 빌드
cd /workspace/catkin_ws
catkin_make

# 2. 소스
source devel/setup.bash

# 3. 실행
roslaunch stonefish_ros simulator.launch
```

## Message Interface

모든 센서/액추에이터 메시지는 `stonefish_msgs` 패키지에 정의되어 있습니다.

**Published Topics:**
- `/robot_name/dynamics/odometry` (nav_msgs/Odometry)
- `/robot_name/navigator/dvl` (stonefish_msgs/DVL)
- `/robot_name/navigator/imu` (sensor_msgs/Imu)
- `/robot_name/camera_front` (sensor_msgs/Image)
- 기타...

**Subscribed Topics:**
- `/robot_name/controller/thruster_setpoints` (cola2_msgs/Setpoints)
- 기타...

## Version

1.5.0 (Stonefish library version)

## License

GPL-3.0

## Maintainer

Patryk Cieslak <patryk.cieslak@udg.edu>
