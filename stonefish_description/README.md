# stonefish_description

Stonefish 해양 로봇 시뮬레이터를 위한 통합 description 패키지입니다. 로봇, 월드, 모델, 시나리오 정의를 포함합니다.

## Features

- 모듈화된 SCN 파일 구조 (재사용 가능)
- argument 기반 위치 지정 (rpy, xyz)
- 로봇별 리소스 관리 (meshes, textures, SCN)
- 공통 설정 파일 (environment, materials, visual)
- 재사용 가능한 모델 라이브러리

## Directory Structure

```
stonefish_description/
├── robots/              # 로봇 정의
│   ├── girona500/
│   │   ├── meshes/             (hull.obj, propeller.obj 등)
│   │   ├── textures/           (*.png)
│   │   └── girona500.scn       (looks, robot 정의)
│   ├── sparus2/
│   ├── manipulators/
│   │   └── eca5emicro/
│   └── urdf/
├── worlds/              # 월드 정의
│   ├── common/                 (environment, materials, visual)
│   ├── tank.scn
│   ├── seabed.scn
│   └── valve_turning.scn
├── models/              # 재사용 가능한 모델
│   ├── tank/
│   │   ├── meshes/
│   │   └── tank.scn            (look, static 정의, arg: rpy/xyz)
│   ├── rock/
│   │   ├── meshes/
│   │   ├── textures/
│   │   └── rock.scn
│   └── valve/
│       ├── textures/
│       └── valve_panel.scn     (look, robot 정의, arg: rpy/xyz)
├── scenarios/           # 최종 시뮬레이션 조합
│   ├── girona500_tank.scn
│   ├── girona500_seabed.scn
│   ├── girona500_valve_turning.scn
│   └── sparus2_tank.scn
└── launch/              # ROS launch 파일
```

## Usage

### Basic Launch

```bash
# 기본 시뮬레이터 실행
roslaunch stonefish_ros simulator.launch

# 특정 시나리오 실행
roslaunch stonefish_description girona500_tank_simulation.launch
roslaunch stonefish_description girona500_valve_turning_simulation.launch
roslaunch stonefish_description sparus2_tank_simulation.launch
```

### Scenario File Structure

시나리오 파일은 world와 robot을 조합하여 정의합니다.

```xml
<!-- scenarios/girona500_tank.scn -->
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

### World File Structure

월드 파일은 environment와 모델을 조합합니다.

```xml
<!-- worlds/tank.scn -->
<?xml version="1.0"?>
<scenario>
    <!-- 공통 설정 -->
    <include file="worlds/common/environment.scn"/>
    <include file="worlds/common/materials.scn"/>
    <include file="worlds/common/visual.scn"/>

    <!-- Tank 모델 -->
    <include file="models/tank/tank.scn">
        <arg name="rpy" value="0.0 0.0 0.0"/>
        <arg name="xyz" value="0.0 0.0 0.0"/>
    </include>

    <!-- Seabed -->
    <static name="SeaBed" type="plane">
        <material name="Sand"/>
        <look name="sand"/>
        <world_transform rpy="0.0 0.0 0.0" xyz="0.0 0.0 5.2"/>
    </static>
</scenario>
```

### Robot File Structure

로봇 파일은 looks와 robot 정의를 포함합니다.

```xml
<!-- robots/girona500/girona500.scn -->
<?xml version="1.0"?>
<scenario>
    <looks>
        <look name="propeller" texture="robots/girona500/textures/propeller_tex.png"/>
        ...
    </looks>

    <robot name="girona500" fixed="false">
        <base_link name="Vehicle" type="compound">
            <external_part name="Hull" type="model">
                <physical>
                    <mesh filename="robots/girona500/meshes/hull_phy.obj"/>
                    ...
                </physical>
                ...
            </external_part>
        </base_link>

        <sensor name="dvl" type="dvl">...</sensor>
        <actuator name="thruster" type="thruster">...</actuator>

        <world_transform rpy="$(arg rpy)" xyz="$(arg xyz)"/>
    </robot>
</scenario>
```

## Available Robots

- **girona500** - GIRONA500 AUV
- **girona500_eca5emicro** - GIRONA500 + ECA 5E Micro 매니퓰레이터
- **sparus2** - SPARUS2 AUV

## Available Worlds

- **tank** - CIRS 수조 환경
- **seabed** - 해저 환경 (바위 포함)
- **valve_turning** - 밸브 조작 환경

## Available Models

- **tank** - CIRS 수조 모델
- **rock** - 바위 모델
- **valve** - 밸브 패널 (회전 가능)

## Materials

- **Neutral** - 중성 부력 (물과 같은 밀도)
- **Rock** - 바위 (3000 kg/m³)
- **Fiberglass** - 섬유유리 (1500 kg/m³)
- **Aluminium** - 알루미늄 (2710 kg/m³)
- **Sand** - 모래 (1600 kg/m³)
- **Seabed** - 해저면 (1800 kg/m³)
- **Concrete** - 콘크리트 (2400 kg/m³)

## Visual Looks

### 기본 색상
- black, white, yellow, gray

### 해양 환경
- sand (밝은 모래색)
- dark_sand (어두운 모래색)
- seabed (해저 갈색)

## Dependencies

- stonefish_ros
- ROS Noetic

## ROS Topics

### GIRONA500

**Published Topics (Sensors):**
- `/girona500/dynamics/odometry` (nav_msgs/Odometry) - Ground truth 절대 위치/속도
- `/girona500/navigator/dvl_sim` (stonefish_msgs/DVL) - DVL 속도 측정
- `/girona500/navigator/altitude` (sensor_msgs/Range) - DVL 고도
- `/girona500/navigator/imu` (sensor_msgs/Imu) - IMU 데이터
- `/girona500/navigator/pressure` (sensor_msgs/FluidPressure) - 압력 센서 (수심)
- `/girona500/navigator/gps` (sensor_msgs/NavSatFix) - GPS (수면 위에서만)
- `/girona500/camera_front/image_color` (sensor_msgs/Image) - 전방 카메라
- `/girona500/fls/image` (sensor_msgs/Image) - Forward-Looking Sonar
- `/girona500/controller/thruster_state` (stonefish_msgs/ThrusterState) - 추진기 상태

**Subscribed Topics (Actuators):**
- `/girona500/controller/thruster_setpoints_sim` (std_msgs/Float64MultiArray) - 추진기 명령

**Thruster Order:**
```
[ThrusterSurgePort, ThrusterSurgeStarboard, ThrusterHeaveBow, ThrusterHeaveStern, ThrusterSway]
```

### SPARUS2

**Published Topics:**
- `/sparus2/dynamics/odometry` (nav_msgs/Odometry) - Ground truth 절대 위치/속도
- `/sparus2/navigator/dvl_sim` (stonefish_msgs/DVL) - DVL 속도
- `/sparus2/navigator/altitude` (sensor_msgs/Range) - 고도
- `/sparus2/navigator/imu` (sensor_msgs/Imu) - IMU
- `/sparus2/navigator/pressure` (sensor_msgs/FluidPressure) - 압력
- `/sparus2/navigator/gps` (sensor_msgs/NavSatFix) - GPS
- `/sparus2/controller/thruster_state` (stonefish_msgs/ThrusterState) - 추진기 상태

**Subscribed Topics:**
- `/sparus2/controller/thruster_setpoints_sim` (std_msgs/Float64MultiArray) - 추진기 명령

**Thruster Order:**
```
[ThrusterHeave, ThrusterSurgeStarboard, ThrusterSurgePort]
```

### 예제: 로봇 제어

```bash
# GIRONA500 제어 (Thruster order: [SurgePort, SurgeStarboard, HeaveBow, HeaveStern, Sway])
# 주의: inverted_setpoint="true" 설정으로 인해 양수=역방향, 음수=정방향

# 전진
rostopic pub /girona500/controller/thruster_setpoints_sim std_msgs/Float64MultiArray \
  "data: [-0.5, -0.5, 0.0, 0.0, 0.0]"

# 후진
rostopic pub /girona500/controller/thruster_setpoints_sim std_msgs/Float64MultiArray \
  "data: [0.5, 0.5, 0.0, 0.0, 0.0]"

# 왼쪽 (port)
rostopic pub /girona500/controller/thruster_setpoints_sim std_msgs/Float64MultiArray \
  "data: [0.0, 0.0, 0.0, 0.0, 0.5]"

# 오른쪽 (starboard)
rostopic pub /girona500/controller/thruster_setpoints_sim std_msgs/Float64MultiArray \
  "data: [0.0, 0.0, 0.0, 0.0, -0.5]"

# 위로 (상승)
rostopic pub /girona500/controller/thruster_setpoints_sim std_msgs/Float64MultiArray \
  "data: [0.0, 0.0, -0.5, -0.5, 0.0]"

# 아래로 (하강)
rostopic pub /girona500/controller/thruster_setpoints_sim std_msgs/Float64MultiArray \
  "data: [0.0, 0.0, 0.5, 0.5, 0.0]"

# 정지
rostopic pub /girona500/controller/thruster_setpoints_sim std_msgs/Float64MultiArray \
  "data: [0.0, 0.0, 0.0, 0.0, 0.0]"

# 위치 모니터링
rostopic echo /girona500/dynamics/odometry/pose/pose/position
```

## Version

1.0.1 (2025-10-17)
