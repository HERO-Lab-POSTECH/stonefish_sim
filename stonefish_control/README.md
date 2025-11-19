# Stonefish Control - ROS2 Control Packages for UUV Simulation

ROS2 기반 수중 로봇(UUV) 제어 패키지 모음. UUV Simulator의 제어 알고리즘을 Stonefish ROS2 시뮬레이터용으로 포팅한 프로젝트입니다.

## 📦 패키지 목록

### ✅ 완성된 패키지

#### 1. **stonefish_control_msgs**
ROS2 제어 메시지 및 서비스 정의
- 4개 메시지: Waypoint, WaypointSet, Trajectory, TrajectoryPoint
- 22개 서비스: 제어기 설정, waypoint 관리, trajectory 생성 등

#### 2. **stonefish_thruster_manager**
추력기 할당 및 관리
- TAM (Thruster Allocation Matrix) 기반 추력 할당
- Wrench (6DOF 힘/토크) → Thruster commands 변환
- Thruster models (Proportional, Custom)
- BlueROV2, Girona500 등 다양한 로봇 지원

#### 3. **stonefish_control** (구조만 완성)
통합 제어 알고리즘 패키지
- PID 계열 컨트롤러
- Cascaded control
- Advanced control (Sliding Mode, Feedback Linearization 등)

### ⏳ 개발 예정 패키지

- **stonefish_trajectory_manager**: Waypoint 및 trajectory 관리
- **stonefish_control_utils**: 제어 유틸리티 및 시각화
- **stonefish_teleop_manager**: 키보드/조이스틱 원격 조종

## 🚀 빠른 시작

### 1. 빌드

```bash
cd /workspace/colcon_ws

# 스크립트 사용
./build_stonefish_control.sh

# 또는 수동 빌드
colcon build --packages-select \
    stonefish_control_msgs \
    stonefish_thruster_manager \
    stonefish_control
```

### 2. 환경 설정

```bash
source /workspace/colcon_ws/install/setup.bash
```

### 3. Thruster Allocator 실행

```bash
# BlueROV2 기본 설정
ros2 launch stonefish_thruster_manager thruster_allocator.launch.py \
    vehicle_name:=bluerov2

# 커스텀 TAM 파일 사용
ros2 launch stonefish_thruster_manager thruster_allocator.launch.py \
    vehicle_name:=bluerov2 \
    tam_file:=/path/to/TAM.yaml \
    max_thrust:=150.0 \
    timeout:=1.0
```

### 4. 제어 명령 전송

```bash
# Wrench 명령 발행 (Surge 10N, Yaw 5N⋅m)
ros2 topic pub /bluerov2/thruster_manager/input geometry_msgs/msg/Wrench \
    "{force: {x: 10.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 5.0}}" \
    --once

# 추력기 출력 확인
ros2 topic echo /bluerov2/setpoint/pwm
```

## 🧪 테스트

### TAM 테스트

```bash
# TAM 로딩 및 변환 테스트
python3 /workspace/colcon_ws/test_tam.py
```

**예상 출력**:
```
✅ TAM loaded successfully
   - Number of thrusters: 8
   - TAM shape: (6, 8)

🧪 Test Case 1: Pure Surge (Fx = 10 N)
   Input Wrench: [10.  0.  0.  0.  0.  0.]
   Thrust Forces: [3.536 3.536 3.536 3.536 0.    0.    0.    0.   ]
   Expected: All horizontal thrusters contribute equally

✅ All tests passed!
```

### Python에서 TAM 사용

```python
from stonefish_thruster_manager.thruster_manager import ThrusterManager
import numpy as np

# TAM 로딩
tam_mgr = ThrusterManager(
    tam_file_path='/workspace/colcon_ws/src/stonefish_description/data/robots/bluerov2/config/TAM.yaml'
)

# Wrench → Thrust
wrench = np.array([10, 0, 20, 0, 0, 5])  # [Fx, Fy, Fz, Tx, Ty, Tz]
thrust_forces = tam_mgr.compute_thrust_forces(wrench)
print(f"Thrust forces: {thrust_forces}")

# Thrust → Wrench (검증)
wrench_check = tam_mgr.compute_wrench(thrust_forces)
print(f"Recovered wrench: {wrench_check}")
```

## 📐 TAM (Thruster Allocation Matrix)

### BlueROV2 Heavy Configuration

BlueROV2는 8개의 추력기를 사용합니다:
- **수평 추력기 4개** (T1-T4): 45° 각도로 배치, Surge/Sway/Yaw 제어
- **수직 추력기 4개** (T5-T8): 아래 방향, Heave/Roll/Pitch 제어

TAM 파일 위치:
```
/workspace/colcon_ws/src/stonefish_description/data/robots/bluerov2/config/TAM.yaml
```

### TAM 공식

```
[Fx, Fy, Fz, Tx, Ty, Tz]ᵀ = TAM × [F1, F2, F3, F4, F5, F6, F7, F8]ᵀ
```

역변환 (Pseudo-inverse):
```
[F1, F2, ..., F8]ᵀ = pinv(TAM) × [Fx, Fy, Fz, Tx, Ty, Tz]ᵀ
```

### 새 로봇 추가하기

1. TAM YAML 파일 생성:
```bash
mkdir -p /workspace/colcon_ws/src/stonefish_description/data/robots/my_robot/config
```

2. TAM.yaml 작성:
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

3. Thruster Allocator 실행:
```bash
ros2 launch stonefish_thruster_manager thruster_allocator.launch.py \
    vehicle_name:=my_robot \
    tam_file:=/workspace/colcon_ws/src/stonefish_description/data/robots/my_robot/config/TAM.yaml
```

## 🔧 개발 상태

### 완료 (50%)
- ✅ ROS2 메시지/서비스 시스템
- ✅ TAM 기반 추력기 할당
- ✅ Thruster models
- ✅ 패키지 구조 및 빌드 시스템

### 진행 중 (50%)
- ⏳ Control interfaces (vehicle dynamics, controller base classes)
- ⏳ PID controllers (standard, nonlinear, underactuated)
- ⏳ Cascaded control (position → velocity → acceleration)
- ⏳ Trajectory management
- ⏳ Teleoperation

자세한 진행 상황: `/workspace/migration_progress.md` 참조

## 📊 시스템 아키텍처

```
┌─────────────────────┐
│  User Command       │
│  (Wrench/Waypoint)  │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  Controller         │
│  (PID/Cascaded/SM)  │
└──────────┬──────────┘
           │ Wrench (6DOF)
           ▼
┌─────────────────────┐
│ Thruster Allocator  │
│  (TAM-based)        │
└──────────┬──────────┘
           │ Thrust Array
           ▼
┌─────────────────────┐
│ Stonefish Simulator │
│  (8 Thrusters)      │
└─────────────────────┘
```

## 🔗 토픽 구조

### Thruster Manager
```
입력:
  /bluerov2/thruster_manager/input              (geometry_msgs/Wrench)
  /bluerov2/thruster_manager/input_stamped      (geometry_msgs/WrenchStamped)

출력:
  /bluerov2/setpoint/pwm                        (std_msgs/Float64MultiArray)
```

### 제어기 (향후)
```
입력:
  /bluerov2/odom                                (nav_msgs/Odometry)
  /bluerov2/reference/pose                      (geometry_msgs/PoseStamped)
  /bluerov2/reference/trajectory                (stonefish_control_msgs/Trajectory)

출력:
  /bluerov2/thruster_manager/input              (geometry_msgs/Wrench)
```

## 📚 참고 자료

### 문서
- [UUV Simulator 제어 시스템 분석](/workspace/control_system_analysis.md)
- [Stonefish 시뮬레이터 분석](/workspace/stonefish_control_analysis.md)
- [마이그레이션 진행 상황](/workspace/migration_progress.md)

### 원본 프로젝트
- [UUV Simulator](https://github.com/uuvsimulator/uuv_simulator)
- [Stonefish](https://github.com/patrykcieslak/stonefish)

### ROS2 문서
- [ROS2 Humble](https://docs.ros.org/en/humble/)
- [Migration Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Migration-Guide.html)

## 🤝 기여

이 프로젝트는 UUV Simulator를 ROS2/Stonefish 환경으로 포팅하는 작업입니다.

### 다음 단계
1. control_interfaces 변환
2. 기본 PID 컨트롤러 구현
3. Cascaded control 구현
4. 통합 테스트

## 📝 라이선스

Apache License 2.0

Original UUV Simulator code:
- Copyright (c) 2016-2019 The UUV Simulator Authors
- Licensed under Apache License 2.0

ROS2 포팅:
- Copyright 2025
- Licensed under Apache License 2.0

---

**현재 상태**: 핵심 인프라 완성 (50%), 제어 알고리즘 개발 진행 중

**문의**: migration_progress.md 참조
