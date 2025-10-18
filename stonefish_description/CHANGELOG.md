# CHANGELOG

## [1.0.3] - 2025-10-18

### Added
- **BlueROV2 로봇 지원 추가**
  - 6개 thruster 구성 (URDF 기반)
  - Material별 메시 분리 및 최적화 (307,785 → 46,164 faces, 85% 감소)
  - Physical mesh: 15개 부위별 분리 (bluerov2_material_*_simplified.obj)
  - Propeller mesh: bluerov2_propcw.obj, bluerov2_propccw.obj
  - ROS topics: `/bluerov2/thruster_manager/input`, `/bluerov2/dynamics/odometry`, `/bluerov2/thrusters/state`
  - 두 가지 시나리오 제공:
    - `bluerov2_tank.scn` - 기본 단색 버전
    - `bluerov2_tank_multipart.scn` - 부위별 색상 버전 (15개 material)
  - Launch 파일: `bluerov2_tank_simulation.launch`, `bluerov2_tank_multipart_simulation.launch`
- **메시 처리 도구**
  - `split_and_simplify.py` - OBJ material 분리 및 단순화 자동화
  - Fast-Quadric-Mesh-Simplification 통합
- README.md에 BlueROV2 설정 및 사용법 추가

### Technical Details
- **Thruster 구성 (원본 bluerov2 thrusters.xacro 기반):**
  - Thruster0: 전방 우측 (xyz: 0.1355, -0.1, -0.0725, yaw: 45°)
  - Thruster1: 전방 좌측 (xyz: 0.1355, 0.1, -0.0725, yaw: -45°)
  - Thruster2: 후방 우측 (xyz: -0.1475, -0.1, -0.0725, yaw: 135°)
  - Thruster3: 후방 좌측 (xyz: -0.1475, 0.1, -0.0725, yaw: -135°)
  - Thruster4: 중앙 우측 수직 (xyz: 0.0025, -0.1105, -0.005, pitch: -90°)
  - Thruster5: 중앙 좌측 수직 (xyz: 0.0025, 0.1105, -0.005, pitch: -90°)
- **메시 최적화 (전체 15% 단순화):**
  - material_10 (메인 프레임): 122,722 → 18,408 faces
  - material_4 (전자장비 통): 99,948 → 14,992 faces
  - material_5 (LED 마커): 16,801 → 2,520 faces
  - material_15 (파란 부품): 13,368 → 2,004 faces
  - material_3: 10,512 → 1,577 faces
  - 기타 10개 부위: 356~975 faces
  - **총 46,164 faces (원본 대비 85% 감소)**
- **부위별 색상 (multipart 버전):**
  - 메인 프레임: 어두운 회색 (RGB: 0.098)
  - 전자장비 통: 베이지색 (RGB: 0.662, 0.645, 0.629)
  - LED 마커: 빨간색 (RGB: 1.0, 0.0, 0.0)
  - 파란 부품: 청록색 (RGB: 0.149, 0.6, 0.816)
  - 기타 부위: 회색/파란색 톤

## [1.0.2] - 2025-10-18

### Added
- **RexROV2 로봇 지원 추가**
  - 6개 thruster 구성 (원본 URDF 기반)
  - Physical mesh: box_simple.obj (scale 1.2)
  - Visual mesh: simplify_RexROV2_no_props.obj
  - Propeller mesh: prop.obj
  - 부력 시스템 (Buoyancy + Battery)
  - FLS 센서 추가
  - ROS topics: `/rexrov2/thruster_manager/input`, `/rexrov2/dynamics/odometry`
- README.md에 RexROV2 설정 및 사용법 추가

### Technical Details
- **Thruster 구성 (원본 rexrov2_actuators.xacro 기반):**
  - Thruster0: 전방 측면 (xyz: 0.4878, 0, 0.2373, yaw: 90°)
  - Thruster1: 후방 상단 (xyz: -0.8217, 0, 0.589, pitch: -75°)
  - Thruster2: 전방 우측 (xyz: 0.8654, -0.5322, 0.5332, pitch: -67.5°, yaw: 90°)
  - Thruster3: 전방 좌측 (xyz: 0.8654, 0.5322, 0.5332, pitch: 67.5°, yaw: 90°)
  - Thruster4: 후방 우측 (xyz: -0.7076, 0.5129, 0.2404, yaw: 25°)
  - Thruster5: 후방 좌측 (xyz: -0.7076, -0.5129, 0.2404, yaw: -25°)
- **부력 설정:**
  - Buoyancy cylinder: radius 0.6m, height 3.0m, mass 1kg, buoyant=true
  - Battery cylinder: radius 0.15m, height 0.5m, mass 150kg, buoyant=false

## [1.0.1] - 2025-10-17

### Fixed
- README.md의 추진기 메시지 타입 수정: `cola2_msgs/Setpoints` → `std_msgs/Float64MultiArray`
- 제어 예제 명령어 수정 (올바른 메시지 타입 및 필드명 적용)
- 제어 방향 수정: 왼쪽/오른쪽, 위/아래 명령어 값 반전

## [1.0.0] - 2025-10-17

### Added
- 통합 description 패키지 생성
- 모듈화된 SCN 파일 구조
- argument 기반 world_transform (rpy, xyz)
- 재사용 가능한 models/ 시스템
- 해양 환경 재질 추가 (Sand, Seabed, Concrete)
- 해양 색상 추가 (sand, dark_sand, seabed)
- 공통 설정 파일 (environment, materials, visual)

### Changed
- cola2_stonefish 패키지를 stonefish_description으로 통합 및 재구성
- 로봇/월드/모델/시나리오 명확한 계층 분리
- meshes/, textures/ 폴더로 리소스 구조화
- 모든 SCN 파일에 looks 포함 (visual.scn 제거)
- 상대 경로 기반 include 구조로 전환

### Removed
- data/ 폴더 (불필요한 중복 제거)
- visual.scn 분리 파일들 (SCN에 직접 통합)
- test 재질 (Sand, Seabed로 대체)

### Structure
```
stonefish_description/
├── robots/          # 로봇 정의 (meshes, textures, SCN with looks)
├── worlds/          # 월드 정의 (common, SCN files)
├── models/          # 재사용 모델 (각 모델별 SCN with looks)
├── scenarios/       # 최종 조합 (world + robot)
└── launch/          # Launch 파일
```

### Migration Notes
- `$(arg position)` → `$(arg rpy)` + `$(arg xyz)`로 분리
- 모든 entity/robot에 argument 기반 위치 지정
- simulation_data 경로: `$(find stonefish_description)`
