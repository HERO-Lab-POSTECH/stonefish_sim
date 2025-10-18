# CHANGELOG

## [1.0.4] - 2025-10-19

### Fixed
- **BlueROV2 추진기 시스템 최적화**
  - Thruster1 inverted 설정 수정 (true → false)
  - Thruster0, Thruster2, Thruster3 inverted 설정 수정 (일부 조정)
  - Propeller diameter: 0.076m → 0.18m (GIRONA500과 동일)
  - Thrust coefficient: 0.96 → 0.48 (안정적인 추진력)
  - Rotor inertia 추가: 0.05 (rpm 안정화)
  - Rotor dynamics: kp=1.0, ki=0.5, ilimit=2.0 (PI controller 안정화)
  - Odometry sensor의 ros_publisher를 sensor 태그 내부로 이동
- **BlueROV2 제어 명령 문서화**
  - README.md에 전 방향 이동 예제 추가 (전진/후진/상승/하강/회전)
  - 실제 테스트를 통한 검증된 제어 명령

### Technical Details
- **추진기 파라미터 최종 설정:**
  - Max RPM: 1000 (1000.0/60.0*2.0*pi rad/s)
  - Propeller: diameter=0.18m, T200 메시 사용
  - Thrust model: fluid_dynamics, coeff=0.48
  - Rotor dynamics: mechanical_pi, rotor_inertia=0.05
  - 안정적인 제어 응답, oscillation 제거

## [1.0.3] - 2025-10-18

### Added
- **BlueROV2 로봇 지원 추가**
  - 6개 T200 thruster 구성 (원본 URDF 기반)
  - 5개 부위별 색상 구현 (Duct, BuoyCover, Plate, Hull, HullCover)
  - Physical/Visual 메시 분리 방식:
    - Physical: 부위별 bounding box (각 12 faces, 총 60 faces)
    - Visual: Blender 최적화 메시 (총 146,590 faces)
  - Propeller: T200 실제 크기 (0.076m 직경)
  - 부력/질량 시스템:
    - BuoyancyBox (internal, buoyant=true): 9.408kg 부력
    - MassCylinder (internal, buoyant=false): 9.2kg 질량
    - 1% 양의 부력 (중성 부력)
  - ROS topics: `/bluerov2/thruster_manager/input`, `/bluerov2/dynamics/odometry`, `/bluerov2/thrusters/state`
- **메시 처리 도구**
  - `create_physical_boxes.py` - 각 부품의 정확한 크기 physical box 자동 생성
- README.md에 BlueROV2 설정 및 사용법 추가

### Technical Details
- **Thruster 구성 (원본 bluerov2 thrusters.xacro 기반):**
  - Thruster0: 전방 우측 (xyz: 0.1355, -0.1, -0.0725, yaw: 45°)
  - Thruster1: 전방 좌측 (xyz: 0.1355, 0.1, -0.0725, yaw: -45°)
  - Thruster2: 후방 우측 (xyz: -0.1475, -0.1, -0.0725, yaw: 135°)
  - Thruster3: 후방 좌측 (xyz: -0.1475, 0.1, -0.0725, yaw: -135°)
  - Thruster4: 중앙 우측 수직 (xyz: 0.0025, -0.1105, -0.005, pitch: -90°)
  - Thruster5: 중앙 좌측 수직 (xyz: 0.0025, 0.1105, -0.005, pitch: -90°)
- **메시 구조:**
  - Duct (덕트/프레임): 어두운 회색, 122,722 faces (visual)
  - BuoyCover (부력 커버): 청록색, 13,368 faces (visual)
  - Plate (플레이트): 매우 어두운 회색, 7,124 faces (visual)
  - Hull (선체): 밝은 회색, 2,318 faces (visual)
  - HullCover (선체 커버): 회색, 1,058 faces (visual)
  - Propeller: propcw.obj, propccw.obj (76mm 직경)
- **부력/질량 설계:**
  - COB (부력 중심): z = 0.03m (위쪽)
  - COG (질량 중심): z ≈ -0.14m (아래)
  - 차이 0.17m로 안정적인 자세 유지

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
