# CHANGELOG

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
