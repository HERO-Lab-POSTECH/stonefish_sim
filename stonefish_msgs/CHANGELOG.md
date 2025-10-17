# CHANGELOG

## [1.0.0] - 2025-10-17

### Added
- Stonefish 메시지 통합 패키지 생성
- stonefish_msgs 하위 패키지: Stonefish 시뮬레이터 전용 메시지 (9 msg, 3 srv)
- cola2_msgs 하위 패키지: COLA2 프로젝트 메시지 (30 msg, 8 srv)

### Changed
- stonefish_ros에서 메시지 정의 분리하여 독립 패키지로 구성
- DVL, DVLBeam 등 중복 메시지 통합 관리

### Technical Details
- stonefish_ros의 msg/srv 폴더를 stonefish_msgs로 이동
- cola2_msgs를 stonefish_msgs 폴더 내로 재조직
- 모든 의존성 업데이트 완료
