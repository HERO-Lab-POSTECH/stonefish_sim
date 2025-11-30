# CHANGELOG

## [Unreleased]

### Added
- `set_ocean_current` ROS2 서비스 구현 (ROS2SimulationManager)
  - 서비스명: `/stonefish_ros2/stonefish_simulator/set_ocean_current`
  - 서비스 타입: `stonefish_msgs/srv/SetOceanCurrent`
  - 기능:
    - 실행 중 해수 흐름(Uniform 타입) 속도 제어
    - 흐름 활성화/비활성화 토글
    - NED 좌표계 속도 입력 [vx, vy, vz]
  - 오류 처리:
    - 해수 흐름 없음 (null ocean)
    - 잘못된 인덱스 (invalid index)
    - 지원하지 않는 타입 (non-Uniform type)
