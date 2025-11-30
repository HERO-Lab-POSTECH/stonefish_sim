# CHANGELOG

## [Unreleased]

### Changed
- `SetWaveHeight.srv` 파도 높이 범위 확대: 0.0-10.0 meters (기존: 0.0-2.0m)

### Added
- `SetWindVelocity.srv` 서비스 정의
  - 목적: 실행 중 대기 바람 속도(Wind Velocity) 동적 제어
  - 필드:
    - `request`: north, east, down (float64, m/s, NED 좌표계)
    - `response`: success (bool), message (string)
  - 용도: 시뮬레이션 중 풍속 조건 실시간 변경
- `SetWaveHeight.srv` 서비스 정의
  - 목적: 실행 중 해양 파도 높이(Ocean Wave Height) 동적 제어
  - 필드:
    - `request`: wave_height (0.0 - 10.0 meters)
    - `response`: 성공 여부 및 상태 메시지
  - 용도: 시뮬레이션 중 파도 조건 실시간 변경 (0.0 = flat ocean)

- `SetOceanCurrent.srv` 서비스 정의
  - 목적: 실행 중 해수 흐름(Ocean Current) 제어
  - 필드:
    - `request`: 흐름 속도 [vx, vy, vz] (m/s, NED 좌표계), 활성화 여부
    - `response`: 성공 여부 및 오류 메시지
  - 용도: 시뮬레이션 중 환경 조건 동적 변경
