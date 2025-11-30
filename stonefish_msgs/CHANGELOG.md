# CHANGELOG

## [Unreleased]

### Added
- `SetOceanCurrent.srv` 서비스 정의
  - 목적: 실행 중 해수 흐름(Ocean Current) 제어
  - 필드:
    - `request`: 흐름 속도 [vx, vy, vz] (m/s, NED 좌표계), 활성화 여부
    - `response`: 성공 여부 및 오류 메시지
  - 용도: 시뮬레이션 중 환경 조건 동적 변경
