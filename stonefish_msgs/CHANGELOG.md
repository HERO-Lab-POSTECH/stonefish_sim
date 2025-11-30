# CHANGELOG

## [Unreleased]

### Added
- `SetOceanCurrent.srv` 서비스 정의
  - 목적: 실행 중 해수 흐름(Ocean Current) 제어
  - 필드:
    - `request`: 흐름 속도 [vx, vy, vz] (m/s, NED 좌표계), 활성화 여부
    - `response`: 성공 여부 및 오류 메시지
  - 용도: 시뮬레이션 중 환경 조건 동적 변경

### Changed
- `SetOceanCurrent.srv` 서비스 확장
  - 새 필드 추가: `current_type` (Uniform/Jet 선택)
  - 새 필드 추가: `outlet_velocity` (Jet 타입용 배출 속도 스칼라값)
  - Uniform 타입: 기존 3D 속도 벡터 제어 유지
  - Jet 타입: 배출 속도 스칼라값으로 제어 (역호환 유지)
