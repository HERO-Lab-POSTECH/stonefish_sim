# CHANGELOG

## [Unreleased]

### Changed
- 환경 설정에서 ocean current 활성화: `data/worlds/common/environment.scn`
  - Uniform current (타입: uniform) 주석 해제
  - 초기 속도: [0.0, 0.0, 0.0]
  - SetOceanCurrent 서비스 기능 활성화 (current_index 0 사용 가능)

### Fixed
- XML 파싱 오류 수정: `data/worlds/world_empty.scn` 파일의 24번 라인
  - `<conditions>` 태그를 자체 종료 태그(`/>`)로 변경
  - 변경 전: `<conditions temperature="20.0" pressure="101300.0" humidity="0.5">`
  - 변경 후: `<conditions temperature="20.0" pressure="101300.0" humidity="0.5"/>`
  - 영향: 시뮬레이션 시나리오 로딩 성공

