# CHANGELOG

## [Unreleased]

### Fixed
- XML 파싱 오류 수정: `data/worlds/world_empty.scn` 파일의 24번 라인
  - `<conditions>` 태그를 자체 종료 태그(`/>`)로 변경
  - 변경 전: `<conditions temperature="20.0" pressure="101300.0" humidity="0.5">`
  - 변경 후: `<conditions temperature="20.0" pressure="101300.0" humidity="0.5"/>`
  - 영향: 시뮬레이션 시나리오 로딩 성공

