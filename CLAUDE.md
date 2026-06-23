# stonefish_sim

ROS2 Humble 수중로봇 시뮬레이션 워크스페이스 — Stonefish 시뮬레이터 C++ 래퍼(`stonefish_ros2`)와
Python 제어/궤적 패키지(`stonefish_control`, `stonefish_description`, `stonefish_msgs`)로 구성된 다중 패키지 repo.

## 작업 전 필독

**모든 코드 작업 전에 [`docs/CONVENTIONS.md`](docs/CONVENTIONS.md)를 먼저 읽을 것.** 그 문서가 이 repo의 단일 진실(SSOT)이다:
- **코딩 컨벤션** — 명명·디렉토리 구조·import·docstring·ROS2 노드 패턴·config·테스트. 외부 표준(REP 144/103/105, PEP 8/257, Google Style)과 대조하고 강제(normative)/관행(practice)을 구분한다.
- **작업 프로세스 게이트** — 비자명한 변경은 ①정독+의존성추적 → ②자료조사 → ③설계 → ④구현 → ⑤검토를 순서대로 거치고 산출물을 남긴다.

## 핵심 사실
- 라이선스 **GPL-3.0**, 메인테이너 Seungmin Kim <luckkim123@gmail.com>.
- 좌표계 **NED**(`frame_id='world_ned'`) — REP 103의 `_ned` 접미사 규약을 따른 정당한 편차(`docs/CONVENTIONS.md` §2.0).
- 테스트: 루트 `conftest.py`의 `load_module` fixture로 모듈 직접 로드(패키지 경로 import 금지), `pytest -v`. CI는 `.github/workflows/ci.yml`(Python 3.10).
- 미해결 수치/명명 이슈는 [`P4_FLAGS.md`](P4_FLAGS.md)에 모인다 — 새 코드는 거기 적힌 안티패턴을 답습하지 않는다.
