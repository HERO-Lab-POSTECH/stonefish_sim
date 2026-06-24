# 버전·상태

이 페이지는 `stonefish_sim`의 현재 버전(0.4.0)과 그 릴리스에서 무엇이 바뀌었는지, 아직 해결되지 않은 알려진 이슈, 그리고 코드 전반에 적용되는 핵심 규약(CONVENTIONS)을 정리한다.

## 현재 버전

현재 버전은 **0.4.0**(2026-06-24)이며, 7개 ROS2 패키지 전체가 동일 버전으로 통일되어 있다. 라이선스는 전 패키지 GPL-3.0이다.

| 패키지 | 버전 | 빌드타입 |
|--------|------|---------|
| `stonefish_msgs` | `0.4.0` | ament_cmake |
| `stonefish_control_msgs` | `0.4.0` | ament_cmake |
| `stonefish_description` | `0.4.0` | ament_cmake |
| `stonefish_ros2` | `0.4.0` | ament_cmake |
| `stonefish_control` | `0.4.0` | ament_python |
| `stonefish_thruster_manager` | `0.4.0` | ament_python |
| `stonefish_trajectory_manager` | `0.4.0` | ament_python |

근거: `*/package.xml` 전수 검토.

## CHANGELOG v0.4.0 요약 (P4 — 알고리즘·수치 정합성)

0.4.0은 알고리즘과 수치 정합성(algorithmic/numeric correctness)을 다룬 P4 작업의 결과다. 변경 사항은 Fixed 5건, Removed 3건, Changed 2건으로 정리된다.

### Fixed (5건)

| 항목 | 내용 |
|------|------|
| T1.1 | `position_controller` 생성자 크래시 수정 |
| T1.2 | 하이브리드 제어기 YAML 미로드(wildcard) 문제 수정 |
| T1.3 | 위치 모드 feedforward의 `M·accel` 차원 오류 수정 |
| T1.4 | 최근 경유점 선정 `argmin` 버그(`np.abs`) 수정 |
| T1.5 | C++ 파서 null 역참조(null-deref) 수정 |

### Removed (3건)

| 항목 | 내용 |
|------|------|
| 죽은 제어층 | UUV-Simulator 기반 사용되지 않던 제어 코드 약 4000줄 제거 |
| 미사용 srv | 사용되지 않던 서비스 정의 5개 제거 |
| teleop stub | teleop 스텁 제거 |

### Changed (2건)

| 항목 | 내용 |
|------|------|
| 버전 통일 | 전 패키지를 `0.4.0`으로 정렬 |
| 재라이선스 | Python 파일 33개를 GPL-3.0으로 재라이선스 |

### 검증

릴리스 검증 결과 **pytest 42 passed**(이전 대비 +6)이다. C++ 측은 정적 검증만 수행했다.

!!! note "C++ 검증 범위"
    C++ 코드는 정적 검증만 거쳤다. 동적 테스트(런타임 실행 검증)는 0.4.0 검증 범위에 포함되지 않았다.

## P4_FLAGS — 미해결 이슈 (9건)

P4 작업 중 식별되었으나 0.4.0 시점에 해결되지 않고 기록만 남겨둔 이슈 9건이다. 영향 범위와 상태를 솔직하게 정리한다.

| # | 이슈 | 영향 | 상태 |
|---|------|------|------|
| 1 | 하이브리드 제어기 YAML과 코드의 `max_force`·`max_torque` 기본값 불일치 | 위치 모드 일부 기본값이 YAML(`800`/`160`)과 코드에서 어긋남 | 문서화만 됨 |
| 2 | C++ QoS에 `SensorDataQoS` 미적용 | 센서 토픽 QoS 프로파일이 권장값과 다를 수 있음 | 미해결 |
| 3 | 로봇 이름 중복 시 silent overwrite | 동일 이름 로봇 등록 시 경고 없이 덮어씀 | 미해결 |
| 4 | 단일 스레드 spin 블로킹 | 콜백이 단일 스레드에서 블로킹될 수 있음 | 미해결 |
| 5 | wall-clock 타임스탬프 + `/clock` 부재 | EKF/TF 시간 동기에 위험(시뮬 시간과 불일치 가능) | 미해결 |
| 6 | `VelocityProfiler` dead 분기 | 도달 불가능한 코드 경로 존재 | 미해결 |
| 7 | 가속도 feedforward end-to-end 미연결 | position 모드 feedforward가 사실상 0으로 동작 | 미해결 |
| 8 | `Waypoint`에 `__hash__` 없음 | latent(현재는 표면화되지 않은 잠재 결함) | 미해결 |
| 9 | 곡률 추정 방식 4종 다양성 | 곡률 추정 구현이 여러 갈래로 존재 | 미해결 |

!!! warning "시간 동기(P4_FLAGS #5)"
    시뮬레이터가 wall-clock 타임스탬프를 사용하고 `/clock`을 게시하지 않는다. EKF나 TF처럼 타임스탬프 정합에 의존하는 구성요소를 함께 운용할 때 시간 동기 문제가 발생할 수 있으므로, `use_sim_time` 사용 시 이 제약을 염두에 둔다.

## CONVENTIONS — 핵심 규약

코드 전반에 일관되게 적용되는 규약이다. 새 코드를 추가하거나 기존 코드를 읽을 때 기준이 된다.

### 좌표계 — NED

좌표계는 NED(North-East-Down)를 따른다. REP-103 관례에 맞춰 프레임 접미사 `_ned`를 사용한다. 시뮬레이터의 진실값 odometry(`/{vehicle}/odometry`)도 NED 기준으로 게시된다.

### 쿼터니언 변환

쿼터니언은 내부 표현과 ROS 메시지 표현의 순서가 다르므로 변환 시 주의한다.

| 영역 | 순서 |
|------|------|
| 내부 표현 | `[w, x, y, z]` |
| ROS 메시지 | `[x, y, z, w]` |

두 표현 사이를 오갈 때 성분 순서를 반드시 맞춰야 한다.

### import 규약

같은 패키지 안(intra-package)에서는 상대 import를, 패키지 간(inter-package)에서는 절대 import를 사용한다. docstring은 Google 스타일을 따른다.

### conftest fixture

테스트는 루트 `conftest.py`의 `load_module` fixture를 통해 모듈을 적재한다. 이 fixture는 ROS/gtsam 오염을 회피하기 위한 것으로, 패키지 직접 import를 금지한다.

```bash
pytest -v
```

근거: `conftest.py:1-22`.

!!! tip "테스트 작성 시"
    패키지를 직접 import하지 말고 `load_module` fixture를 거친다. ROS·gtsam 등 외부 의존성이 테스트 환경을 오염시키는 것을 막기 위한 규약이다.

## 알려진 제약 (솔직하게)

0.4.0은 알고리즘·수치 정합성에 집중한 릴리스이며, 다음 제약을 명시적으로 안고 있다.

- **C++ 동적 검증 부재**: C++ 코드는 정적 검증만 거쳤고, 런타임 실행 기반 검증은 수행되지 않았다.
- **시간 동기 미해결(P4_FLAGS #5)**: wall-clock 타임스탬프와 `/clock` 부재로 EKF/TF 시간 동기 위험이 남아 있다.
- **YAML/코드 기본값 불일치(P4_FLAGS #1)**: 하이브리드 제어기의 일부 한계값이 YAML과 코드에서 어긋나 있으며 현재는 문서화만 된 상태다.
- **position 모드 feedforward 비활성(P4_FLAGS #7)**: 가속도 feedforward가 end-to-end로 연결되어 있지 않아 position 모드에서 사실상 0으로 동작한다.
- **단일 스레드 spin(P4_FLAGS #4)**: 콜백이 단일 스레드에서 블로킹될 수 있다.

이 제약들은 숨기지 않고 P4_FLAGS로 추적되고 있다. 위에 추린 것은 영향이 큰 일부이며, 나머지는 위 표의 9건에 정리되어 있다.
