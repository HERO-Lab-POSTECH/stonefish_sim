# stonefish_sim — 코딩 컨벤션 & 작업 프로세스

> **작업 전 필독.** 이 문서는 `stonefish_sim`(다중 ROS2 패키지: 시뮬레이터 래퍼 + 제어/궤적)에서
> 코드를 추가·수정·리팩토링할 때 따라야 할 두 가지를 규정한다: **(1) 코딩 컨벤션**(이 repo의
> *실제 코드*에서 귀납한 규칙, 추측 아님) + **(2) 작업 프로세스 게이트**(구현 전 분석·조사 의무).
>
> 이 규칙은 P3(2026-06)부터 적용한다. 컨벤션은 이 repo에 *이미 존재하는 다수 패턴*에서 근거를
> 인용해 도출했다 — 새 규칙을 발명한 것이 아니라 집을 짓던 방식을 명문화한 것이다.

---

## 1. 작업 프로세스 게이트 (먼저 읽을 것)

P1·P2에서 테스트·CI 안전망을 깔았지만 본 코드는 거의 손대지 않았다. P3부터는 **실제 동작을
바꾸는 변경**이 시작되므로, 모든 비자명한 작업은 아래 5단계 게이트를 **순서대로** 통과한다.
"간단해 보여서 바로 고침"이 이 프로젝트에서 가장 위험한 안티패턴이다.

| 단계 | 무엇을 | 산출물(증거를 남길 것) |
|:---|:---|:---|
| ① 정독 + 의존성 추적 | 대상 코드를 줄 단위로 읽고, 그것을 import/호출하는 곳·그것이 의존하는 곳을 모두 추적 | "누가 이걸 쓰나" 목록 (grep/LSP 근거) |
| ② 자료 조사 | ROS2 Humble 관행·gtsam/numpy/scipy API·수중로봇 도메인 관행을 공식 문서로 확인 (기억 금지) | 인용 URL/문서 근거 |
| ③ 설계 | 변경 범위·인터페이스·하위호환·테스트 전략을 먼저 글로 적음 | 설계 노트 (.sp/ 또는 작업 브리프) |
| ④ 구현 | 설계대로 최소 변경. 기존 스타일을 따름(§2) | diff |
| ⑤ 검토 | 별도 패스(자가승인 금지)로 정합성·테스트·회귀 확인 | 테스트 결과 + 리뷰 |

**게이트 운용 원칙**
- **추측 금지·증거 우선.** 파일 경로·함수 시그니처·동작은 *읽어서* 확인한다. "아마 이럴 것"으로 코드를 쓰지 않는다.
- **단계별 산출물을 남긴다.** 분석·조사 결과를 `.sp/`(scratch)나 작업 브리프에 기록해, 검토자가 *왜* 그렇게 고쳤는지 추적할 수 있게 한다.
- **authoring과 review는 분리한다.** 같은 컨텍스트에서 자기 작업을 승인하지 않는다. 검토는 별도 에이전트/패스로 한다.
- **규모에 맞게 조절.** 3–4줄 자명한 수정에 5단계 전부를 강요하지 않는다. 로직·수치·인터페이스·다중파일을 건드리면 전체 게이트, 오타·주석은 ④⑤만.
- **3-Strike / 15분 규칙.** 같은 접근이 3번 실패하거나 한 문제에 15분 막히면 방법을 바꾼다.

---

## 2. 코딩 컨벤션 (이 repo의 실제 패턴)

### 2.0 명명·구조 표준 — 외부 근거와 대조

이 절은 §2.1~2.3의 명명·구조 규칙이 *왜* 맞는지를 권위 있는 외부 표준으로 뒷받침한다.
표준은 **강제(normative — ROS RMW/RCL 코드나 REP가 실제로 거부)**와 **관행(practice — 커뮤니티 norm,
린터 수준)**으로 나뉜다. 이 repo의 현재 관행은 대부분 표준에 부합하지만, 어긋나는 지점은 명시한다.

| 항목 | 외부 표준 | 등급 | 이 repo 현황 | 출처 |
|:---|:---|:---|:---|:---|
| 패키지명 | 소문자 영숫자+`_`, 알파벳 시작, 2자↑, `__`·하이픈·대문자 금지 | **강제** | 7개 패키지 전부 준수 | [REP 144](https://ros.org/reps/rep-0144.html) |
| 패키지 접미사 | `_msgs`(msg/srv), `_description`(URDF+meshes) 등 | 규약 | `stonefish_msgs`·`stonefish_description` 정확 | REP 144 |
| 패키지 catchall | `utils` 같은 포괄명 회피 | 권고(SHOULD) | 패키지명엔 없음(서브모듈명엔 무관) | REP 144 |
| ament_python 레이아웃 | `<pkg>/<pkg>/`+setup.py+setup.cfg+`resource/<pkg>` 마커+test/ | 사실상 강제(툴체인) | 3개 ament_python 패키지 준수 | [ros2cli template](https://github.com/ros2/ros2cli/tree/rolling/ros2pkg/ros2pkg/resource/ament_python) |
| 내부 서브디렉토리 | `nodes/`·`controllers/` 등 세부 레이아웃 | **신뢰할 출처 없음**(공식 규정 없음) | 자유롭게 분리 — 표준 위반 아님 | [design.ros2.org/ament](https://design.ros2.org/articles/ament.html) |
| 노드명 문자 | 영숫자+`_`만, 숫자시작·`/` 금지 | **강제(RMW)** | 준수 | [rmw validate_node_name.c](https://github.com/ros2/rmw/blob/master/rmw/src/validate_node_name.c) |
| 노드명 스타일 | lower_snake_case | 관행 | snake_case 준수 | (community) |
| 토픽/서비스명 문자 | `[a-zA-Z0-9_/~{}]`, `//`·`__` 금지, `~/foo`(≠`~foo`) | **강제(RCL)** | 준수 | [design.ros2.org/topic names](https://design.ros2.org/articles/topic_and_service_names.html) |
| 토픽 스타일 | snake_case + `/` 계층 | 관행(wire는 대문자 허용) | snake_case·계층 사용 | (community) |
| .msg/.srv 필드명 | lower_snake_case 강제(대문자 금지) | **강제(rosidl)** | (해당 시 준수) | [rosidl#25](https://github.com/ros2/rosidl/pull/25) |
| 파라미터 dot notation | `outer_loop.Kp` 중첩 표기 | 관행(정식 char spec 없음) | dot notation 사용 | [design.ros2.org/parameters](https://design.ros2.org/articles/ros_parameters.html) |
| 함수·변수 | lower_snake_case / 클래스 `CapWords` / 상수 `UPPER_WITH_UNDER` / 비공개 `_` | **강제(PEP 8)** | 준수 | [PEP 8](https://peps.python.org/pep-0008/) |
| import | 그룹(stdlib→3rd→local), 줄당 1개, **wildcard 회피** | PEP 8 권고 | sim은 상대 import·wildcard 미사용 | PEP 8 / [Google](https://google.github.io/styleguide/pyguide.html) |
| docstring | 모든 public 모듈/함수/클래스 + Google Args/Returns/Raises | PEP 257 권고 | 새 코드 준수 | [PEP 257](https://peps.python.org/pep-0257/) |

**좌표계 — REP 103 (★중요, 이 repo의 핵심 결정)**
- ROS 표준(REP 103)은 world 프레임에 **ENU**(x east/y north/z up), body 프레임에 **FLU**(x forward/y left/z up)를 1차로 규정한다.
- 이 repo는 수중로봇 도메인 관행(해양·항공 = NED)을 따라 `world_ned`를 쓴다. **이는 표준 위반이 아니다** — REP 103이 NED를 위해 정확히 `_ned` 접미사 secondary frame을 규정한다: *"define an appropriately transformed secondary frame with the '_ned' suffix"*. 즉 `world_ned`는 **표준이 허용한, 이름으로 명시된 편차**다.
- 쿼터니언: `geometry_msgs/Quaternion` 필드 순서는 `x y z w`(scalar-last) — 내부 `[w,x,y,z]`와 변환 지점에서 반드시 명시(§2.7).
- 수중로봇 정식 REP는 아직 없다(marine REP 2024 제안만, 미비준). 출처: [REP 103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst), [REP 105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst).

**이 repo에서 발견된 명명 위반(P4_FLAGS 후보, 고칠 때까지 새 코드는 답습 금지)**
- ⚠️ **노드명 중복**: `controllers/velocity_controller_node.py`와 `nodes/position_controller_node.py`가 둘 다 `super().__init__('pid_4dof_controller')`로 초기화된다. 동시 실행 시 ROS2 고유 노드명 위반. 새 노드는 파일·역할과 일치하는 고유 이름을 쓴다.

---

### 2.1 파일·디렉토리 구조
- 모든 Python 파일은 **snake_case**. 노드 실행 진입점은 **`*_node.py`** 접미사를 붙인다.
  근거: `nodes/path_generator_node.py`, `nodes/path_following_node.py`, `controllers/hybrid_controller_node.py`.
- 각 ROS2 패키지는 **ament_python 레이아웃**: `setup.py` + `setup.cfg`가 `entry_points`(console_scripts)를
  정의하고, 노드는 `nodes/`, 컨트롤러는 `controllers/`, 알고리즘은 전용 서브모듈(`path_generator/`,
  `common/`, `control_interfaces/`)에 둔다. 근거: `stonefish_trajectory_manager/setup.py:23-29`.
- 테스트는 **패키지별 `test/` 디렉토리**, config는 패키지 루트 `config/`, launch는 `launch/`.
- `__init__.py`는 공개 API를 import해 `__all__`로 재노출하거나(`stonefish_trajectory_manager/__init__.py:28-44`),
  최소한 `__version__`만 둔다(`stonefish_control/__init__.py:15`).

### 2.2 import
- 순서: **(1) `__future__` → (2) stdlib → (3) 서드파티(numpy, scipy, transforms3d, rclpy, ROS msgs) → (4) 로컬**.
- **같은 패키지 내부는 상대 import** (`from ..common import X`), 다른 패키지는 절대 import. 근거: `nodes/path_generator_node.py:35-46`.
- 선택적/무거운 의존성은 `try/except`로 감싼다(예: `casadi`, deprecated `scipy.misc`). 근거: `control_interfaces/vehicle.py:18-30`.
- ⚠️ slam과 다름: slam은 절대 import + 일부 wildcard를 쓰지만, **sim은 상대 import가 표준**이다. 새 코드는 wildcard import를 쓰지 않는다.

### 2.3 명명
- 변수·함수·ROS 파라미터: **snake_case**. 내부/비공개는 `_` 접두(`_path_generated`, `_vehicle_pos`).
- 클래스: **PascalCase**(`PathGeneratorNode`, `HybridController`). 상수: **UPPERCASE**(`FINAL_WAYPOINT_COLOR`).
- **단위는 변수명에 넣지 않는다**(`_m`, `_rad` 금지). 단위는 docstring·YAML 주석으로 문서화한다. 근거: `path_generator_node.py:61`(미터 단위지만 이름엔 없음).

### 2.4 docstring·타입힌트
- **Google-style docstring**(`Args:`, `Returns:`, `Raises:`). 모듈 docstring은 전 파일 필수(저작권 헤더 뒤), 클래스·공개 메서드 필수, 비공개 헬퍼는 선택. 언어는 **영어**. 학술 출처는 모듈 docstring에 `Reference:`로 인용.
- **타입힌트는 공개 API·복잡한 함수에 선택적으로**(런타임 강제 아님). 새 공개 인터페이스에는 붙이는 것을 권장.
- ⚠️ 상속받은 레거시(UUV Simulator 유래: `bezier_curve.py`, `waypoint.py`)는 markdown-style docstring·무타입힌트다. **이는 "grandfathered"로 두되 새 코드의 본보기가 아니다** — 새 코드는 위 표준을 따른다.

### 2.5 에러 처리·로깅
- 명시적 예외(`ValueError`, `RuntimeError`, `TypeError`)를 서술적 메시지와 함께 raise. `__init__`에서 유효성 검증(예: `raise ValueError('Mass has to be positive')`). **bare except 금지**.
- ROS2 노드는 **`self.get_logger()`** 만 사용: `.info()`(정상 흐름), `.warn()`(저하 동작), `.error()`(예외). 고빈도 메시지는 `throttle_duration_sec`. 근거: `path_following_node.py:355`.
- `print()`·`traceback.print_exc()`는 **오직 `main()` 진입점의 예외 핸들러에서만**. 프로덕션 경로에 `print()` 금지.
- `main(args=None)` 진입점은 `rclpy.init`/`spin`/`shutdown` + `try/except/finally`로 정리.

### 2.6 ROS2 노드 패턴
- 모든 노드는 `rclpy.node.Node` 상속. `super().__init__('node_name')` 호출.
- 파라미터는 `__init__` 초기에 `declare_parameter(name, default)` → `get_parameter().value`로 **한 번만** 읽어 `self._attr`에 저장(핫패스에서 재조회 금지).
- pub/sub은 `create_publisher`/`create_subscription`(QoS=10 기본). 콜백은 **`_<event>_callback`**(`_odometry_callback`). 주기 작업은 `create_timer(1.0/rate, cb)`.

### 2.7 config·상수·단위·좌표계
- 파라미터는 노드에서 `declare_parameter`로 선언, 기본값은 차량별 YAML(`config/bluerov2/*.yaml`)에서. 코드에 매직넘버 금지.
- **NED 좌표계**: `frame_id='world_ned'`로 통일. SI 단위(m, rad, rad/s, N, Nm, kg, kg·m²), YAML 주석·docstring으로 단위 명시.
- 쿼터니언 규약: **내부 `[w, x, y, z]` vs ROS `[x, y, z, w]`** — 변환 지점에 주석으로 명시(`path_generator_node.py:286-287`). 회전은 `transforms3d`(euler2quat/quat2euler).

### 2.8 테스트 (P2에서 정립)
- 테스트는 패키지별 `test/`에 `test_*.py`, 함수는 `test_*`.
- **import-time 오염(ROS/gtsam)을 피하려고 루트 `conftest.py`의 `load_module` fixture**(`importlib.util.spec_from_file_location`)로 모듈 파일을 직접 로드한다. 패키지 경로 import 금지. 근거: `conftest.py:8-23`.
- `pytest.ini`의 `testpaths = stonefish_control`로 discovery를 제한(시뮬레이터 래퍼 `stonefish_ros2` 제외).
- 기대값은 **수학적 정답**으로 작성한다. 코드 출력과 불일치하면 코드 버그로 보고 `P4_FLAGS.md`에 기록한다(테스트를 코드에 맞추지 않는다).
- 테스트 추가 시 **live 코드 0줄 변경** 원칙(P2). 코드 변경이 필요한 테스트(지연 import·콜백 추출 등)는 P3 리팩토링으로 따로 다룬다.
- CI: `.github/workflows/ci.yml`(Python 3.10 고정, `pip install pytest numpy scipy scikit-learn transforms3d` → `pytest -v`).

---

## 3. 적용 메모
- 이 repo의 SSOT는 이 문서다. `CLAUDE.md`가 생기면 "작업 전 `docs/CONVENTIONS.md` 필독" 포인터 1줄을 둔다(CLAUDE.md만 자동 로드되므로).
- 라이선스는 **GPL-3.0**(루트 `LICENSE` 기준). 메인테이너: Seungmin Kim <luckkim123@gmail.com>.
- `P4_FLAGS.md`에 모인 수치/알고리즘 이슈는 P4에서 소화한다(현재: bezier list-concat 버그).
