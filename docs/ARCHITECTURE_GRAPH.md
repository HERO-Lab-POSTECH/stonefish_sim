# stonefish_sim — 아키텍처 그래프 맵

> **2026-08-31 graphify로 전면 재측정.** 이전 판의 수치는 은퇴한
> code-review-graph(CRG) 스냅샷이었고 §1~§4를 전부 교체했다. 측정 대상은
> `.graphify/graph.json` — 빌드 커밋 `b74e5d0`, 현 브랜치 HEAD 대비 1커밋 뒤이고 그
> 1커밋은 문서·ignore 파일만 건드리므로 코드 수치에는 영향이 없다.
>
> **CRG 수치와 직접 비교하지 말 것 — 세는 대상이 다르다.** CRG는 tree-sitter로 코드만
> 봤고, graphify는 의미 추출 패스를 돌려 산문도 노드로 갖는다. 이 repo는 2,353노드 중
> 코드가 1,048개뿐이고 나머지 1,305개가 문서·근거·개념 노드다. 노드 수가 703→2,353으로
> "늘어난" 게 아니라 코퍼스가 달라진 것이다.
>
> **§4는 지표 자체가 바뀌었다.** CRG의 flow criticality는 graphify에 대응물이 없어
> `graphify query`의 BFS 반경으로 대체했다. 묻는 것("어느 진입점이 얼마나 넓게 닿나")은
> 같지만 숫자의 정의가 다르므로 이전 판의 criticality 값과는 비교 불가다.
>
> 크로스 repo 관찰(sim↔slam 비교, 토픽 경계)은 워크스페이스
> `.omp/wiki/architecture-2026-08-21-graph-map.md`에 있다. 이 문서는 sim 내부만 다룬다.

---

## 1. 규모

| 항목 | 값 |
|:--|--:|
| 노드 | 2,353 |
| 링크 | 3,116 |
| 하이퍼엣지 | 14 |
| 인덱싱된 파일 | 146 |
| 커뮤니티 | 263 |

노드 종류: code 1,048 · document 570 · rationale 511 · concept 209 · paper 15.
출처: AST(무료·오프라인) 1,899 · semantic(유료 LLM 패스) 454.
확장자별: `.py` 1,144 · `.md` 765 · `.h` 140 · `.cpp` 97 · `.pdf` 30 · `.yaml` 29 ·
`.rst` 21 · `.txt` 12.

링크 종류: contains 767 · references 654 · rationale_for 481 · calls 445 · method 294 ·
imports 155 · defines 98 · imports_from 46 · conceptually_related_to 44 · inherits 25 ·
shares_data_with 25 · re_exports 19 · implements 17 · semantically_similar_to 17 ·
uses 14 · cites 14 · indirect_call 1.

**calls 445 < contains 767이라는 점을 먼저 읽어야 한다.** 이 그래프는 호출 지도가 아니라
포함·참조 지도에 가깝다. 호출 관계만 보고 싶으면 `graphify query --context call`로 좁혀야
하고, 그러지 않으면 문서 참조가 결과를 채운다.

inherits가 25뿐인 것은 CRG 때(18)와 같은 결론을 준다 — 이 repo는 상속보다 조합·모듈
함수로 짜였다.

## 2. 노드 분포

**graphify의 커뮤니티는 디렉토리가 아니다.** 263개 Leiden 클러스터(노드당 평균 9개)라
심볼 단위에 가깝고, 패키지 지도로 쓸 수 없다. CRG의 "커뮤니티 = 디렉토리" 표가 하던 역할은
디렉토리별로 따로 집계해야 한다. 아래는 **코드 노드만** 센 것이다(산문 제외).

| 최상위 경로 | 코드 노드 |
|:--|--:|
| `stonefish_control` | 395 |
| `stonefish_ros2` | 257 |
| `test` | 199 |
| _(source_file 없음 — 외부 심볼)_ | 115 |
| `stonefish_albc_bridge` | 64 |
| `stonefish_sonar_yolo` | 9 |
| `stonefish_msgs` | 4 |
| `nav_interfaces` | 2 |
| `stonefish_description` | 1 |

`stonefish_control` 395는 4개 하위 패키지로 갈린다:
`stonefish_trajectory_manager` 279 · `stonefish_control` 71 ·
`stonefish_thruster_manager` 42 · `stonefish_control_msgs` 3.

**무게중심은 CRG 때와 같다** — 시뮬레이터 브리지(257)보다 경로생성·경로추종(279)이 크다.

`source_file`이 없는 115개는 `Node`·`shared_ptr`·`Publisher`·`ImageTransport` 같은
**외부 심볼**이다. 이 repo에 정의가 없어 파일이 안 붙는다. 결함이 아니라 rclcpp/std 의존을
그래프가 노드로 잡은 결과다.

### 2.1 trajectory_manager 드릴다운 (패키지 코드 노드 263)

| 노드 | 파일 |
|--:|:--|
| 34 | `common/trajectory_generator.py` |
| 32 | `path_generator/path_generator.py` |
| 29 | `common/waypoint.py` |
| 27 | `path_following/ilos_guidance.py` |
| 26 | `common/waypoint_set.py` |
| 25 | `common/trajectory_point.py` |
| 18 | `path_generator/lipb_interpolator.py` |
| 12 | `path_generator/cs_interpolator.py` |
| 12 | `path_generator/bezier_curve.py` |
| 9 | `path_generator/linear_interpolator.py` |
| 9 | `nodes/path_following_node.py` |
| 8 | `nodes/path_generator_node.py` |
| 7 | `path_generator/line_segment.py` |
| 6 | `path_following/alos_guidance.py` |
| 4 | `nodes/utils.py` |

`common/`(웨이포인트·궤적 자료형) 114노드가 `path_generator/` 90노드보다 여전히 크다 —
자료형 계층이 알고리즘 계층보다 무겁다. `nodes/` 21노드는 얇은 ROS 래퍼라는 설계와 맞다.

### 2.2 stonefish_control 드릴다운 (코드 노드 63)

| 노드 | 파일 |
|--:|:--|
| 17 | `control_interfaces/dynamics_loader.py` |
| 11 | `nodes/hybrid_controller_node.py` |
| 10 | `nodes/position_controller_node.py` |
| 7 | `controllers/position_controller.py` |
| 7 | `controllers/hybrid_controller.py` |
| 5 | `controllers/cascade_controller.py` |

`dynamics_loader.py`(차량 물성 — mass·inertia·cog·cob·buoyancy)가 최대이고, 컨트롤러
3개는 각각 5~7노드로 작다. **여기서 노드 수를 코드량으로 읽으면 안 된다**:
P5에서 도입한 `cascade_controller.py`는 5노드지만 경로추종의 cross-track 채널을 단독으로
담당한다(auto-memory `stonefish-p5-cascade-complete`).

### 2.3 stonefish_ros2 드릴다운 (코드 노드 257)

| 노드 | 파일 |
|--:|:--|
| 68 | `include/stonefish_ros2/ROS2SimulationManager.h` |
| 47 | `include/stonefish_ros2/ROS2Interface.h` |
| 36 | `src/stonefish_ros2/ROS2SimulationManager.cpp` |
| 26 | `src/stonefish_ros2/ROS2Interface.cpp` |
| 15 | `include/stonefish_ros2/ROS2ScenarioParser.h` |
| 13 | `src/stonefish_ros2/ROS2ScenarioParser.cpp` |

**헤더가 소스보다 노드가 많다**(68 > 36, 47 > 26). C++ 쪽 노드는 대부분 선언에서
나오므로, 여기서 "노드 = 로직의 양"으로 읽으면 틀린다. `.cpp` 97노드 대 `.h` 140노드는
구현이 아니라 인터페이스 표면의 크기다.

## 3. 허브 노드 (`graphify god-nodes`)

`god-nodes`는 `_callable` 노드만 센다 — 파일 노드(`ROS2SimulationManager.cpp` 58 등)는
집계에서 빠진다. in/out은 링크에 기록된 방향이며, 그래프 자체는 `directed=false`다.

| 노드 | 파일 | degree | in / out |
|:--|:--|--:|:--|
| `Waypoint` | `.../common/waypoint.py` | 72 | 44 / 28 |
| `ROS2SimulationManager` | `.../ROS2SimulationManager.h` | 61 | 1 / 60 |
| `TrajectoryPoint` | `.../common/trajectory_point.py` | 42 | 18 / 24 |
| `PathGenerator` | `.../path_generator/path_generator.py` | 42 | 11 / 31 |
| `WaypointSet` | `.../common/waypoint_set.py` | 42 | 18 / 24 |
| `WPTrajectoryGenerator` | `.../common/trajectory_generator.py` | 41 | 8 / 33 |
| `load_module()` | `conftest.py` | 35 | 34 / 1 |
| `_make()` | `test/test_characterization_lipb.py` | 31 | 29 / 2 |
| `ILOSGuidance` | `.../path_following/ilos_guidance.py` | 30 | 7 / 23 |
| `ROS2Interface` | `.../ROS2Interface.h` | 28 | 1 / 27 |
| `LIPBInterpolator` | `.../path_generator/lipb_interpolator.py` | 24 | 5 / 19 |
| `_make_cascade()` | `test/test_cascade_controller.py` | 24 | 23 / 1 |
| `ROS2Robot` | `.../ROS2SimulationManager.h` | 23 | 9 / 14 |
| `DynamicsLoader` | `.../control_interfaces/dynamics_loader.py` | 22 | 7 / 15 |
| `ObsBuilder` | `.../stonefish_albc_bridge/obs_builder.py` | 21 | 19 / 2 |

**CRG 때의 결론 하나가 뒤집힌다.** 이전 판은 "허브는 대부분 out이 크고 in이 1~2 — 많이
호출하는 조립 지점이라 바꿔도 자기만 깨진다"고 읽었다. graphify에서는 최상위 허브
`Waypoint`가 **in 44**다. 즉 진짜로 많이 의존되는 자료형이고, 시그니처를 바꾸면 파급이
바깥으로 간다. `TrajectoryPoint`(in 18)·`WaypointSet`(in 18)도 같다. **이 repo에서 가장
위험한 변경 지점은 컨트롤러가 아니라 `common/`의 웨이포인트·궤적 자료형이다.**

CRG가 옳았던 부분도 있다 — `ROS2SimulationManager`(1/60)·`ROS2Interface`(1/27)는
여전히 순수한 조립 지점이다.

테스트 쪽 in-degree 세 개가 단일 실패 지점이다:
- `conftest.py`의 `load_module()` — **in 34**. 모든 테스트가 이 fixture로 모듈을 파일
  경로에서 로드한다(CLAUDE.md "테스트가 모듈을 파일 경로로 로드하는 이유"). 여기를 바꾸면
  테스트 스위트 전체가 동시에 깨진다.
- `_make()`(in 29) · `_make_cascade()`(in 23) — 각각 해당 characterization 파일 전체.

## 4. 진입점 BFS 반경 (`graphify query`, depth 2)

CRG의 flow criticality를 대체하는 측정이다. 각 진입점 식별자에서 깊이 2 BFS로 닿는
노드 수 — "이 진입점을 건드리면 몇 개가 시야에 들어오나"에 해당한다.

| 진입점 | 닿는 노드 | 전체 대비 |
|:--|--:|--:|
| `ROS2SimulationManager` | 93 | 4.0% |
| `ILOSGuidance` | 65 | 2.8% |
| `CascadeController` | 45 | 1.9% |
| `BridgeNode` | 39 | 1.7% |
| `PathGeneratorNode` | 24 | 1.0% |
| `PID4DOFNode` | 24 | 1.0% |
| `PathFollowing4DOFNode` | 21 | 0.9% |
| `ThrusterAllocatorNode` | 20 | 0.9% |
| `HybridController4DOFNode` | 18 | 0.8% |

**CRG의 "얕고 넓다"는 성격 판정은 도구를 바꿔도 유지된다.** ROS 진입점 5개가 모두
18~24노드에 그치고, 최대인 `ROS2SimulationManager`도 전체의 4%다. slam은 진입점 하나가
17%를 덮는다(그쪽 문서 §5) — 정반대다.

따라서 회귀 전략도 그대로다: **sim은 진입점마다 개별 검증이 필요하고, 한 flow를 덮었다고
다른 flow가 덮이지 않는다.** 알고리즘 노드(`ILOSGuidance` 65 · `CascadeController` 45)가
ROS 래퍼보다 반경이 크다는 것도 같은 얘기다 — 래퍼는 얇고 로직은 아래 있다.

## 5. 이 그래프가 못 보는 것

- **ROS 토픽 결합**: `thruster_manager/input`·`cmd_pose`·`cmd_vel`은 엣지가 아니다.
  `ros2 topic info -v`로 검증할 것.
- **`stonefish_msgs` → slam 의존**: 크로스 repo라 blast radius가 0으로 나온다.
  "영향 없음"이 아니라 "측정 불가"다.
- **`.scn` 씬 파일**: 노드가 **0개**다. 시뮬 physics 파라미터 변경은 그래프가 항상 빈
  답을 준다 — `grep`으로 형제 파일과 비교할 것.
- **YAML의 *값***: `.yaml`은 29노드로 그래프에 있지만 그중 22개가 semantic 패스가 만든
  개념·근거 노드다(AST는 7개). 즉 그래프가 아는 것은 "이 config가 무엇에 관한 것인가"이지
  게인 숫자가 아니다. **값을 바꾼 변경은 그래프에 나타나지 않는다.** 이전 판의
  "YAML은 노드를 만들지 않는다"는 서술은 graphify에서 반증됐지만, 실용적 결론(값 비교는
  `grep`)은 같다.

## 6. 조회

```bash
cd /workspace/src/stonefish_sim
graphify query "ILOSGuidance"    # 심볼·개념에서 BFS
graphify god-nodes               # 가장 많이 연결된 허브
graphify update .                # 편집 후 갱신 (AST 패스는 무료·오프라인)
```

**루트(`/workspace`)에서 물으면 안 된다** — meta-repo가 `src/`를 gitignore하므로 루트엔
그래프가 없다. 여러 단어로 물으면 매칭이 안 되니 식별자 하나로 좁힐 것. 신뢰 규칙 정본은
워크스페이스 `.claude/rules/code-graph.md`.
