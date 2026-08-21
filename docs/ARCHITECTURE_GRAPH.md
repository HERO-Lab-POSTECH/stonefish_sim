# stonefish_sim — 아키텍처 그래프 맵

> code-review-graph 측정 스냅샷. **커밋 `95fee95` (브랜치 `exp/albc-72d-bias-ema`)** 기준이며
> `head_matches_build: true` 상태에서 조회했다. main 기준이 아니므로 main과 비교할 때는
> `code-review-graph build`로 재측정할 것.
>
> 크로스 repo 관찰(sim↔slam 비교, 토픽 경계)은 워크스페이스
> `.omp/wiki/architecture-2026-08-21-graph-map.md`에 있다. 이 문서는 sim 내부만 다룬다.

---

## 1. 규모

| 항목 | 값 |
|:--|--:|
| 파일 | 86 |
| 노드 | 703 |
| 엣지 | 5,848 |
| 언어 | python, cpp, c |

노드 구성: Function 469 · Test 108 · File 86 · Class 40.
엣지 구성: CALLS 4,071 · CONTAINS 642 · TESTED_BY 610 · IMPORTS_FROM 469 · INHERITS 18 · REFERENCES 38.

INHERITS가 18뿐이라는 것은 이 repo가 상속보다 조합·모듈 함수로 짜였다는 뜻이다.
TESTED_BY 610은 Test 108개가 평균 5.6개 노드를 덮는다는 의미다.

## 2. 커뮤니티 (= 디렉토리)

커뮤니티는 Leiden 클러스터가 아니라 **디렉토리 기반**이다(`description` 필드로 확인).
따라서 아래 표는 사실상 "디렉토리별 노드 분포"다.

| 커뮤니티 | 디렉토리 | 노드 | cohesion | 주 언어 |
|:--|:--|--:|--:|:--|
| common-generate | `stonefish_control/stonefish_trajectory_manager` | 262 | 0.229 | python |
| test-import | `test` | 93 | 0.158 | python |
| stonefish-ros2-publish | `stonefish_ros2/src` | 86 | 0.007 | cpp |
| control-interfaces-control | `stonefish_control/stonefish_control` | 79 | 0.167 | python |
| policy-odom | `stonefish_albc_bridge/stonefish_albc_bridge` | 31 | 0.145 | python |
| test-inputs | `stonefish_albc_bridge/test` | 25 | 0.114 | python |
| stonefish-thruster-manager-wrench | `stonefish_control/stonefish_thruster_manager` | 22 | 0.173 | python |
| stonefish-ros2-namespace | `stonefish_ros2/include` | 11 | 0.000 | c |
| launch-generate | `stonefish_ros2/launch` | 5 | 0.000 | python |
| stonefish-sim-load | `conftest` | 2 | 0.000 | python |

**trajectory_manager 한 패키지가 703노드 중 262(37%)**를 차지한다. 이 repo의 무게중심은
시뮬레이터 브리지가 아니라 경로생성·경로추종 쪽이다.

### 2.1 trajectory_manager 드릴다운 (262노드)

구성: Function 240 · Class 15 · Test 7.

| 노드 | 파일 |
|--:|:--|
| 33 | `common/trajectory_generator.py` |
| 32 | `common/waypoint_set.py` |
| 31 | `path_generator/path_generator.py` |
| 28 | `common/waypoint.py` |
| 26 | `path_following/ilos_guidance.py` |
| 25 | `common/trajectory_point.py` |
| 17 | `path_generator/lipb_interpolator.py` |
| 11 | `path_generator/bezier_curve.py` |
| 11 | `path_generator/cs_interpolator.py` |
| 8 | `nodes/path_following_node.py` |
| 8 | `path_generator/linear_interpolator.py` |
| 7 | `nodes/path_generator_node.py` |
| 6 | `path_generator/line_segment.py` |
| 5 | `path_following/alos_guidance.py` |

`common/`(웨이포인트·궤적 자료형) 118노드가 `path_generator/` 84노드보다 크다 —
자료형 계층이 알고리즘 계층보다 무겁다. `nodes/` 15노드는 얇은 ROS 래퍼라는 설계와 맞다.

### 2.2 control_interfaces 드릴다운 (79노드)

`control_interfaces/data_types.py` 24노드가 최대. 게인·리밋·상태의 dataclass
(`OuterLoopGains`·`InnerLoopGains`·`ControlLimits`·`VehicleState`·`TrajectoryReference`)와
공통 수학 헬퍼(`angle_wrap`·`rotation_matrix_z`·`rotation_matrix_full`)가 여기 모인다.
`dynamics_loader.py`가 차량 물성(mass·inertia·cog·cob·buoyancy)을 담당하고,
컨트롤러는 `hybrid_controller.py`·`position_controller.py` 둘이다.

### 2.3 cohesion 0.007을 결함으로 읽지 말 것

`stonefish_ros2/src`는 86노드에 cohesion 0.007이다. 이는 C++ 브리지라서
tree-sitter가 호출 관계를 파이썬만큼 잡지 못한 **측정의 얕음**이지, 코드가 파편화됐다는
뜻이 아니다. 실제로 이 디렉토리의 허브 3개는 out_degree 181·81·63으로 매우 조밀하다.

## 3. 허브 노드 (blast radius 큰 지점)

| 노드 | 파일 | out | 총 degree |
|:--|:--|--:|--:|
| `SimulationStepCompleted` | `stonefish_ros2/src/.../ROS2SimulationManager.cpp` | 181 | 182 |
| `ParseSensor` | `stonefish_ros2/src/.../ROS2ScenarioParser.cpp` | 81 | 82 |
| `PathFollowing4DOFNode.__init__` | `.../nodes/path_following_node.py` | 67 | 69 |
| `ObsBuilder.update` | `stonefish_albc_bridge/.../obs_builder.py` | 45 | 67 |
| `ParseRobot` | `stonefish_ros2/src/.../ROS2ScenarioParser.cpp` | 63 | 64 |
| `BezierCurve.__init__` | `.../path_generator/bezier_curve.py` | 58 | 59 |
| `_make` (test helper) | `test/test_characterization_lipb.py` | 30 | 58 |
| `DynamicsLoader.__init__` | `.../control_interfaces/dynamics_loader.py` | 53 | 54 |
| `_inputs` (test helper) | `stonefish_albc_bridge/test/test_obs_builder.py` | 30 | 52 |
| `PathFollowing4DOFNode._guidance_update_callback` | `.../nodes/path_following_node.py` | 51 | 52 |

**읽는 법**: 대부분은 out_degree가 크고 in_degree가 1~2다 — 즉 "많이 호출당하는 공용
함수"가 아니라 **많이 호출하는 조립 지점**(생성자·파서·콜백)이다. 이런 노드는 변경하면
자기 자신이 깨지지, 다른 곳을 깨뜨리지는 않는다.

예외가 둘 있고, 그쪽이 진짜 위험하다:
- `ObsBuilder.update` — in 22 / out 45. 양방향 모두 크다. ALBC 정책 브리지의 관측 조립
  지점이라 변경 시 실제 파급이 있다.
- 테스트 헬퍼 `_make`(in 28)·`_inputs`(in 22) — 시그니처를 바꾸면 해당 테스트 파일 전체가
  깨진다. characterization 테스트의 단일 실패 지점이다.

## 4. 실행 흐름

| flow | criticality | 노드 |
|:--|--:|--:|
| `on_tick` | 0.48 | 6 |
| `step` | 0.41 | 7 |
| `forward` | 0.40 | 6 |
| `compute_guidance` (ILOS) | 0.40 | 12 |
| `generate_reference` | 0.38 | 6 |
| `interpolate` | 0.38 | 9 |
| `from_message_static` | 0.38 | 6 |
| `wrench_callback` | 0.37 | 3 |
| `wrench_stamped_callback` | 0.37 | 3 |
| `update` | 0.37 | 3 |

최고 criticality가 0.48이고 flow당 6~12노드다. sim은 **얕고 넓은** 구조 — 진입점이 많고
각각이 짧다. slam(최고 0.73, 96노드)과 정반대이므로 회귀 전략도 달라야 한다:
sim은 진입점마다 개별 검증이 필요하고, 한 flow를 덮었다고 다른 flow가 덮이지 않는다.

`compute_guidance`가 12노드로 가장 깊은데, P4에서 4개 헬퍼로 분해한 결과가 그래프에
반영된 것이다(auto-memory `stonefish-sim-p4-plan-approved` 참조).

## 5. 이 그래프가 못 보는 것

- **ROS 토픽 결합**: `thruster_manager/input`·`cmd_pose`·`cmd_vel`은 엣지가 아니다.
  `ros2 topic info -v`로 검증할 것.
- **`stonefish_msgs` → slam 의존**: 크로스 repo라 blast radius가 0으로 나온다.
  "영향 없음"이 아니라 "측정 불가"다.
- **`.scn` physics 파라미터·YAML**: 노드를 만들지 않는다. config로 동작이 바뀌는 변경은
  그래프가 항상 빈 답을 준다 — `grep`으로 형제 파일과 비교할 것.

## 6. 조회

```
repo_root: /workspace/src/stonefish_sim
```

생략하면 오류가 아니라 `status: "ok"`에 0건이 돌아온다. 신뢰 규칙 정본은 워크스페이스
`.claude/rules/code-review-graph.md`.
