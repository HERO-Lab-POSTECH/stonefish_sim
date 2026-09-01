# P4 Investigation Flags

> **2026-09-01 갱신 패스.** 양 repo 코드 건강 감사(37건 적대 검증)에 맞춰 이 파일의 **유령 백로그를
> 정리**했다 — 삭제된 파일을 가리키던 항목, 이미 분해된 god-method, 반증된 가설에 각각 종결·정정
> 배너를 달았다(원 서술은 이력으로 보존). **이번 감사에서 새로 확정된 결함은 이 파일이 아니라
> `.hq/community/posts/finding/008-2026-09-01-code-health-audit.md`가 SSOT**이며, 원 보고 전문은
> `.hq/work/project/audit-2026-09-01/`에 있다. 두 곳에 같은 항목을 중복 기재하지 말 것.

## hybrid position_mode max_force/torque: yaml ↔ code default 불일치 (release 발견, latent)
`hybrid_controller_node.py:73-74`의 declare_parameter default는 `position_mode.max_force=200.0`/`max_torque=50.0`인데, `config/bluerov2/hybrid_controller.yaml:56-57`은 position_mode를 `800.0`/`160.0`(velocity_mode와 동일값)으로 override한다. yaml이 우선하므로 bluerov2 런타임 실제값은 800/160 — 즉 position 모드 포화한계가 velocity 모드와 같아져 "position=정밀(낮은 한계)" 설계 의도와 어긋날 수 있다. README는 코드 default(200/50)를 문서화하도록 교정했으나(release), 어느 값이 진짜 의도인지(yaml 800을 default로 내릴지, default 200을 yaml에도 반영할지)는 owner/런타임(RTX4070) 판단 필요. 동작 변경이라 release에선 미수정.
**[해소 — fix/thrust-map, 2026-08-22]** 힘→PWM 제곱 왜곡 수정과 함께 전 모드 yaml·declare 기본값을 물리 한계 55 N/13.7 N·m로 동기. 위 서술은 이력.

## T4.5 C++ 동시성·QoS 표준정합 제안 (P4 문서화만, RTX4070 sign-off 대상)

이 섹션의 제안은 모두 **현재 동작 변경 없음** — 코드 주석으로 불변식을 명시(T4.5)했으며,
아래 변경은 RTX4070 런타임 검증 후 owner 채택 결정이 필요하다.

1. **센서 토픽 SensorDataQoS(best_effort) 미적용**: `ROS2ScenarioParser.cpp`의 모든
   `create_subscription`/`create_publisher`가 depth=10 기본 QoS(reliable, KeepLast)를 사용한다.
   ROS2 표준은 고빈도 센서 토픽에 `rclcpp::SensorDataQoS()`(best_effort, KeepLast(5))를
   권장한다 — reliable은 재전송으로 지연·대역폭 낭비를 유발할 수 있다.
   변경 시 consumer(EKF/SLAM 노드 등)도 호환 QoS로 동시 수정 필요, RTX4070 런타임 검증 필수.

2. **robot 이름 중복 시 subs_/pubs_ 맵 silent overwrite**: `ROS2ScenarioParser.cpp`의 키가
   `robot->getName() + "/thrusters"` 등 로봇 이름 prefix라서, 시나리오 XML에 동일 이름의
   로봇이 두 개 정의되면 두 번째가 첫 번째를 조용히 덮어쓴다(현재 경고 없음).
   제안: 키 삽입 전 `subs_.count(key) > 0`을 체크해 중복 시 `RCLCPP_WARN` 발행.
   다중 robot 시나리오에서 진단 가능성을 높이는 방어적 개선이나, 동작 변경이라 owner 결정.

3. **단일스레드 spin이 시뮬레이션 step과 콜백을 직렬화**: 현재 `rclcpp::spin(node)`
   (SingleThreadedExecutor)는 시뮬레이션 step 처리와 모든 ROS2 콜백을 하나의 스레드에서
   순차 실행한다. 고빈도 토픽·고부하 시나리오에서 콜백 처리가 시뮬 스텝을 지연시킬 수 있다.
   `MultiThreadedExecutor` + `MutuallyExclusiveCallbackGroup` 도입으로 분리 가능하나,
   `servoSetpoints_`·`subs_`/`pubs_`/`srvs_` 등 공유 상태 전체에 동기화(mutex) 추가가
   필요한 비자명한 재설계 — 별도 사이클로 처리.

## VelocityProfiler 잔재 (T2.2 부분 — interpolator 내부 분기는 Phase 3로)
- **dangling `__all__` 엔트리 제거 완료(T2.2)**: `path_generator/__init__.py:49`의 `'VelocityProfiler'`는 import되지 않는데 `__all__`에 등재돼 `from path_generator import *` 시 AttributeError를 내는 버그였다(class VelocityProfiler가 repo 전체에 부재). 제거함.
- **[2026-09-01 재측정]** 실측 분포는 `cs_interpolator.py` **7곳**·`lipb_interpolator.py` **7곳**·
  `common/trajectory_generator.py` **6곳**(총 20)으로, 아래 "14곳/15곳" 서술은 부정확하다. 또한 이연 근거였던
  `lipb_interpolator.init_interpolator` god-method는 이미 분해됐다(152→**26줄**, L50-75) — 따라서 이 정리는
  더 이상 god-method 작업에 묶여 있지 않고 독립 삭제 가능하다. 잔존 함정: `trajectory_generator.py`가
  이 목록에 없었는데 실제로는 6곳을 갖고 있다(`[DAMPING]` print 포함).
- **interpolator 내부 dead 분기는 Phase 3로 이연**(이력): `cs_interpolator.py`(14곳)·`lipb_interpolator.py`(15곳)에 `_velocity_profiler`/`_use_velocity_profiler` 상태·조건 분기가 광범위 분포. `if self._use_velocity_profiler:`(config 0건이라 항상 False) 안에서 미정의 `VelocityProfiler(...)`를 호출(cs:159, lipb:257) → 활성화 시 NameError(latent-dead). 29곳에 걸친 제거는 LIVE 보간 모듈 수술이라 무방비 제거 시 회귀 위험 큼. Phase 3 god-method 정리(lipb `init_interpolator`는 god-method) 시 characterization 골든 보호 하에 함께 제거. 현재는 unreachable(use_velocity_profiler config 0건)이라 active bug 아님.

## C++ 잔여 null-deref (T1.5 후속 — pre-existing, code-reviewer 발견)
- **robot null-deref (ParseRobot)**: `ROS2ScenarioParser.cpp:254` `robot->getActuator(...)`가 `Robot* robot = getSimulationManager()->getRobot(nameStr)`(L244) 결과를 null 체크 없이 deref. `<robot name="X">`가 미등록 엔티티명이면 getRobot이 null 반환 가능 → segfault(T1.5와 동일 class, 한 줄 아래). pre-existing이라 T1.5 scope 밖(surgical 유지)으로 inline 미수정. fix: L244 후 `if(robot == nullptr) { RCLCPP_ERROR(...); return false; }`. ParseAnimated는 안전(L459 getEntity→L460 null 체크 존재).

## C++ 시간동기 (T1.6 — RTX4070 sign-off 환경에서 구현, 이 컨테이너 검증 0)
- **wall-clock 타임스탬프 + /clock 부재 (무조건적 live defect)**: 모든 센서/TF publish가 `rclcpp::Clock(RCL_ROS_TIME).now()`(wall-clock)로 스탬프하고 `Sample.getTimestamp()`(센서 샘플 sim-time)를 무시한다(`ROS2Interface.cpp:110,128,155,...` 다수, `ROS2SimulationManager.cpp:321,333,361,500,...`). `/clock` publisher·`rosgraph_msgs::Clock`·`use_sim_time` 파라미터가 src/launch 어디에도 없음. 게다가 `getSimulationClock()`(`ROS2SimulationManager.cpp:82-86`) 자체가 wall-clock 구동이라 시뮬레이션이 real-time-locked. → EKF/TF 시간동기 깨짐, replay/결정성 불가.
- **fix 설계(atomic 3-part, 분리 금지)**: (a) `rosgraph_msgs::msg::Clock` publisher 추가 + `getSimulationTime()`(`SimulationManager.h:437`, Scalar 초)을 SimulationStepCompleted마다 publish; (b) 각 센서 `header.stamp`를 `s.getTimestamp()`(Sample s 이미 scope 내)로 빌드한 `rclcpp::Time`으로 교체, TF는 `getSimulationTime()`; (c) 노드에 `use_sim_time=true`. ★half-applied 금지 — sample 스탬프만 하고 /clock 없으면 consumer가 없는 클럭 참조, /clock만 하고 restamp 없으면 센서 데이터 wall-clock 태그 잔존.
- **검증 한계**: 이 컨테이너엔 colcon 빌드·런타임 둘 다 부재 → T1.6은 검증 0. T2(`rosgraph_msgs`/`rclcpp::Time` 변환 컴파일) + T3(RTX4070 런타임: `/clock` advance·`tf2_echo` sim-time·EKF time-jump 미거부) 필수. atomicity는 정적으로 부분검증 불가 — T3 필수.
- **follow-up(별도, T1.6에 fold 금지)**: `getSimulationClock()`(`:82-86`) wall-clock override를 physics step accumulator 기반으로 바꾸는 건 real-time pacing에 영향 주는 큰 아키텍처 변경 → 별도 cycle.

## H4 잠재 위험 (T1.2 후속 — catalogue-only, 현재 active bug 아님)
- **position_controller.yaml non-wildcard 키**: `config/bluerov2/position_controller.yaml:4` 키 `position_controller_4dof:`(non-wildcard)는 노드 생성자명 `pid_4dof_controller`(position_controller_node.py:55)와도, 파일명 stem과도 불일치. 현재 launch 참조 0건(catalogue-only)이라 active bug는 아니나, 미래에 launch로 연결되면 T1.2/H4와 동일한 silent 파라미터 fallback 발생. launch 연결 시 키를 `/**:`로 정합 필요(repo 컨벤션). T1.2에서 hybrid만 고친 이유: hybrid가 유일 LIVE 경로의 active bug, position은 catalogue-only라 scope 분리.
- **✅ 노드명 충돌 `pid_4dof_controller` — 해소(2026-09-01 확인)**: `velocity_controller_node.py`가 삭제돼(`f5acd52`/`8f3aedf`) `pid_4dof_controller` 생성자명은 `position_controller_node.py` 단독이다. 예상대로 자동 해소됐다.

## ④ 고도화 제안 (P4 진행 중 도출 — owner 채택 결정 필요, P4 미구현)
- **가속도 feedforward end-to-end wiring (T1.3 후속)**: T1.3에서 `PositionController`의 position 모드 feedforward 계약을 `M·a`(힘=질량×가속도)로 교정했다(이전 `M·velocity`=운동량은 차원 오류). 그러나 이 가속도를 *공급*하려면 데이터 흐름 wiring이 필요하다 — 현재 (a) `path_following_node.py`가 `msg.acceleration`을 채우지 않고 `msg.velocity`만 publish, (b) `hybrid_controller_node.py:106-113` cmd_callback이 `msg.acceleration`을 안 읽음. `TrajectoryPoint.msg`는 `geometry_msgs/Accel acceleration` 필드를 이미 보유하므로 메시지 계약 변경은 불필요. **이 wiring은 신규 동작(④고도화)이라 owner 채택 결정 필요** — 미구현 시 position 모드 feedforward는 0(architect 확인: LIVE 경로에서 원래도 position 진입 시 velocity=0이라 동작 변경 없음). 구현 시 RTX4070 런타임 sign-off로 궤적 추종 개선 확인. **[P2 2026-08-22 해소 — 형태 변경]**: 리뷰에서 msg 경유 wiring이 기각됨 (outer surge 상시 clip으로 guidance 원신호 미분이 unmatched disturbance). 대신 `CascadeController` 내부에서 clip 후 v_sp를 수치미분해 `M_eff·v̇_sp`를 inner에 합산하는 구조로 구현(`feat/model-injection`). `TrajectoryPoint.acceleration`은 계속 미사용, position 모드 accel ff도 비대상(정지점 운용) 유지. MINOR-3/4 기록: position 모드 `accel_ff` 경로는 hybrid 경유로 도달 불가한 dead path — 의도적(정지점에서 acc=0), `test_feedforward_dimensional.py`가 단위 계약만 고정. **[P2 후속 2026-08-22]**: cascade acc_ff는 기본 비활성(`accel_ff_cutoff_hz: 0.0`) — 단일 스택 폐루프 4런에서 이득 미입증: leg 사행은 ON/OFF 무관 발생(ON 1/1 runE 0.456 / OFF 1/3 runN 0.459, 청정 runF·runM 0.227~0.228). 사행의 실제 원인은 guidance 코너 감속이 outer 위치항의 v_sp clip 포화에 무력화되는 쌍안정으로 판정되어 `guidance_speed_margin` 캡으로 처방(같은 커밋). v_sp가 outer P 경유 차량 위치의 함수라 acc_ff의 미분은 자기되먹임 경로이기도 하다. 참고: 중간의 runG~runL은 TERM 생존 고아 컨트롤러 누적(wrench 50→250 Hz)에 오염된 무효 측정. 켜려면 폐루프 재검증 필수. **[2026-08-23 정정]**: 위 런 비교(runE/runF/runM/runN)는 `control_mode` 레이스로 활성 컨트롤러가 런마다 갈린 상태에서 이뤄져 **교란**됐다 — runF/runM은 velocity, runE/runN은 cascade였다. 따라서 "acc_ff ON/OFF 무관"도, "사행 원인 = 감속 무력화 쌍안정"도, `guidance_speed_margin`이 그것을 처방했다는 것도 성립하지 않는다(캡은 cascade 전용 코드인데 청정 런 runO/P/Q는 전부 velocity). 레이스는 `db2c3e9`에서 latched QoS로 차단했다. acc_ff·캡·모델 주입의 실효는 **cascade 모드를 명시 지정한 재캠페인**에서만 판정 가능하다.
- **0.1 feedforward_gain 재정당화 (T1.3 후속)**: `position_controller_node.py`의 `feedforward_gain=0.1`은 틀린 `M·velocity` 항을 누르려던 fudge factor였다. `M·a`로 교정된 뒤엔 올바른 feedforward는 gain `1.0`(전체 모델) 또는 의도적 calibration knob(예: `unified_controller.yaml:62`의 `0.8`)을 써야 한다. **0.1을 hybrid 노드로 전파 금지**(워크어라운드 cargo-cult). 가속도 wiring 채택 시 함께 재정당화.
- **accel_ff 명시 rename (Option B)**: 현재 `PositionController.compute_control`은 `vel_ff`(velocity 모드 setpoint) + `accel_ff`(position 모드 feedforward) 두 인자를 받는다. 더 깔끔한 설계는 mode별 인자를 완전 분리하는 것이나, blast radius(HybridController 위임 + 호출자) 때문에 P4에선 최소 변경(accel_ff 추가)만 했다. 향후 리팩토링 후보.

## T4.4 측정 결론 (sim P4 실행 중 — 가정 반증, owner 범위 결정 대기)
P4_FLAGS의 즉시-수정 가정을 측정이 대거 반증함. 4건 중 teleop만 즉시 삭제 확정, 나머지는 latent/의도/설계 사안.
- **✅ teleop_manager = 삭제 완료(2026-09-01 확인)**: `stonefish_control/` 하위에 `stonefish_teleop_manager/`가 더 이상 없다. 아래는 결정 당시 기록(보존). `stonefish_control/stonefish_teleop_manager/`는 README.md 1개뿐(package.xml/setup.py 부재 → ament 미인식). README가 "not yet implemented placeholder"로 명시, 코드/launch 참조 0건. = dead stub. 부활은 README 로드맵대로 별도 기능 작업.
- **Waypoint __hash__ 부재 = latent**: `common/waypoint.py:68` `__eq__` 정의 + `__hash__` 부재 → Python이 `__hash__=None` 설정 → unhashable(set/dict 키 불가). 단 측정 결과 Waypoint를 set/dict 키로 쓰는 LIVE 사용처 **0건**. 현재 미발현. 수정(=`__hash__` 추가 또는 `__eq__` 재검토)은 신규 동작이라 owner 결정.
- **gravity 9.82 = 의도적 calibration(버그 아님 가능성 높음)**: `dynamics_loader.py:123` default 9.82 + `dynamics_params.yaml` 전체(부력식 197.7N 역산 포함)가 9.82 일관. 표준 9.80665/9.81과 다르나, 실제 부력 평형을 9.82로 맞춘 **하드웨어 보정값** 성격(CLAUDE.md "하드웨어는 미니멀 모델이 못 보는 보정 노브" 원칙). 임의로 9.81로 바꾸면 부력 평형 깨짐. 표준화하려면 부력식 동시 재계산 필요 — owner 결정.
- **data/ 경로 = C++ 런타임 --data 베이스 의존(설계 사안, O6/O7)**: `.scn`이 `data/robots/.../meshes/*.obj`를 **작업디렉토리 상대 경로**로 참조(package:// 아님). scenarios/(7개 진입 .scn) ↔ data/(로봇 부품 .scn+mesh) 분리, CMakeLists가 둘 다 install. Stonefish C++가 `--data` 베이스 경로로 해석하므로 ROS package 경로화하려면 .scn 다수 + C++ 로더 + 런타임(RTX4070) 검증 동시 필요. 단순 수정 아닌 설계 결정.

## P4 후보 (P2 발견 — T4.4에서 정밀 재측정)
- `bezier_curve.py::BezierCurve.__init__`: tangents를 list로 받으면 `tangents[0]+tangents[1]`이 list concat(길이 6)이 되어 order=3/4 경로에서 np.dot shape 오류. assert는 len==3 list를 허용하나 내부 연산은 np.array만 정상. 수정안: 생성자에서 tangents/pnts를 np.asarray로 정규화. (동작 변경이라 P4에서 처리)
  - **T4.4 재측정**: L63-69 for loop이 tangents 원소를 검증만 하고 변환 안 함(np.asarray 미적용). LIVE 호출처 `bezier_curve.py:214`(`generate_cubic_curve`, cs_interpolator:80에서 LIVE)가 `[tangents[i], tangents[i+1]]` 전달. tangents 원소가 np.array면 정상(element-wise), list면 concat 버그. test_bezier_curve.py 통과 중이라 현재 numpy 경로로 동작 추정 → **LIVE 트리거 가능성은 caller가 list를 넘기는지에 달린 latent**. np.asarray 정규화는 방어적으로 옳으나 동작 변경이라 owner 결정. 실제 LIVE 트리거 여부 정밀 확인(generate_cubic_curve 내부 tangents 생성 타입) 선행 권장.

## ✅ P4 후보 (P3.0 컨벤션 조사 발견) — **해소(2026-09-01 확인)**

> `controllers/velocity_controller_node.py`가 repo에서 삭제됐다(`f5acd52`/`8f3aedf`) — `pid_4dof_controller`
> 생성자명을 쓰는 노드는 이제 하나뿐이라 충돌 조건 자체가 소멸했다. 아래는 발견 당시 기록(보존).

- **노드명 중복**: `controllers/velocity_controller_node.py:55`와 `nodes/position_controller_node.py:55`가 둘 다 `super().__init__('pid_4dof_controller')`. 동시 실행 시 ROS2 고유 노드명 요구(RMW 강제)를 위반해 노드 등록 충돌. 근거: [rmw validate_node_name.c](https://github.com/ros2/rmw/blob/master/rmw/src/validate_node_name.c). 수정안: 각자 고유 이름(예: `velocity_controller`·`position_controller`)으로 초기화. 동작 변경(노드명 의존 토픽 네임스페이스 영향 가능)이라 P4에서 처리.

## P4 후보 (P3 실행 중 강등) — **2026-09-01 부분 해소**

> **삭제로 소멸한 3항목**: `controllers/velocity_controller_node.py`·`controllers/unified_controller_node.py`가
> repo에 더 이상 없다(`f5acd52`/`8f3aedf`, 2026-09-01 `find` 실측). 따라서 아래 "velocity_controller_node
> dead 파일"·"unified_controller_node orphan"·"velocity dead 파일 §2.2 import 미교정" 세 항목은 **종결**이며
> 이력으로만 보존한다. **여전히 열린 항목은 아래 넷째 `trajectory nodes/__init__.py eager import`와
> 다섯째 라이선스 헤더 불일치 둘뿐이다.**

- **~~velocity_controller_node dead 파일~~ (삭제됨)**: `controllers/velocity_controller_node.py`는 존재하지 않는 `from stonefish_control.controllers.pid_4dof import PID4DOF`(L36)에 의존해 `ros2 run` 시 즉시 ImportError. P3에서 **console_scripts 엔트리만 삭제**(기동 불가 노드라 배포 토픽 그래프에 부재 → 동작보존). 파일 자체와 복구(=`pid_4dof.py` 신규 작성 또는 노드 재설계)는 신규 동작이라 P4. dead 상태는 `test/test_characterization_node_entries.py::test_g2_velocity_node_is_dead`가 동결(pid_4dof 부활 시 RED).
- **~~unified_controller_node orphan~~ (삭제됨)**: `controllers/unified_controller_node.py`는 setup.py console_scripts에 미등록(dead *entry* 아닌 dead *file*). 상대 import 사용(§2.2 준수). P3에서 보존(삭제=잠재 의도 코드 손실 위험, 등록=미동작 노드를 동작시킴=신규 동작). 운명(삭제/등록/이동) 결정은 P4.
- **~~velocity dead 파일 §2.2 import 미교정~~ (파일 삭제로 소멸)**: `controllers/velocity_controller_node.py`는 intra-package 절대 import(`from stonefish_control.controllers.pid_4dof import`, `from stonefish_control.control_interfaces import`)를 유지 — §2.2 상대 통일 대상이나 dead 파일이라 P3 T3b 변환에서 제외(working 노드 hybrid/position만 교정). P4 복구(또는 삭제) 시 함께 정리. dead 파일을 nodes/로 옮기지도 않음(기동 불가 노드는 정렬 무의미).
- **trajectory nodes/__init__.py eager import**: `stonefish_trajectory_manager/.../nodes/__init__.py:1-2`가 `from .path_generator_node import main` 등으로 노드를 eager import → 이 패키지를 import하면 import-time에 rclpy가 끌려옴(§2.2 모델 패키지인데 자기 규칙 위반, 동작보존 중립 아님). control의 `nodes/__init__.py`는 inert(comment-only)로 올바름. eager import 제거는 import-time 동작 변경이라 P4. 상태는 `test_g4_trajectory_nodes_init_eager_is_known_p4`가 동결(P3에서 고치면 RED).
- **소스 헤더 라이선스 ↔ 패키지 메타데이터 불일치**: P3 T1에서 3개 setup.py를 package.xml SSOT에 맞춰 `GPL-3.0`으로 정렬했으나, **소스 파일 헤더는 45개가 Apache-2.0**(`Licensed under the Apache License, Version 2.0`), GPL 헤더는 3개뿐. 즉 패키지는 GPL 선언이나 다수 소스가 Apache 헤더 → 법적 불일치. 헤더 일괄 정정은 (a) 대규모(45파일), (b) 어느 쪽이 진짜 의도인지 owner/법적 판단 필요 → P4. (메타데이터는 package.xml·CLAUDE.md가 일관되게 GPL-3.0이라 그쪽으로 정렬함.)

## P4 후보 (P3 T6 재감사 — 거대 모듈/메서드) — **2026-09-01 AST 재측정, 절반 종결**

아래 표가 현재 실측이다. 이전 서술은 줄번호·줄수가 전부 drift했고 한 항목은 파일 자체가 없다.

| 대상 | P3 T6 기록 | 2026-09-01 실측 | 판정 |
|:--|:--|:--|:--|
| `ilos_guidance.py::compute_guidance` | 319줄 (L632-951) | **66줄** (L927-992), 파일 1115줄 | **종결** — 분해 완료 |
| `lipb_interpolator.py::init_interpolator` | 152줄 (L61-213) | **26줄** (L50-75), 파일 545줄 | **종결** — 분해 완료 |
| `los_guidance.py::update` | 177줄 (L153-330) | **파일 부재** | **유령 항목** — repo에 `los_guidance.py`가 없다(`find` 실측). 원래 오기재 |
| `path_following_node.py::__init__` | 170줄 (L48-218) | **199줄** (L39-237), 파일 547줄 | **열림, 악화** |
| `path_following_node.py::_guidance_update_callback` | 146줄 (L335-481) | **145줄** (L356-500) | **열림** |
| ilos curvature estimator 4종 | L446/498/547/598 | L456/508/560/611, 여전히 4종 | **열림** |

- **남은 실작업은 `path_following_node.py` 하나**: `__init__` 199줄 + `_guidance_update_callback` 145줄이
  파일 547줄의 63%를 차지한다. Node 콜백·구독 분해는 토픽 그래프·타이밍 변경이라 characterization 선작성 필요.
- **curvature estimator 4종 통합**은 공식이 서로 달라 수치 변경 — 통합 자체가 목적이면 어느 공식이 SSOT인지
  먼저 결정해야 한다(`_estimate_signed_curvature`의 부호 규약은 2026-09-01 확인 시점에 docstring이
  구현과 정합하도록 이미 정정돼 있다 — 아래 P7 항목 참조).

## P4 sign-off 의무 (P3 변경의 런타임 검증 — runnable ROS2 필요)
P3 안전망은 정적·국소 검증이라 런타임 rclpy registry 의미를 못 덮는다. 아래는 `colcon build`+`ros2 launch` 환경에서 확인할 것.
- **[fix/thrust-map] yaw 스텝 응답 오버슈트/진동 확인**: inner yaw Kp=4는 Izz=0.13(점질량, added inertia 미실측) 기준 명목 ω_c≈31 rad/s로 선형축(≈2)보다 공격적 — 로터 지연(τ≈0.3 s) 대비 진동 성향 가능. 실기 yaw 스텝으로 오버슈트/진동 확인, 필요시 Kp_r 하향. 역추력맵·물리 포화 전체(P1 lawnmower A/B)도 동일 sign-off 대상.
- **T5 VehicleParams 추출 런타임 검증**: runnable ROS2에서 (a) 컨트롤러 노드의 `ros2 param list`와 `Vehicle` 생성이 동결된 36-call 골든 마스터 + 8개 raise 타이밍을 재현하는지, (b) `ParameterAlreadyDeclaredException` 미발생, (c) 실제 rclpy의 list→array 타입 변환 하에서 `len(cog)!=3` 검증이 동일하게 동작하는지 확인. 통과 전까지 §4 git-revert 롤백(loader-split 커밋) armed 유지.
- **T3 노드 이동/import 변환 런타임 검증**: 각 launch 파일에 대해 `colcon build` 후 `ros2 launch` 스모크 1회로 (a) console_scripts 좌변이 여전히 노드를 해석하는지, (b) 상대 import가 런타임에 올바른 심볼로 binding되는지(정적 target-set diff가 못 보는 동명이클래스·__init__ 섀도잉), (c) install 트리에 stale `thruster_allocator` 모듈이 안 남는지 확인.

## characterization 안전망 강화 백로그 (P3 사각지대, 비차단)
P3 characterization이 못 덮는 값 정확성 측면(code-reviewer 지적). 이번엔 코드 정독으로 보존 확인됐으나 향후 회귀 취약.
- ~~Vehicle 속성→값 매핑 미검증~~ → **P3에서 해소**(`test_vehicle_init_attribute_value_mapping`: cog≠cob 구별값으로 10개 속성 직접 단언).
- ~~declare default 값 미검증~~ → **P3에서 해소**(`test_vehicle_init_declare_defaults_flow_when_param_absent`: density 1028.0 흐름 확인).
- **fake node ↔ 실제 rclpy 의미 차이**: fake node는 list→array 변환·타입 검증·`ParameterValue` 래핑을 안 한다. 골든 마스터가 "rclpy 동등성"까지 보증한다고 과신 금지 — 위 T5 sign-off로 닫는다.

## P5 cascade 재설계 이월 (경로추종 position-cascade — p5-path-cascade)
P5에서 ILOS의 cross-track 이중보정(heading arctan + 비표준 sway PID)을 제거하고, 별도 `CascadeController`(outer position-P → inner velocity-PI)로 cross-track을 단일 채널 처리하도록 재설계했다. 아래는 이번 범위에서 의도적으로 단순화·이월한 천장으로, RTX4070 실기 측정 후 검토한다. 설계 SSOT: `/workspace/.sp/plans/2026-06-25-path-following-position-cascade.md`.
- **inner M·a feedforward (accel_ff = M·v̇_sp)**: `v_sp` 수치미분이 노이즈를 증폭할 위험으로 미구현. 현재 `CascadeController`는 생성자에서 `mass`/`inertia_zz`를 받되 `compute_control`에서 미사용(시그니처 동형성 유지 — P4에서 필터링 후 추가 시 호출부 무변경). 추가 시 v̇_sp 저역통과 필터 설계 선행.
- **outer Kp=[0.4,0.4,0.3,0.8] / v_sp_limit=[0.5,0.3,0.25,0.6]**: 시간상수 분리 원칙(outer가 inner보다 느림)에서 도출한 초기값. 닫힌루프 정착시간·오버슈트는 컨테이너 미검증이라 RTX4070 실기 측정으로 미세조정. v_sp_limit는 OWNER DECISION #1(c) ALIGN 보수값. **[갱신 — fix/thrust-map]** cruise 1.0과의 모순 해소로 surge 0.5→1.2(게이트 테스트로 강제), sway는 P6에서 0.5.
- **모드 전환 첫 tick 점프**: velocity→cascade 진입 시 `set_mode`의 reset + outer 출력 clamp(v_sp_limit)로 1차 완화. integral preloading(전환 시 적분기를 직전 속도로 시드)은 실기 관측 후 검토. 또한 cascade_controller 미생성 환경에서 `set_mode('cascade')` 시 `active_mode='cascade'`로 보고하나 실제 라우팅은 position으로 폴백하는 latent footgun(Task 4 리뷰 지적) — 가드 추가는 P4.
- **코너 추종 (sway=0 + 고정 lookahead 3m)**: cross-track sway 채널 제거로 코너에서 cascade outer만 횡오차를 닫는다. adaptive lookahead 재활성·curvature preview는 미구현(현재 `adaptive_lookahead: false`). 코너 추종 정확도는 실기 sign-off 항목. **[2026-08-23 부분 해소]**: velocity 모드용으로 guidance 층에 cross-track 위치 피드백(`cross_track_gain`, 기본 0.4)을 되살렸다 — P5가 제거한 것은 ILOS heading arctan과 **중복**되던 비표준 sway PID였고, 이번 것은 heading 보정이 닿지 않는 정적 오프셋만 담당하는 단일 P항이라 이중보정이 아니다. cascade는 outer 위치 P가 같은 일을 하므로 종전대로 별 채널. 폐루프 runT: e_y max 0.254 m·leg RMS 0.042(기준선 runO 0.531/0.263). adaptive lookahead·curvature preview는 여전히 미구현.
- **닫힌루프 안정성·정착시간·thruster allocation 포화**: 단위테스트(84 passed)는 순수 산술(CascadeController 손계산·ILOS 축소 골든)과 정적 게이트만 덮는다. 닫힌루프 안정성·정착시간·thruster 포화는 컨테이너 골든 미검증 → `colcon build` + `ros2 launch` + RTX4070 실기 sign-off 필요. 통과 전까지 cascade 모드는 검증 미완 상태로 간주.

## P6 코너 feedforward 이월
- [cascade] sway_ff_gain=0.1은 m/Kp_inner 모델 추정. 항력·부가질량 미반영 →
  RTX4070 실기에서 코너 e_y 측정하며 튜닝(과소면 ↑). 0이면 비활성(P5 거동).
- [cascade] v_sp_limit sway 0.5로 상향 — inner/thruster 포화 한도 실기 미검증.
- [cascade] feedforward는 kinematic(v²κ)만. inner M·a feedforward(P5 이월)는 별개
  — 두 ff는 독립이라 동시 적용 시 결합 튜닝 필요.
- [cascade] sway Kp 0.4→0.5 + v_sp_limit 0.3→0.5 + 곡률 ff 활성으로 §P5 "닫힌루프
  안정성·정착시간·thruster allocation 포화" sign-off 범위가 확장됨 — feedforward+상향
  게인 결합 거동(과도 응답·정착시간·포화)을 RTX4070 실기에서 재확인해야 한다.
- [cascade] ~~reset() staleness~~ (P6 최종리뷰 발견 → **2026-08-21 수정됨**,
  `fix/guidance-reset-staleness`): ILOS.reset()이 _signed_curvature_filtered·
  _current_curvature를 안 지웠다(P5부터 잔존). reset()에 두 변수 0.0 추가 —
  ALOS는 super().reset() 경유라 한 곳 수정으로 양쪽 해소. 회귀 테스트
  `test_reset_clears_curvature_state`(ILOS)·`test_alos_reset_clears_curvature_state`.
  corner-entry 필터 lag(tau_up)는 여전히 sway_ff_gain과 함께 실기 튜닝 체크리스트.

## P7 코너 cascade 결함 A+C 처방 (2026-06-25)

### 적용 (RTX4070 실기 검증 이월)
- **A (r_d 부호)**: `ilos_guidance.py` `_compute_body_velocities`에서 r_d가
  `_signed_curvature_filtered`(부호 있음) 사용. 좌/우회전 r_d 부호는 합성 경로
  실측 골든으로 고정. **실기에서 좌·우 코너 모두 heading이 경로를 향하는지
  프로브 재측정 필요** (e_yaw 부호가 정렬 방향).
- **C (yaw 게이트)**: `cascade_controller.py` outer sway 채널에
  `max(cos(e_yaw),0)` 게이트. cos 게이트 강도는 고정(파라미터 미노출 — YAGNI).
  **실기에서 코너 e_y 2.49m→<0.5m, 동그란 overshoot 소멸 확인 필요**.

### 미처리 (이번 범위 밖)
- **B (heading chi_p 점프)**: `chi_d=chi_p`가 lookahead가 아닌 발밑 접선(s+0.1m)을
  봐 piecewise-linear 코너에서 계단 점프. A+C 프로브 후 재평가 결정. C가 overshoot를
  완화하면 B는 불필요할 수 있음.
- **✅ ~~[ilos docstring] `_estimate_signed_curvature` 부호 오류~~ — 수정 완료(2026-09-01 확인)**:
  현재 `ilos_guidance.py:511-513`은 "Positive = right turn (starboard) / Negative = left turn (port)"로
  구현과 정합하며, 부호 SSOT가 `test_characterization_alos` 골든임을 명시하는 주석까지 붙어 있다.
  아래는 발견 당시 기록(보존). `ilos_guidance.py:504-506`의
  docstring("Positive = left turn")이 구현(좌회전→κ_signed<0, 우회전→κ>0)과 **반대**다.
  Task 1 리뷰 발견(MEDIUM). 기존 결함이고 r_d 산식 주석(L891-893)에 올바른 관례가
  명시되어 있으나, 함수 docstring 자체는 여전히 거짓이라 후행 개발자가 이 함수를
  먼저 보면 잘못된 부호를 내재화한다. 구현이 SSOT이고 주석이 맞음.
  **fix=docstring L504-506을 "Positive = right turn / Negative = left turn"으로 정정**(별도 작업 — 동작 무관).
- **[cascade docstring] `compute_control` Returns 절 불완전**: `cascade_controller.py`의
  Returns 절이 `debug_info['e_outer']`를 raw body 오차로만 설명하나, 결함 C 수정 후
  sway 슬롯은 cos(e_yaw) 게이트된 값이다. Task 2 리뷰 발견(LOW).
  **fix=Returns 절에 "sway 슬롯은 게이트 후 값" 한 줄 추가**(별도 작업).
- **[cascade 주석] 기하학적 부정확성**: `cascade_controller.py` 게이트 주석 "body sway가
  world cross-track으로 기여하는 성분이 정확히 cos(e_yaw)"는 기하학적으로 부정확
  (실제는 cos(yaw_curr); cos(e_yaw)는 제어 의도 스케일링). Task 2 리뷰 발견(Nit).
  동작 무관이지만 후행 개발자 혼동 위험.

## 감사 확정 버그 4건 수정 이월 (2026-08-21, fix/audit-bugs)

opus 적대검증 CONFIRMED 4건을 수정했다. 컨테이너는 GPU 없어 닫힌루프 검증 불가 —
아래는 RTX4070 실기 sign-off 항목이다. 정적 게이트·골든은 이미 GREEN(139 passed).

### 실기 sign-off 필요
- **[control.nodes] position_controller dt 클램프 경계**: `nodes/position_controller_node.py`
  control_loop에 `dt = max(0.001, min(dt, 0.1))`를 추가했다(형제 `hybrid_controller_node.py`
  패턴 미러). 경계값 0.001/0.1은 **형제와의 정합에서 온 값이지 이 노드의 제어주기로
  실측 튜닝한 값이 아니다** — control_rate가 10Hz 미만이면 상한 0.1s가 정상 틱을
  잘라 적분이 실제보다 느려진다. 실기에서 (a) 정상 운전 중 클램프가 발동하지
  않는지(발동 빈도 로깅), (b) 일시정지 후 재개 시 적분항 폭주가 사라졌는지 확인.
  rclpy 부재로 컨테이너 검증은 AST 게이트(`test/test_position_controller_dt_gate.py`)까지다.
- **[trajectory.path_following] ALOS r_d 부호**: `alos_guidance.py`의 r_d가
  `_signed_curvature_filtered`(부호 있음)를 쓰도록 교정했다 — 부모 ILOS의 P7 결함 A
  수정을 ALOS가 `compute_guidance` 전면 오버라이드 탓에 물려받지 못한 건이다.
  좌/우 코너 r_d 부호 반대는 합성 경로 골든으로 고정(`test_characterization_alos.py`).
  **실기에서 ALOS 모드로 좌·우 코너 모두 heading이 경로를 향하는지 프로브 재측정
  필요** — ILOS 결함 A와 동일한 sign-off 항목이며, ILOS 프로브와 함께 수행한다.
  ALOS는 필터 상태(`_signed_curvature_filtered`)를 이번에 처음 갱신하므로 코너 진입
  필터 lag(tau_up=0.3)도 ILOS와 같은 튜닝 체크리스트에 든다.

### 미처리 (의도적 — 이번 범위 밖)
- **[alos] ~~`reset()` 필터 staleness~~** (**2026-08-21 수정됨** — ILOS와 한 건으로
  `fix/guidance-reset-staleness`에서 공통 `reset()` 한 곳 수정, 상속 고정 테스트 포함.
  위 P6 항목 참조).
- **[ros2.parser] `ActuatorType::PUSH` 미와이어링**: 이미 `RCLCPP_WARN`으로
  "not supported"를 내보내고 있어 현상 유지가 최소 diff — 손대지 않았다.
- **[control.cascade] anti-windup이 allocator 스케일링을 인지하지 못함 (P2 2026-08-22 발견)**:
  back-calculation은 컨트롤러 자체 55 N clip 기준으로만 적분을 되돌리는데, 하류
  thruster_allocator가 per-thruster T_max 20.68 N 초과 시 wrench를 균등 스케일링
  (launch log WARN 실측 0.43~0.7배)하므로 실제 인가 wrench는 컨트롤러 인지값보다
  작다 — 구조적 사실(코드 확인). 단일 스택 검증 런 기준 증거: acc_ff ON인 runE에서
  포화 경고 1459건과 추종 열화 동반(OFF·무포화 runF는 청정). 근본 처방 후보는
  allocator 인가율 피드백(스케일 계수를 컨트롤러에 회신) 또는 우선순위 할당
  (yaw 권한 보전) — 신규 동작이라 owner 채택 결정 필요.
- **[control.docs] 패키지 README에 cascade 모드 부재**: `stonefish_control/stonefish_control/README.md`가
  Position/Hybrid(velocity·position)만 서술하고 P5의 cascade 모드·CascadeController를
  다루지 않는다(mode 목록도 두 개뿐). P5 이전부터의 공백으로 P2 범위 밖 — README
  정비 시 cascade 절(아키텍처·게인·v_sp_limit·ff 정책) 추가 필요.
