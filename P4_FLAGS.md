## hybrid position_mode max_force/torque: yaml ↔ code default 불일치 (release 발견, latent)
`hybrid_controller_node.py:66-67`의 declare_parameter default는 `position_mode.max_force=200.0`/`max_torque=50.0`인데, `config/bluerov2/hybrid_controller.yaml:56-57`은 position_mode를 `800.0`/`160.0`(velocity_mode와 동일값)으로 override한다. yaml이 우선하므로 bluerov2 런타임 실제값은 800/160 — 즉 position 모드 포화한계가 velocity 모드와 같아져 "position=정밀(낮은 한계)" 설계 의도와 어긋날 수 있다. README는 코드 default(200/50)를 문서화하도록 교정했으나(release), 어느 값이 진짜 의도인지(yaml 800을 default로 내릴지, default 200을 yaml에도 반영할지)는 owner/런타임(RTX4070) 판단 필요. 동작 변경이라 release에선 미수정.

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
- **interpolator 내부 dead 분기는 Phase 3로 이연**: `cs_interpolator.py`(14곳)·`lipb_interpolator.py`(15곳)에 `_velocity_profiler`/`_use_velocity_profiler` 상태·조건 분기가 광범위 분포. `if self._use_velocity_profiler:`(config 0건이라 항상 False) 안에서 미정의 `VelocityProfiler(...)`를 호출(cs:159, lipb:257) → 활성화 시 NameError(latent-dead). 29곳에 걸친 제거는 LIVE 보간 모듈 수술이라 무방비 제거 시 회귀 위험 큼. Phase 3 god-method 정리(lipb `init_interpolator`는 god-method) 시 characterization 골든 보호 하에 함께 제거. 현재는 unreachable(use_velocity_profiler config 0건)이라 active bug 아님.

## C++ 잔여 null-deref (T1.5 후속 — pre-existing, code-reviewer 발견)
- **robot null-deref (ParseRobot)**: `ROS2ScenarioParser.cpp:254` `robot->getActuator(...)`가 `Robot* robot = getSimulationManager()->getRobot(nameStr)`(L244) 결과를 null 체크 없이 deref. `<robot name="X">`가 미등록 엔티티명이면 getRobot이 null 반환 가능 → segfault(T1.5와 동일 class, 한 줄 아래). pre-existing이라 T1.5 scope 밖(surgical 유지)으로 inline 미수정. fix: L244 후 `if(robot == nullptr) { RCLCPP_ERROR(...); return false; }`. ParseAnimated는 안전(L459 getEntity→L460 null 체크 존재).

## C++ 시간동기 (T1.6 — RTX4070 sign-off 환경에서 구현, 이 컨테이너 검증 0)
- **wall-clock 타임스탬프 + /clock 부재 (무조건적 live defect)**: 모든 센서/TF publish가 `rclcpp::Clock(RCL_ROS_TIME).now()`(wall-clock)로 스탬프하고 `Sample.getTimestamp()`(센서 샘플 sim-time)를 무시한다(`ROS2Interface.cpp:110,128,155,...` 다수, `ROS2SimulationManager.cpp:321,333,361,500,...`). `/clock` publisher·`rosgraph_msgs::Clock`·`use_sim_time` 파라미터가 src/launch 어디에도 없음. 게다가 `getSimulationClock()`(`ROS2SimulationManager.cpp:82-86`) 자체가 wall-clock 구동이라 시뮬레이션이 real-time-locked. → EKF/TF 시간동기 깨짐, replay/결정성 불가.
- **fix 설계(atomic 3-part, 분리 금지)**: (a) `rosgraph_msgs::msg::Clock` publisher 추가 + `getSimulationTime()`(`SimulationManager.h:437`, Scalar 초)을 SimulationStepCompleted마다 publish; (b) 각 센서 `header.stamp`를 `s.getTimestamp()`(Sample s 이미 scope 내)로 빌드한 `rclcpp::Time`으로 교체, TF는 `getSimulationTime()`; (c) 노드에 `use_sim_time=true`. ★half-applied 금지 — sample 스탬프만 하고 /clock 없으면 consumer가 없는 클럭 참조, /clock만 하고 restamp 없으면 센서 데이터 wall-clock 태그 잔존.
- **검증 한계**: 이 컨테이너엔 colcon 빌드·런타임 둘 다 부재 → T1.6은 검증 0. T2(`rosgraph_msgs`/`rclcpp::Time` 변환 컴파일) + T3(RTX4070 런타임: `/clock` advance·`tf2_echo` sim-time·EKF time-jump 미거부) 필수. atomicity는 정적으로 부분검증 불가 — T3 필수.
- **follow-up(별도, T1.6에 fold 금지)**: `getSimulationClock()`(`:82-86`) wall-clock override를 physics step accumulator 기반으로 바꾸는 건 real-time pacing에 영향 주는 큰 아키텍처 변경 → 별도 cycle.

## H4 잠재 위험 (T1.2 후속 — catalogue-only, 현재 active bug 아님)
- **position_controller.yaml non-wildcard 키**: `config/bluerov2/position_controller.yaml:4` 키 `position_controller_4dof:`(non-wildcard)는 노드 생성자명 `pid_4dof_controller`(position_controller_node.py:55)와도, 파일명 stem과도 불일치. 현재 launch 참조 0건(catalogue-only)이라 active bug는 아니나, 미래에 launch로 연결되면 T1.2/H4와 동일한 silent 파라미터 fallback 발생. launch 연결 시 키를 `/**:`로 정합 필요(repo 컨벤션). T1.2에서 hybrid만 고친 이유: hybrid가 유일 LIVE 경로의 active bug, position은 catalogue-only라 scope 분리.
- **노드명 충돌 `pid_4dof_controller`**: position_controller_node.py:55와 velocity_controller_node.py:55가 동일 생성자명. velocity는 Phase 2 삭제 대상이라 자동 해소되나, position이 살아남으면 충돌 가능성 잔존(P4_FLAGS 기존 항목과 중복 — velocity 삭제 후 단독이 되므로 실질 해소).

## ④ 고도화 제안 (P4 진행 중 도출 — owner 채택 결정 필요, P4 미구현)
- **가속도 feedforward end-to-end wiring (T1.3 후속)**: T1.3에서 `PositionController`의 position 모드 feedforward 계약을 `M·a`(힘=질량×가속도)로 교정했다(이전 `M·velocity`=운동량은 차원 오류). 그러나 이 가속도를 *공급*하려면 데이터 흐름 wiring이 필요하다 — 현재 (a) `path_following_node.py`가 `msg.acceleration`을 채우지 않고 `msg.velocity`만 publish, (b) `hybrid_controller_node.py:106-113` cmd_callback이 `msg.acceleration`을 안 읽음. `TrajectoryPoint.msg`는 `geometry_msgs/Accel acceleration` 필드를 이미 보유하므로 메시지 계약 변경은 불필요. **이 wiring은 신규 동작(④고도화)이라 owner 채택 결정 필요** — 미구현 시 position 모드 feedforward는 0(architect 확인: LIVE 경로에서 원래도 position 진입 시 velocity=0이라 동작 변경 없음). 구현 시 RTX4070 런타임 sign-off로 궤적 추종 개선 확인.
- **0.1 feedforward_gain 재정당화 (T1.3 후속)**: `position_controller_node.py`의 `feedforward_gain=0.1`은 틀린 `M·velocity` 항을 누르려던 fudge factor였다. `M·a`로 교정된 뒤엔 올바른 feedforward는 gain `1.0`(전체 모델) 또는 의도적 calibration knob(예: `unified_controller.yaml:62`의 `0.8`)을 써야 한다. **0.1을 hybrid 노드로 전파 금지**(워크어라운드 cargo-cult). 가속도 wiring 채택 시 함께 재정당화.
- **accel_ff 명시 rename (Option B)**: 현재 `PositionController.compute_control`은 `vel_ff`(velocity 모드 setpoint) + `accel_ff`(position 모드 feedforward) 두 인자를 받는다. 더 깔끔한 설계는 mode별 인자를 완전 분리하는 것이나, blast radius(HybridController 위임 + 호출자) 때문에 P4에선 최소 변경(accel_ff 추가)만 했다. 향후 리팩토링 후보.

## T4.4 측정 결론 (sim P4 실행 중 — 가정 반증, owner 범위 결정 대기)
P4_FLAGS의 즉시-수정 가정을 측정이 대거 반증함. 4건 중 teleop만 즉시 삭제 확정, 나머지는 latent/의도/설계 사안.
- **teleop_manager = 즉시 삭제 확정(owner O8)**: `stonefish_control/stonefish_teleop_manager/`는 README.md 1개뿐(package.xml/setup.py 부재 → ament 미인식). README가 "not yet implemented placeholder"로 명시, 코드/launch 참조 0건. = dead stub. 부활은 README 로드맵대로 별도 기능 작업.
- **Waypoint __hash__ 부재 = latent**: `common/waypoint.py:68` `__eq__` 정의 + `__hash__` 부재 → Python이 `__hash__=None` 설정 → unhashable(set/dict 키 불가). 단 측정 결과 Waypoint를 set/dict 키로 쓰는 LIVE 사용처 **0건**. 현재 미발현. 수정(=`__hash__` 추가 또는 `__eq__` 재검토)은 신규 동작이라 owner 결정.
- **gravity 9.82 = 의도적 calibration(버그 아님 가능성 높음)**: `dynamics_loader.py:123` default 9.82 + `dynamics_params.yaml` 전체(부력식 197.7N 역산 포함)가 9.82 일관. 표준 9.80665/9.81과 다르나, 실제 부력 평형을 9.82로 맞춘 **하드웨어 보정값** 성격(CLAUDE.md "하드웨어는 미니멀 모델이 못 보는 보정 노브" 원칙). 임의로 9.81로 바꾸면 부력 평형 깨짐. 표준화하려면 부력식 동시 재계산 필요 — owner 결정.
- **data/ 경로 = C++ 런타임 --data 베이스 의존(설계 사안, O6/O7)**: `.scn`이 `data/robots/.../meshes/*.obj`를 **작업디렉토리 상대 경로**로 참조(package:// 아님). scenarios/(7개 진입 .scn) ↔ data/(로봇 부품 .scn+mesh) 분리, CMakeLists가 둘 다 install. Stonefish C++가 `--data` 베이스 경로로 해석하므로 ROS package 경로화하려면 .scn 다수 + C++ 로더 + 런타임(RTX4070) 검증 동시 필요. 단순 수정 아닌 설계 결정.

## P4 후보 (P2 발견 — T4.4에서 정밀 재측정)
- `bezier_curve.py::BezierCurve.__init__`: tangents를 list로 받으면 `tangents[0]+tangents[1]`이 list concat(길이 6)이 되어 order=3/4 경로에서 np.dot shape 오류. assert는 len==3 list를 허용하나 내부 연산은 np.array만 정상. 수정안: 생성자에서 tangents/pnts를 np.asarray로 정규화. (동작 변경이라 P4에서 처리)
  - **T4.4 재측정**: L63-69 for loop이 tangents 원소를 검증만 하고 변환 안 함(np.asarray 미적용). LIVE 호출처 `bezier_curve.py:214`(`generate_cubic_curve`, cs_interpolator:80에서 LIVE)가 `[tangents[i], tangents[i+1]]` 전달. tangents 원소가 np.array면 정상(element-wise), list면 concat 버그. test_bezier_curve.py 통과 중이라 현재 numpy 경로로 동작 추정 → **LIVE 트리거 가능성은 caller가 list를 넘기는지에 달린 latent**. np.asarray 정규화는 방어적으로 옳으나 동작 변경이라 owner 결정. 실제 LIVE 트리거 여부 정밀 확인(generate_cubic_curve 내부 tangents 생성 타입) 선행 권장.

## P4 후보 (P3.0 컨벤션 조사 발견)
- **노드명 중복**: `controllers/velocity_controller_node.py:55`와 `nodes/position_controller_node.py:55`가 둘 다 `super().__init__('pid_4dof_controller')`. 동시 실행 시 ROS2 고유 노드명 요구(RMW 강제)를 위반해 노드 등록 충돌. 근거: [rmw validate_node_name.c](https://github.com/ros2/rmw/blob/master/rmw/src/validate_node_name.c). 수정안: 각자 고유 이름(예: `velocity_controller`·`position_controller`)으로 초기화. 동작 변경(노드명 의존 토픽 네임스페이스 영향 가능)이라 P4에서 처리.

## P4 후보 (P3 실행 중 강등)
- **velocity_controller_node dead 파일**: `controllers/velocity_controller_node.py`는 존재하지 않는 `from stonefish_control.controllers.pid_4dof import PID4DOF`(L36)에 의존해 `ros2 run` 시 즉시 ImportError. P3에서 **console_scripts 엔트리만 삭제**(기동 불가 노드라 배포 토픽 그래프에 부재 → 동작보존). 파일 자체와 복구(=`pid_4dof.py` 신규 작성 또는 노드 재설계)는 신규 동작이라 P4. dead 상태는 `test/test_characterization_node_entries.py::test_g2_velocity_node_is_dead`가 동결(pid_4dof 부활 시 RED).
- **unified_controller_node orphan**: `controllers/unified_controller_node.py`는 setup.py console_scripts에 미등록(dead *entry* 아닌 dead *file*). 상대 import 사용(§2.2 준수). P3에서 보존(삭제=잠재 의도 코드 손실 위험, 등록=미동작 노드를 동작시킴=신규 동작). 운명(삭제/등록/이동) 결정은 P4.
- **velocity dead 파일 §2.2 import 미교정**: `controllers/velocity_controller_node.py`는 intra-package 절대 import(`from stonefish_control.controllers.pid_4dof import`, `from stonefish_control.control_interfaces import`)를 유지 — §2.2 상대 통일 대상이나 dead 파일이라 P3 T3b 변환에서 제외(working 노드 hybrid/position만 교정). P4 복구(또는 삭제) 시 함께 정리. dead 파일을 nodes/로 옮기지도 않음(기동 불가 노드는 정렬 무의미).
- **trajectory nodes/__init__.py eager import**: `stonefish_trajectory_manager/.../nodes/__init__.py:1-2`가 `from .path_generator_node import main` 등으로 노드를 eager import → 이 패키지를 import하면 import-time에 rclpy가 끌려옴(§2.2 모델 패키지인데 자기 규칙 위반, 동작보존 중립 아님). control의 `nodes/__init__.py`는 inert(comment-only)로 올바름. eager import 제거는 import-time 동작 변경이라 P4. 상태는 `test_g4_trajectory_nodes_init_eager_is_known_p4`가 동결(P3에서 고치면 RED).
- **소스 헤더 라이선스 ↔ 패키지 메타데이터 불일치**: P3 T1에서 3개 setup.py를 package.xml SSOT에 맞춰 `GPL-3.0`으로 정렬했으나, **소스 파일 헤더는 45개가 Apache-2.0**(`Licensed under the Apache License, Version 2.0`), GPL 헤더는 3개뿐. 즉 패키지는 GPL 선언이나 다수 소스가 Apache 헤더 → 법적 불일치. 헤더 일괄 정정은 (a) 대규모(45파일), (b) 어느 쪽이 진짜 의도인지 owner/법적 판단 필요 → P4. (메타데이터는 package.xml·CLAUDE.md가 일관되게 GPL-3.0이라 그쪽으로 정렬함.)

## P4 후보 (P3 T6 재감사 — 거대 모듈/메서드, 변경 없음)
P3에서 **코드 변경 없이 목록화만**. 전부 상태 적분기·수치 계산·토픽 타이밍을 품은 분해라 동작보존 밖(P4).
- **ilos_guidance.py (1068줄)**: `ILOSGuidance.compute_guidance`가 **319줄 god-method**(L632-951). curvature estimator **4종**(`_estimate_curvature` L446, `_estimate_signed_curvature` L498, `_estimate_curvature_3d_frenet` L547, `_estimate_max_curvature_preview` L598)이 서로 다른 공식. ILOS는 integral term(상태 적분기) 보유 → 분해 시 수치 변경 위험. curvature 4종 통합도 공식이 달라 수치 변경. P4.
- **los_guidance.py (569줄)**: `LOSGuidance.update`가 **177줄 god-method**(L153-330). cross-track/curvature/tangent 계산 분산. `_normalize_angle`(L500, arctan2)은 modulo와 경계 2π 발산이라 통합 금지(이미 P3 격리). LOS 상태 추적 포함 분해 = 수치 변경. P4.
- **lipb_interpolator.py (513줄)**: `init_interpolator`가 **152줄 god-method**(L61-213). 보간 알고리즘(LIPB) 분해 = 수치 변경. P4.
- **path_following_node.py (526줄)**: `__init__` 170줄(L48-218) + `_guidance_update_callback` 146줄(L335-481). Node 콜백/구독 분해 = 토픽 그래프·타이밍 동작 변경. P4.
- **공통**: 이 4개 모듈은 "전면 재구조화"가 가장 필요한 god-class/god-method 집합이나, 전부 알고리즘 상태·수치·타이밍을 품어 P3 동작보존 철칙 밖. P4에서 characterization(수치 골든 테스트) 선작성 후 분해.

## P4 sign-off 의무 (P3 변경의 런타임 검증 — runnable ROS2 필요)
P3 안전망은 정적·국소 검증이라 런타임 rclpy registry 의미를 못 덮는다. 아래는 `colcon build`+`ros2 launch` 환경에서 확인할 것.
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
- **outer Kp=[0.4,0.4,0.3,0.8] / v_sp_limit=[0.5,0.3,0.25,0.6]**: 시간상수 분리 원칙(outer가 inner보다 느림)에서 도출한 초기값. 닫힌루프 정착시간·오버슈트는 컨테이너 미검증이라 RTX4070 실기 측정으로 미세조정. v_sp_limit는 OWNER DECISION #1(c) ALIGN 보수값.
- **모드 전환 첫 tick 점프**: velocity→cascade 진입 시 `set_mode`의 reset + outer 출력 clamp(v_sp_limit)로 1차 완화. integral preloading(전환 시 적분기를 직전 속도로 시드)은 실기 관측 후 검토. 또한 cascade_controller 미생성 환경에서 `set_mode('cascade')` 시 `active_mode='cascade'`로 보고하나 실제 라우팅은 position으로 폴백하는 latent footgun(Task 4 리뷰 지적) — 가드 추가는 P4.
- **코너 추종 (sway=0 + 고정 lookahead 3m)**: cross-track sway 채널 제거로 코너에서 cascade outer만 횡오차를 닫는다. adaptive lookahead 재활성·curvature preview는 미구현(현재 `adaptive_lookahead: false`). 코너 추종 정확도는 실기 sign-off 항목.
- **닫힌루프 안정성·정착시간·thruster allocation 포화**: 단위테스트(84 passed)는 순수 산술(CascadeController 손계산·ILOS 축소 골든)과 정적 게이트만 덮는다. 닫힌루프 안정성·정착시간·thruster 포화는 컨테이너 골든 미검증 → `colcon build` + `ros2 launch` + RTX4070 실기 sign-off 필요. 통과 전까지 cascade 모드는 검증 미완 상태로 간주.
