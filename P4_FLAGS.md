## P4 후보 (P2 발견)
- `bezier_curve.py::BezierCurve.__init__`: tangents를 list로 받으면 `tangents[0]+tangents[1]`이 list concat(길이 6)이 되어 order=3/4 경로에서 np.dot shape 오류. assert는 len==3 list를 허용하나 내부 연산은 np.array만 정상. 수정안: 생성자에서 tangents/pnts를 np.asarray로 정규화. (동작 변경이라 P4에서 처리)

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
