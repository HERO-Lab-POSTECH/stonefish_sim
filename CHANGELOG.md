# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added

- **P2 — 실측 모델 주입 (system identification)**: open-loop 스텝 프로브
  (sim+allocator만, body wrench 직접 주입)로 플랜트 실측 —
  유효질량 M_eff [70.2, 62.0, 63.9] kg(부가질량이 건질량의 3.1~3.5배)·
  yaw I_zz≈0.24(order만 신뢰)·감쇠 d1/d2·heave 잔류부력 7.27 N·
  v_max(55 N)=0.911 m/s. 산출을 `dynamics_params.yaml`에 기입
  (`added_mass_diag` 등 flat diag — rcl 파라미터 타입 제약).
  `CascadeController`에 accel feedforward 추가: clip 후 v_sp를 내부
  수치미분(1차 LPF)해 `M_eff·v̇_sp`를 inner에 합산(포화 전) —
  guidance 원신호 미분은 outer surge 상시 clip 때문에 unmatched
  disturbance가 되어 리뷰에서 기각. **폐루프 어블레이션(runE vs runF,
  단일 스택 검증)에서 accel ff ON이 직선 leg 사행·allocator 포화를
  유발함이 실측되어 기본 비활성**(`accel_ff_cutoff_hz: 0.0` — v_sp가
  outer P 경유로 차량 위치의 함수라 그 미분이 폐루프 자기되먹임이 됨;
  ON: e_y RMS 0.456·포화 경고 1459건 → OFF: 0.227·0건).
  damping·부력 ff는 순기능으로 유지(e_y RMS 0.227 m vs P1 0.323 m).
  게이트 테스트: 실측치↔게인 정합
  (`Kp=M_eff·ω_c`)·node 기본값 drift·v_sp_limit≤실측 v_max

- **`stonefish_sonar_yolo` 패키지** (김민종 colcon_ws2 통합, 원명 `sonar_yolo_ros2`):
  FLS 소나 이미지 YOLO 추론 노드 (`sonar_yolo/detections` JSON + `sonar_yolo/annotated`).
  `stonefish_` 접두사 규칙에 맞춰 albc 선례대로 개명(디렉토리·모듈·resource 마커·
  entry point 일괄). ROS 노드/실행자명은 `sonar_yolo_node` 유지.
  pip 전용 `ultralytics` 필요, 가중치(`stonefish_yolo_sofa.pt`)는 repo 미포함 — README 참조
- **`nav_interfaces` 패키지** (김민종 통합): 실해역 bag 디코드용 항법 메시지
  (`LIGnav`/`NavAtt`/`NavVel`). bag에 기록된 타입명 호환 때문에 `stonefish_` 접두사
  개명 불가 — 워크스페이스 `.omp` 명명 규칙에 예외로 등록(stonefish_ws PR 참조)
- README Docker 설치 안내 — 전용 배포 repo
  [`stonefish_bringup`](https://github.com/HERO-Lab-POSTECH/stonefish_bringup) 참조
  (core+sim+slam 소스를 bake하는 멀티스테이지 GPU 이미지, 호스트 실빌드 검증 완료)
- 협업 규칙 `CONTRIBUTING.md` + PR 템플릿 (GitHub Flow · Conventional Commits, 발효 2026-07-23)

### Changed

- **P2 — cascade 게인 실측 재산정·속도 상한 실측 정합**: inner
  Kp=M_eff·ω_c=[70,62,64], Ki=Kp/2 (yaw는 I_zz 실측 불확실로
  P1 검증값 유지, ω_c=2 rad/s — 단일 스택 폐루프 runF에서 무포화·청정
  실증). `v_sp_limit[surge]` 1.2→0.7, `cruise_speed` 1.0→0.7 —
  실측 v_max 0.911 m/s에 더해, 0.8 이상에서는 allocator 균등 스케일링의
  yaw 권한이 3 N·m 이하로 붕괴(실측 항력 기반 권한 표는 config 주석).
  0.7은 단일 스택 폐루프(runF)에서 무포화·청정 실증.
  실기 요구 속도는 Open Q1(사용자 결정)

- BlueROV2 FLS 장착 pitch 60°→80° 하향 (`bluerov2.scn`, 김민종 통합) — 측량 고도에서
  해저면이 소나 fan 안에 들어오도록 조정, SLAM 특징 추출이 지형 리턴을 보게 함
- **BREAKING** `albc_bridge` → `stonefish_albc_bridge` 개명 — 유일하게 `stonefish_` 접두사
  규칙(`.omp/rules.json` naming `src/*/`)을 어기던 패키지. 디렉토리·`package.xml`·`setup.py`·
  `setup.cfg`·ament resource 마커·import 경로·launch 참조 일괄 변경.
  실행 명령이 바뀝니다: `ros2 launch stonefish_albc_bridge albc_smoke.launch.py`.
  ROS 노드 이름은 형제 패키지 관행(기능명)에 따라 `albc_bridge`로 유지
- 테스트 게이트에 `albc_bridge` 편입 — `pytest.ini` testpaths 추가 + `ros2 pkg create`
  잔재 lint 테스트 3개 제거(형제 패키지 7개가 이미 삭제한 관행과 동일).
  `python3 -m pytest` 115 → 136 passed
- README 설치 가이드: 실존하지 않는 `/workspace/colcon_ws` 경로 제거, `--merge-install` 표준화,
  패키지 표에 albc_bridge 보완
- **동작 불변 정리** — `stonefish_control`/`stonefish_thruster_manager`/`stonefish_trajectory_manager`의
  호출자 0 dead code 삭제(미사용 dataclass 10종·회전 헬퍼 2종·`_log` 모듈·`update_gains`·
  `get_summary`·`to_4dof_message`·`waypoint_set.py` 이식 API 7종, 각각 grep으로 재확증)와
  죽은 launch 인터페이스 제거(`thruster_manager.launch.py`의 무효 `output_topic`
  arg+리맵, `path.launch.py`의 no-op `namespace` 인자, `thruster_allocator_node.py`의
  미구독 `wrench_callback`)
- `stonefish_albc_bridge` 버전 0.4.0 → 0.5.0 (형제 8개 `package.xml`과 동일하게)
- 문서 drift 정리 — docs 사이트·`CONVENTIONS.md`·`P4_FLAGS.md`·`stonefish_control` README를
  0.4.0 동결 상태(버전·패키지 수·pytest 통과 수)에서 실측 현재값(0.5.0·8개 패키지·139
  passed)으로 갱신, 잔여 `--symlink-install`/`/workspace/colcon_ws` 서술 제거

### Fixed

- **추진기 힘→PWM 제곱 왜곡 수정 (경로추종 실패 근본 원인)**: allocator가
  힘[N]을 `max_thrust`로 선형 나눗셈해 발행한 setpoint를 Stonefish가 rpm 분율로
  해석(추력 ∝ n|n|)해 실제 추력이 명령의 제곱으로 붕괴했다(스케일 100 기준
  100 N 명령 → 실추력 7.3 N). `force_to_pwm()` √ 역추력맵으로 교체하고
  `max_thrust`를 물리 최대 추력 20.68 N(.scn 사양·environment.scn ρ=1031
  유도)으로 재정의. `max_thrust ≤ 0` 입력 검증 추가(NaN setpoint 차단)
- **컨트롤러 포화 한계 물리 정합**: 전 모드(hybrid 3모드 + position 단독)
  `max_force` 800→55 N, `max_torque` 160→13.7 N·m — TAM×T_max **단일축**
  상한(surge/sway 58.5 N, yaw 13.77 N·m) 이내. 종전 ~14배 초과 설정으로
  anti-windup이 미발동해 코너 적분 폭주 유발. 물리 정합 게이트 테스트
  (`test_thrust_saturation_consistency.py`: config 전체 glob + allocator
  기본값 대조) 추가
- **allocator 방향보존 균등 스케일링**: 다축 동시명령(코너 surge+yaw)이
  추진기당 한계를 넘을 때 종전 element-wise 클립은 wrench 방향을 왜곡
  (yaw 붕괴)했다 — `scale_thrust_to_limit()`로 방향 보존, 발동 시
  throttled warning 노출
- **cascade `v_sp_limit[surge]` 0.5→1.2**: guidance `cruise_speed` 1.0과의
  모순(상시 windup 압력) 해소. (P2에서 실측 v_max 0.911 m/s로 재반증되어
  1.2→0.7 재조정 — 아래 P2 항목)
- **게인 물리 기반 재산정**: 종전 게인(Kp 200~400)은 제곱 왜곡 플랜트 위에서
  튜닝된 값이라 승계 불가 — inner/velocity Kp≈m·ω_c(≈40), position Kp≈m·ω_n²
  초기값으로 교체(P1 닫힌루프 튜닝 대상). (cascade inner는 P2 실측 유효질량
  으로 재산정되어 140/124/128 — 아래 P2 항목; velocity/position 모드는 유지) declare_parameter 기본값·컨트롤러
  시그니처 기본값도 YAML과 동기화(silent-fallback 시 구 플랜트 부활 차단).
  README·docs/site의 구 의미 서술("PWM 정규화 척도, 물리 한계 아님")도
  물리 한계 정의로 전면 갱신
- `albc_smoke.launch.py`: `/workspace` 절대경로 하드코딩 → `FindPackageShare` (타 머신 이식성)
- `albc_bridge` 메타데이터 placeholder 제거 (버전 0.4.0 통일, GPL-3.0, 실제 description)
- `obs_builder.py`·`bridge_node.py` docstring·`test_obs_builder.py` 라벨의 "69D" 서술을
  실제 72D 계약(bias-EMA 3ch 추가)으로 정정

## [0.5.0] - 2026-06-25

**P5 — 경로추종 position-cascade 재설계 (의도적 동작 변경).** 코너 추종
trajectory 결함의 근인이던 cross-track **이중보정**(ILOS heading arctan 항 +
비표준 sway PID)을 제거하고, 별도 `CascadeController`(outer position-P → inner
velocity-PI)가 cross-track을 단일 채널로 닫도록 제어 구조를 재설계했다. 차량이
fully-actuated이므로 underactuated ILOS 대신 진짜 position-cascade가 성립한다.
depth 채널(`_integral_ez`)은 ILOS에 그대로 유지된다. 모든 변경은 executor가
구현하고 별도 code-reviewer / verifier가 독립 검증했으며(전건 APPROVE, 0 blocker),
verifier는 골든값을 기하·삼각·제어이론으로 0부터 재유도해 1e-9 이내 일치를 확인했다.
닫힌루프 안정성·정착시간·thruster 포화는 컨테이너 미검증으로 RTX4070 실기
sign-off에 이월된다(`P4_FLAGS.md` §P5 cascade).

### Added
- **`CascadeController`** (`stonefish_control/controllers/cascade_controller.py`):
  outer position-P가 body-frame 위치오차(`R.T @ e_pos_world`)로 속도 setpoint를
  내고, inner velocity-PI가 wrench를 낸다. anti-windup(back-calculation)·사다리꼴
  적분·integral-limit은 검증된 `position_controller.py`(F1) 기제를 충실히 포팅.
  `Kp_outer is None`이면 미생성(optional)이라 기존 velocity/position 경로는
  무변경. 단위테스트 `test_cascade_controller.py` (B1~B8 골든).
- **cascade 모드 라우팅**: `HybridController.set_mode`/`hybrid_controller_node`
  mode_callback 화이트리스트에 `'cascade'` 추가. `bluerov2/hybrid_controller.yaml`에
  `cascade.*` 파라미터 블록(outer/inner Kp·Ki·Kd·Kb, v_sp_limit, max_force/torque).
- **AST 정적 게이트** (`test/test_cascade_static_gate.py`): rclpy 미설치 환경에서
  적분기 갱신 부활·모드 문자열 회귀를 AST/소스로 고정(mutation 검증으로 진위 확인).

### Changed
- **ILOS 가이던스 축소**: FOLLOW 모드에서 `χ_d = χ_p`(path tangent만) — cross-track
  heading arctan 항과 sway PID(`v_lateral=0.0`)를 제거. cross-track 보정은 cascade
  outer가 전담(이중보정 제거). `e_y`는 진단용으로 계속 계산·로깅되나 heading에 미반영.
- **publisher 모드 문자열** `'hybrid'` → `'cascade'`: subscriber가 드롭하던
  `'hybrid'`를 화이트리스트와 정합하는 `'cascade'`로 정정(`path_following_node`).
- **버전** 0.4.0 → 0.5.0 (7 package.xml + 3 setup.py).

### Removed
- ILOS heading 적분(`_integral_ey`)·sway 적분(`_integral_ey_lateral`) 갱신 로직과
  dead 로컬 `curvature_ff`(χ_d=χ_p 축소로 미소비). `_integral_ez`(depth)는 유지.

### Documented
- `path_following.yaml`의 `lateral_gain`·`integral_gain`·`max_lateral_velocity`에
  `[deprecated §4]` 주석(YAML 로드 호환 위해 키만 유지, 값 무변경).
- 축소 전 ILOS 공식을 모듈·클래스 docstring에 `[deprecated §4]` 이력으로 격하.
- `P4_FLAGS.md` §P5: cascade 이월 5항목(inner M·a feedforward, outer Kp/v_sp_limit
  실기 튜닝, 모드전환 첫 tick 점프+None-cascade footgun, 코너 lookahead, 닫힌루프
  안정성 RTX4070 sign-off).

### Verification
- 102 passed (정식 `pytest.ini` 범위: `stonefish_control` + `test`). cascade 단위
  B1~B8·ILOS 축소 characterization·AST 게이트 7건 포함, 0 실패.
- 독립 verifier: S3 yaw/sway·B1/B5/B6/B8·S6 heave 골든을 외부 기준으로 재유도,
  1e-9 이내 일치. 이중보정 제거를 B2(vel_ff[1]=0)+B4(sway=Kp·e_y)+S3(CTE+1인데
  sway=0) 세 테스트로 다각 회귀 고정.
- AST 게이트 mutation 검증: `_integral_ey_lateral +=` 회귀 주입 시 게이트 FAIL 확인.

### Notes
- **RTX4070 실기 sign-off 미완**: 단위·정적 테스트는 순수 산술과 구조만 덮는다.
  닫힌루프 안정성·정착시간·thruster allocation 포화·코너 추종(sway=0 + 고정 3m
  lookahead) 정확도는 `colcon build` + `ros2 launch` + 실기 측정 전까지 미검증.
  통과 전 cascade 모드는 "검증 미완"으로 간주(`P4_FLAGS.md` §P5).
- 클래스명 `ILOSGuidance`는 호환을 위해 유지하나 잔존 적분 항은 depth 하나뿐이다.

## [0.4.0] - 2026-06-24

**P4 — algorithmic/numeric correctness + intentional behavior change.** The
behavior-preservation rule that held through P3 ends here: live controller bugs
were fixed, dead UUV-Simulator-ported layers removed, the package license and
versions unified, and dead service interfaces pruned. The verification standard
shifts from "same as before" to "correct as intended." Every change was made by
an executor and independently verified by a separate architect / code-reviewer /
verifier pass (all APPROVE, 0 blockers); owner decisions (O5 GPL, O6 single
version, O8 teleop, O12 C++ scope) gated the intentional changes. C++ changes are
static-only (no build toolchain in CI); runtime sign-off deferred to the RTX4070
environment, tracked in `P4_FLAGS.md`.

### Fixed
- **position_controller_node construction crash** (T1.1, CRITICAL): the node
  passed `integral_limit=` to a `PositionController.__init__` that has no such
  parameter -> immediate `TypeError` on `ros2 run`. A registered console_scripts
  node that could not start. Bound the auto-derivation parameter
  (`integral_safety_factor`) instead.
- **hybrid controller parameters not loaded** (T1.2 / H4): the YAML key was the
  node's class name, not a wildcard, so ROS2 silently fell back to defaults on
  the only LIVE control path. Switched the config key to `/**:` (repo
  convention).
- **position-mode feedforward dimensional error** (T1.3): the feedforward term
  computed `M·velocity` (momentum) where Newton's 2nd law requires `M·acceleration`
  (force). Corrected the contract and added an `accel_ff` argument. (End-to-end
  acceleration wiring is a separate ©enhancement deferred to `P4_FLAGS`; on the
  LIVE path feedforward was always 0, so no runtime behavior changed.)
- **closest-waypoint selection bug** (T1.4): `argmin` over a signed difference
  returned the earliest waypoint, not the nearest. Switched to `np.abs` so the
  nearest waypoint along the path is chosen.
- **C++ scenario-parser null-derefs** (T1.5): four unchecked XML-attribute
  derefs in `ROS2ScenarioParser` could segfault on a malformed `.scn`. Guarded
  with the existing `!= XML_SUCCESS || ptr == nullptr` idiom + `RCLCPP_ERROR`.
- **stonefish_msgs missing rosidl runtime export** (T4.3b): added the
  `ament_export_dependencies(rosidl_default_runtime)` that the sibling
  control_msgs already had, so downstream packages receive the transitive
  runtime dependency.

### Removed
- **Dead UUV-Simulator-ported control layers (~4000 lines)** (T2.2): the entire
  `control_interfaces` inheritance stack (Vehicle / DPControllerBase /
  DPPIDControllerBase, 0 subclasses), the unified/velocity controllers, dead
  `los_guidance`, orphan configs, and a dangling `VelocityProfiler` `__all__`
  entry -- all confirmed dead by a 3-axis liveness audit (`docs/LIVENESS_AUDIT.md`).
  The single LIVE control path is `hybrid_controller_node`.
- **5 dead service interfaces** (T4.3a): `Hold`, `ResetController`, `GetPIDParams`,
  `SetPIDParams` (control_msgs) and `SetMode` (stonefish_msgs) had zero
  create_service/create_client usage. Removed the srv files, their CMakeLists
  registrations, and all README docs advertising them. (`SetMode` was a grep-trap:
  the live `set_mode()` is a std_msgs/String topic callback, not the service.)
  LIVE srv preserved: `ResetTrajectory` + the 5 C++-served environment/sonar
  services.
- **teleop stub + phantom control_utils docs** (T4.4a): `stonefish_teleop_manager`
  was a README-only placeholder (no package.xml -> invisible to ament); the
  `stonefish_control` README also documented a `stonefish_control_utils` package
  that never existed. Both removed.

### Changed
- **Unified all package versions to 0.4.0** (T4.2, O6): seven packages had drifted
  to 1.3.0 / 1.0.0 / 0.3.0 with no release SSOT. Unified 13 declaration sites
  (7 package.xml, 3 setup.py, 3 `__init__.py __version__`) to a single monorepo
  baseline.
- **Relicensed 33 Python sources to GPL-3.0** (T4.1, O5): the package metadata
  declared GPL-3.0 but 33 .py files still carried Apache-2.0 headers. Converted to
  SPDX `GPL-3.0-or-later` headers (matching the C++ grant), preserving every
  original copyright holder (UUV Simulator Authors / Stonefish Contributors /
  maintainer).

### Documented
- **C++ concurrency + QoS premises** (T4.5, O12): annotated the SingleThreadedExecutor
  invariant (lock-free shared-state access depends on it), the map-key
  silent-overwrite hazard, and the default-QoS premise -- static only, no behavior
  change. Standard-alignment proposals (SensorDataQoS, duplicate-key warning,
  MultiThreadedExecutor) recorded in `P4_FLAGS.md` for RTX4070 sign-off.
- **stonefish_control README brought in line with as-built code** (release): removed
  docs for the deleted standalone velocity controller (a *mode* of the hybrid
  controller, never a separate node), corrected the position feedforward formula to
  `M·a_ff` (matching the T1.3 fix) in both the README and the `PositionController`
  docstring, fixed the mode-switch mechanism (`control_mode` String topic, not a
  `set_mode()` service), and synced topic/parameter tables. All doc-only -- no
  executable code changed. (The `position_mode.max_force/max_torque` table values
  were corrected to the node's declared defaults 200.0/50.0; note the
  bluerov2 `hybrid_controller.yaml` overrides these to 800.0/160.0 -- a
  config-vs-default divergence recorded in `P4_FLAGS.md`.)

### Added (pre-P4, released in 0.4.0)
- TF broadcasting to odometry publisher
- SetWaveHeight / SetWindVelocity / SetOceanCurrent ROS2 services for runtime
  environment control
- BlueBoat surface vehicle support
- Comprehensive README documentation

### Changed (pre-P4, released in 0.4.0)
- Replaced hardcoded absolute paths with package-relative paths
- Updated BlueROV2 scenario and configurations
- Legacy robot definitions moved to `_legacy/`
- Fixed hardcoded workspace paths in launch files
- Fixed path-following restart on trajectory re-reception

### Verification
- `env -i /usr/bin/python3 -m pytest -q`: **42 passed** (baseline was 36; +6 from
  P4 characterization/regression tests). Python correctness fully verified in CI.
- C++ changes (T1.5 null-guards, T4.3b export, T4.5 docs) are static-only -- this
  environment has no colcon/ament toolchain. Runtime build + simulator sign-off
  deferred to the RTX4070 environment (`P4_FLAGS.md`).

### Notes
- Deferred to a separate cycle (high-risk, ©-only): god-method decomposition
  (ilos/lipb/path_following), data/ path package://-ification, and the latent
  items measured but not changed (Waypoint `__hash__`, gravity 9.82 calibration,
  bezier tangent normalization) -- all recorded in `P4_FLAGS.md` with rationale.
