# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

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
