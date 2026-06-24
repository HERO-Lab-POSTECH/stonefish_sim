# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

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
