# Task 1 Findings — ALBC Stonefish Port Discovery

Scope: `ksm-ubuntu` only (`stonefish_dev` + `marinelab-isaaclab` containers). No access to
`agent-jetson` (the real board) in this task.

## (a) npforward.py public API

File: `albc_bridge/albc_bridge/policy/npforward.py` (staged from pack_B, sha256
`8eff0046cf1665ee15955fd8fa52a5a2d96d9a7003f8beff2b9382c57a008741`, byte-identical to
marinelab source and to `MANIFEST.json`'s recorded hash).

Public functions/classes (`grep -nE '^(def|class) '`):
```
15:def linear(x, w, b)
20:def elu(x, alpha=1.0)
25:def softsign(x)
30:def layer_norm(x, gamma, beta, eps=1e-5)
42:def conv1d(x, w, b, stride=1)
62:def gru_cell(x_t, h, w_ih, w_hh, b_ih, b_hh)
85:def _sigmoid(x)
90:class TeacherActor      # normalizer + actor MLP
124:class StudentTCN        # channel_transform -> 3x(Conv1d+ELU) -> head -> softsign
153:class StudentGRU        # not used by pack_B (no weights_gru.npz shipped)
```

Deployed entrypoints (pack_B ships `weights_teacher.npz` + `weights_tcn.npz`, no GRU
weights — the TCN student variant is the one to call):

- `StudentTCN(weights_tcn_npz).forward(win)`
  - input `win`: `(batch, hist_len=9, obs_dim=69)` float32 — a sliding window of the last
    9 raw (unnormalized) 69D observations.
  - output: `latent (batch, 9)` float32, passed through `softsign` (range (-1,1)).
  - confirmed by `golden_tcn.npz`: `input_window (1,9,69) -> ... -> forward_out (1,9)`.

- `TeacherActor(weights_teacher_npz)`:
  - `.normalize(obs)`: `obs (batch,69) -> obs_normalized (batch,69)`, formula
    `(x - mean) / (std + eps)`, `eps=0.01` (rsl_rl `EmpiricalNormalization` parity, note
    in npforward.py docstring: `_std` is NOT eps-folded, must add eps at call time).
  - `.act(obs_normalized, latent)`: concatenates to `actor_input (batch,78)` then runs a
    4-layer MLP (256→128→64→8, ELU between layers, none after last) →
    `action (batch,8)` float32. **No final activation/squash** on the action output.
  - confirmed by `golden_teacher.npz`: `obs(1,69) -> obs_normalized(1,69)`,
    `latent(1,9)` (from StudentTCN) `-> actor_input(1,78) -> action(1,8)`.

Full call sequence for one 50Hz control step:
```
win        = ring buffer of last 9 raw obs, shape (1, 9, 69)
latent     = StudentTCN(weights_tcn).forward(win)            # (1, 9)
obs        = current raw obs, shape (1, 69)
obs_norm   = TeacherActor(weights_teacher).normalize(obs)    # (1, 69)
action     = TeacherActor(weights_teacher).act(obs_norm, latent)  # (1, 8)
```

Weight npz key shapes (from `pack_B/MANIFEST.json`, byte-verified against the staged
copies): `normalizer._mean/._std (1,69)`, `actor.{0,2,4,6}.{weight,bias}` MLP
256/128/64/8, `channel_transform.0.* (32,69)/(32,)`, `conv.{0,2,4}.*` (64,32,3)→
(128,64,3)→(128,128,3), `head.0.* (128,384)/(128,)`, `head.2.*` LayerNorm gamma/beta
(128,), `head.3.* (9,128)/(9,)`.

py2.7-compat constraint (board runtime, `marinelab: tests/deploy/test_npforward_compat.py`):
AST-gated — no `@` matmul, no f-strings, no annotations, no kwonly args. npforward.py as
staged already satisfies this (verified by that test's presence upstream; not re-run here
since this task's container is py3-only, but the file content is unchanged from the
gated source).

## (b) np_policy obs-assembly reuse verdict: **NOT FOUND / re-implementation required**

Searched exhaustively for an existing deploy-side ROS node (`np_policy`, `build_proprio`,
`numpy_port`) on both accessible hosts:
- `marinelab-isaaclab`: `find / -iname '*np_policy*' -o -iname '*build_proprio*'` → no
  hits anywhere in the container filesystem.
- `ksm-ubuntu` host: same `find` → no hits.

**Conclusion: no real-robot ROS deploy node is reachable from this task's SSH scope.**
Per vault memory (`project_albc_26d_sensor_mapping.md`), that reimplementation exists on
`agent-jetson` (a different machine, out of scope for Task 1's global constraints, which
restrict all commands to `ksm-ubuntu`). Task 6 cannot reuse board code directly from here;
it can only use the spec below as reference and must re-derive the obs assembly for
Stonefish/ROS2.

What IS available on `marinelab-isaaclab` (the `grep -rlE
'integral_leak|manipulability|hist_len|thruster.*filter|root_ang_vel'` hits):
- `constrained_albc/envs/main/mdp/observations.py` — `compute_policy_obs(env, robot)` —
  **simulation-side, torch + Isaac Lab `Articulation` API**. Tightly coupled to
  `env._euler_cache`, `env._thruster.state`, `env._manipulability`, `robot.data.joint_pos`
  — internal simulator state, GPU batch tensors. **Not directly callable outside Isaac
  Lab; not reusable as-is for ROS I/O.**
- `constrained_albc/envs/main/config.py:398` — `integral_leak: float = 0.99` (leaky
  integrator: `I_{t+1} = leak*I_t + err*dt`).
- `constrained_albc/envs/main/config.py:459-462` — `hist_len: int = 3`,
  `hist_stride: int = 3` (ring buffer recorded every 3rd control step, 3 stored steps).
- `observations.py:9-33` (module docstring) — the authoritative 69D layout spec:
  - Command (3D): `ang_cmd[roll_att, pitch_att, yaw_rate]`
  - Body State (6D): `euler(3)`, `root_ang_vel_b(3)` — no linear velocity (no DVL)
  - Arm State (5D): `joint_pos(2)`, `joint_vel(2)`, `manipulability(1)` (Yoshikawa index)
  - Thruster (6D): filtered ESC output m0-m5
  - Temporal history (46D): `(q_des_prev - q_actual, joint_vel) x3` (12D) +
    `(ang_err[att_rp(2)+yaw_rate(1)], rpy(3)) x3` (18D) + `full_action(8D) x2` (16D)
  - Integral error (3D): leaky-integrated [roll, pitch, yaw_rate]

**Verdict: ENTANGLED with the simulator, not reusable.** Task 6 must reimplement obs
assembly in pure numpy/ROS2 from this spec (mirroring how `npforward.py` mirrors the
torch model), the same pattern already used for the model forward pass. This spec's
numeric constants (`integral_leak=0.99`, `hist_len=3`, `hist_stride=3`) match vault memory
(`project_albc_26d_sensor_mapping.md`), corroborating this is the correct reference.
Note: this 46D "history ring buffer" (baked into the single-step 69D obs vector) is
distinct from the TCN's separate 9-step `(1,9,69)` window (finding a) — two different
temporal mechanisms, both required.

## (c) Stonefish revolute continuous-rotation verdict: **YES, unlimited by default**

`/opt/stonefish/include/Stonefish/joints/RevoluteJoint.h`:
```
100:        //! A method to set the limits of the joint.
104:        void setLimits(Scalar min, Scalar max);
```
`setLimits(min, max)` is the **only** API for constraining rotation range, and it is an
explicit opt-in method call — neither constructor (`RevoluteJoint(name, solidA, solidB,
pivot, axis, ...)` nor the world-attached overload) takes limit arguments, and no default
limit is set internally (no `lower`/`upper`/`continuous` members in the class, only
`axisInA`, `pivotInA`, damping/IC fields — confirmed by the full header dump, no other
`limit`-related state exists).

**Conclusion: a `RevoluteJoint` with `setLimits()` never called is unconstrained /
continuous by construction** — suitable for representing cable-winding (unlimited
rotation) joints. `ScenarioParser.cpp` source (XML→C++ parsing logic) is not present in
this container (only compiled headers/lib — `find / -iname ScenarioParser.cpp` → no
hits), so the exact `.scn` XML attribute name that triggers `setLimits()` was not
confirmed textually; Task 3/8 should simply **omit any limit-related joint attribute in
the `.scn`** to get unlimited rotation, and can verify empirically (spin the joint past
2π in a smoke test) rather than relying on XML schema docs not available in this
container. No existing `.scn` in `stonefish_sim/stonefish_description/` uses
`type="revolute"` as a precedent (grep returned no hits), so there is no in-repo example
to copy from.

## Repo-editing decision (step 8)

`albc_bridge` was created at `/workspace/src/albc_bridge` (sibling of
`/workspace/src/stonefish_sim`, per the brief's step 2), but the git repo root is
`/workspace/src/stonefish_sim` (`.git` lives there) — so a sibling dir is **outside** that
repo's tree and invisible to `git add -A` run from within it.

`stonefish_sim` already contains multiple ROS packages as subdirectories of the *same*
git repo: `stonefish_control/`, `stonefish_description/`, `stonefish_msgs/`,
`stonefish_ros2/`. colcon recursively scans `/workspace/src/` for `package.xml`/`setup.py`
regardless of git-repo boundaries, so nesting a new package inside an existing repo's
directory is exactly the pattern already in use here and needs no symlink or second git
repo.

**Decision: moved `albc_bridge` into `/workspace/src/stonefish_sim/albc_bridge`** (simplest
option — matches existing convention, no symlinks, no second `git init`, colcon build
unaffected since it scans recursively). `.gitignore` in `stonefish_sim` does not exclude
`.npz` files, so all 5 policy assets are tracked normally (not LFS, not gitignored) —
flagged as a concern below.

## Concerns for later tasks

- Policy weight/golden `.npz` binaries (~800KB total) are committed directly into git
  history (no LFS). Fine for now given the modest size, but worth noting if more/larger
  checkpoints get added later.
- `agent-jetson`'s existing `np_policy`/`build_proprio` deploy code (per vault memory) was
  not inspected in this task — it's on a different machine outside this task's SSH scope.
  If Task 6 wants to reuse logic (not code) from it, a human with agent-jetson access
  would need to pull it in; this task could not verify it exists in a copy-pasteable form.
- Stonefish's exact `.scn` XML attribute for joint limits could not be confirmed from
  source (ScenarioParser.cpp not present in the container) — Task 3/8 should verify via a
  runtime smoke test (spin past 2π), not assume a specific XML tag name untested.
