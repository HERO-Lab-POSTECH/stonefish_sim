# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Assemble the 72D policy observation from Stonefish sensors (pure numpy).

Reimplemented from the Isaac Lab / MarineGym training source (NOT reused from
the old deploy node -- Task 1 established that node is not reusable). The
student policy consumes exactly this 69D vector; a subtle layout/constant error
corrupts the input silently, so every field below is matched byte-for-byte to
the SSOT and cited with file:line.

SSOT (marinelab-isaaclab container /workspace/constrained-albc):
  * constrained_albc/envs/main/mdp/observations.py:compute_policy_obs  -- 20D current block
  * constrained_albc/envs/main/albc_env.py
        _get_hist_features   (555-616) -- 18D history feature per step
        _update_hist         (617-632) -- strided ring recording
        _get_observations    (1099-1155) -- final 69D concat (jb_hist + act_hist + integral)
        _update_manipulability (720-734) -- Yoshikawa index
        _get_rewards         (1164-1193) -- leaky integral update (leak -> gated add -> clamp)
  * constrained_albc/envs/main/config.py -- all constants (hist_len/stride, integral_*, delta_scale)
  * marinelab/core/thruster.py:apply_dynamics -- first-order ESC filter (tau_up/down)

=== 72D LAYOUT (o_t) ===
Current proprioception (20D, compute_policy_obs):
    [0:3]   ang_cmd = [roll_att_cmd, pitch_att_cmd, yaw_rate_cmd]
    [3:6]   euler (roll, pitch, yaw)                 <- frames.stonefish_odom_to_isaac
    [6:9]   body angular velocity (p, q, r)          <- frames.stonefish_odom_to_isaac
    [9:11]  joint_pos (raw cumulative, [joint1, joint2])
    [11:13] joint_vel
    [13]    manipulability w/w_max in [0,1] (uses joint_pos[1] = joint2)
    [14:20] thruster filtered state (6D, normalized ESC feedback m0..m5)
History (46D, ring buffer hist_len=3, stride=3, 18D/step -> sliced in _get_observations):
    [20:50] jb_hist = ring[:, 0:10] flattened over all 3 steps (oldest->newest):
              per step: joint_pos_err(2)=q_des_prev-q_actual, joint_vel(2),
                        att_rp_err(2, wrapped), yaw_rate_err(1), euler rpy(3)
    [50:66] act_hist = ring[-2:, 10:18] flattened (2 newest steps' action, 8D each)
Integral (3D):
    [66:69] leaky-integrated [roll_err, pitch_err, yaw_rate_err]
Bias-EMA (3D):
    [69:72] ungated leaky EMA of [roll_err, pitch_err, yaw_rate_err]

BIAS-EMA BLOCK [69:72] (added 2026-07-27 for the buoyfix s30 72D pack):
config.py use_bias_ema_obs=True (adopted 260716) appends this 4th block
[69:72] = _bias_ema [roll, pitch, yaw_rate], making obs 72D. The deployed
buoyfix s30 pack IS 72D (policy normalizer mean/std are (1,72)), so this
module assembles the bias_ema block. It is a leaky EMA of the SAME err3 the
integral uses, but UNGATED (albc_env.py _get_rewards updates it every step
when reward.k_bias != 0 -- no error gate), alpha = reward.bias_ema_alpha =
0.99 (main/mdp/rewards.py:119).

=== ObsBuilder.update() CONTRACT (Task 7 bridge wires to this) ===
    update(odom_quat_xyzw, odom_angvel_sf, joint_pos, joint_vel,
           ang_cmd, joint_targets, last_action) -> np.ndarray(69,) float32

Called ONCE per control step (50 Hz), building the obs that produces the NEXT
action. Arguments describe the state AFTER the most recent action was applied:
    odom_quat_xyzw   (4,) nav_msgs/Odometry pose orientation (x,y,z,w), native
                     Stonefish base_link; passed straight to frames.
    odom_angvel_sf   (3,) twist.twist.angular, native body frame; passed to frames.
    joint_pos        (2,) arm joint positions, ORDER [joint1, joint2] (=ALBC_JOINT_NAMES);
                     raw cumulative angle. manipulability uses joint_pos[1] (joint2).
    joint_vel        (2,) arm joint velocities, same order.
    ang_cmd          (3,) [roll_att_cmd, pitch_att_cmd, yaw_rate_cmd].
    joint_targets    (2,) cumulative joint position target q_des the bridge commands
                     (bridge accumulates q_des += delta_scale*action[:2]; delta_scale=0.10).
                     Used only for history joint_pos_err = joint_targets - joint_pos.
    last_action      (8,) the action MOST RECENTLY produced by the policy and applied
                     (2D arm delta + 6D thruster). It advances the thruster filter this
                     step; the ObsBuilder internally remembers the PREVIOUS last_action
                     to populate the history action field (mirrors Isaac's _prev_actions,
                     so the action history lags the thruster state by one recorded step).
                     On the first step pass zeros(8).

Initialization policy (matches Isaac reset _reset_action_buffers/_init_*):
    ring buffer = zeros((3,18)), stride counter = 0, integral = zeros(3),
    thruster state = zeros(6), action memory = zeros(8). The first (stride-1)
    updates therefore emit an all-zero history block, exactly as the Isaac env
    does right after reset -- the ring fills in over the first stride*hist_len
    strided steps.

Known mapping nuance (Task 8 E2E is the behavioral arbiter): Isaac splits each
env step into pre-physics (history recorded from the step-start state, using
q_des_prev) and post-physics (current proprio + integral from the step-end
state). A once-per-step bridge has a single sensor snapshot, so this module uses
the same current state for both the current block and the freshly-recorded
history entry, and takes joint_pos_err = joint_targets - joint_pos directly.
This collapses at most a one-substep offset; the frame/constant/formula matching
is exact.
"""

from __future__ import annotations

import numpy as np

from albc_bridge.frames import stonefish_odom_to_isaac

# --- layout dims (config.py observation_space breakdown) ---
PROPRIO_DIM = 20
HIST_LEN = 3            # config.hist_len
HIST_STRIDE = 3         # config.hist_stride
HIST_FEAT_DIM = 18      # config.hist_feature_dim: joint(4)+body(6)+action(8)
HIST_ACTION_LEN = 2     # config.hist_action_len
ACTION_DIM = 8          # config.action_space
NUM_THRUSTERS = 6       # ALBCThrusterCfg.num_thrusters
BIAS_EMA_DIMS = 3       # _bias_ema block [69:72] (use_bias_ema_obs=True)
OBS_DIM = 72            # config.observation_space (69 + 3 bias-ema)

# --- integral (config.py + rewards.py sigmas) ---
INTEGRAL_DIMS = 3
INTEGRAL_LEAK = 0.99    # config.integral_leak
INTEGRAL_CLAMP = 2.0    # config.integral_clamp
# gate sigmas = [att_rp.sigma, att_rp.sigma, yaw_vel.sigma] (albc_env.py:208-215);
# rewards.py: att_rp.sigma=0.10, yaw_vel.sigma=0.10
INTEGRAL_GATE_SIGMA = np.array([0.10, 0.10, 0.10])
STEP_DT = 0.02          # sim.dt(0.005) * decimation(4)

# --- bias-ema (use_bias_ema_obs=True; UNGATED leaky EMA of err3) ---
BIAS_EMA_ALPHA = 0.99   # reward.bias_ema_alpha (main/mdp/rewards.py:119), ~2s window

# --- thruster first-order filter (ALBCThrusterCfg / thruster.py) ---
THRUSTER_TAU_UP = 0.1     # time_constant_up
THRUSTER_TAU_DOWN = 0.05  # time_constant_down

# --- manipulability (albc.py ALBC_LINK{1,2}_LENGTH) ---
LINK1_LEN = 0.233
LINK2_LEN = 0.233


def _wrap(x):
    """Wrap angle(s) to (-pi, pi] via atan2(sin, cos) (matches _get_hist_features)."""
    return np.arctan2(np.sin(x), np.cos(x))


class ObsBuilder:
    """Stateful assembler for the 69D policy observation (see module docstring)."""

    def __init__(self):
        """Initialize all internal buffers to the Isaac post-reset zero state."""
        self._hist = np.zeros((HIST_LEN, HIST_FEAT_DIM), dtype=np.float64)
        self._hist_counter = 0
        self._integral = np.zeros(INTEGRAL_DIMS, dtype=np.float64)
        self._thr_state = np.zeros(NUM_THRUSTERS, dtype=np.float64)
        # mirrors env._actions: holds the previous call's last_action (a_{k-1});
        # the history entry records the value from BEFORE this shift (a_{k-2}).
        self._prev_action = np.zeros(ACTION_DIM, dtype=np.float64)
        # obs[69:72]: leaky EMA of err3, UNGATED (mirrors env._bias_ema)
        self._bias_ema = np.zeros(BIAS_EMA_DIMS, dtype=np.float64)

    def update(self, odom_quat_xyzw, odom_angvel_sf, joint_pos, joint_vel,
               ang_cmd, joint_targets, last_action):
        """Build and return the current 69D observation (float32). See module docstring."""
        joint_pos = np.asarray(joint_pos, dtype=np.float64)
        joint_vel = np.asarray(joint_vel, dtype=np.float64)
        ang_cmd = np.asarray(ang_cmd, dtype=np.float64)
        joint_targets = np.asarray(joint_targets, dtype=np.float64)
        # env clamps stored/applied actions to [-1, 1] (_update_action_buffers, apply_dynamics)
        last_action = np.clip(np.asarray(last_action, dtype=np.float64), -1.0, 1.0)

        # --- frame conversion (SSOT for obs[3:6] and obs[6:9]) ---
        euler, ang_vel = stonefish_odom_to_isaac(odom_quat_xyzw, odom_angvel_sf)

        # --- tracking errors (att_rp wrapped, yaw rate) ---
        att_rp_err = _wrap(ang_cmd[:2] - euler[:2])
        yaw_rate_err = ang_cmd[2] - ang_vel[2]

        # --- leaky integral update: I = clamp(leak*I + gate*err*dt) (_get_rewards) ---
        err3 = np.array([att_rp_err[0], att_rp_err[1], yaw_rate_err])
        self._integral *= INTEGRAL_LEAK
        gate = (np.abs(err3) < INTEGRAL_GATE_SIGMA).astype(np.float64)
        self._integral += gate * err3 * STEP_DT
        np.clip(self._integral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP, out=self._integral)

        # --- bias-ema update: UNGATED leaky EMA of err3 (mirror _get_rewards, k_bias!=0) ---
        self._bias_ema = BIAS_EMA_ALPHA * self._bias_ema + (1.0 - BIAS_EMA_ALPHA) * err3

        # --- Yoshikawa manipulability (joint2 = joint_pos[1]) ---
        theta2 = joint_pos[1]
        w = np.sqrt(np.abs(LINK1_LEN * LINK2_LEN * np.sin(theta2)))
        manip = w / np.sqrt(LINK1_LEN * LINK2_LEN)

        # --- thruster first-order filter, advanced with the applied action ---
        target = last_action[2:2 + NUM_THRUSTERS]
        tau = np.where(target > self._thr_state, THRUSTER_TAU_UP, THRUSTER_TAU_DOWN)
        self._thr_state = self._thr_state + (STEP_DT / tau) * (target - self._thr_state)

        # --- current proprioception (20D) ---
        current = np.concatenate([
            ang_cmd,                    # [0:3]
            euler,                      # [3:6]
            ang_vel,                    # [6:9]
            joint_pos,                  # [9:11]
            joint_vel,                  # [11:13]
            np.array([manip]),          # [13]
            self._thr_state,            # [14:20]
        ])

        # --- action-buffer shift (mirror _update_action_buffers) ---
        action_for_hist = self._prev_action        # a_{k-2}
        self._prev_action = last_action.copy()      # a_{k-1}

        # --- strided history record (mirror _update_hist) ---
        self._hist_counter += 1
        if self._hist_counter % HIST_STRIDE == 0:
            entry = np.concatenate([
                joint_targets - joint_pos,          # [0:2] joint_pos_err
                joint_vel,                          # [2:4]
                att_rp_err,                         # [4:6]
                np.array([yaw_rate_err]),           # [6]
                euler,                              # [7:10]
                action_for_hist,                    # [10:18]
            ])
            self._hist[:-1] = self._hist[1:]
            self._hist[-1] = entry

        # --- assemble history slices (mirror _get_observations) ---
        jb_hist = self._hist[:, :10].reshape(-1)                      # 10 * hist_len = 30
        act_hist = self._hist[-HIST_ACTION_LEN:, 10:].reshape(-1)     # 8 * action_len = 16

        obs = np.concatenate([current, jb_hist, act_hist, self._integral, self._bias_ema])
        assert obs.shape == (OBS_DIM,), obs.shape
        return obs.astype(np.float32)
