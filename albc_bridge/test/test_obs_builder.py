# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Structural / invariant tests for the 69D ObsBuilder.

There is NO golden for obs assembly (only the policy forward has goldens), so
these assert layout placement, source constants, and buffer invariants against
the Isaac SSOT. Task 8's end-to-end smoke test is the behavioral arbiter.
"""

import numpy as np

from albc_bridge.frames import stonefish_odom_to_isaac
from albc_bridge.obs_builder import (
    HIST_ACTION_LEN,
    HIST_LEN,
    HIST_STRIDE,
    INTEGRAL_CLAMP,
    LINK1_LEN,
    LINK2_LEN,
    OBS_DIM,
    ObsBuilder,
)


def _inputs(**kw):
    d = dict(
        odom_quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        odom_angvel_sf=np.array([0.1, 0.2, 0.3]),
        joint_pos=np.array([0.3, 0.5]),
        joint_vel=np.array([0.01, -0.02]),
        ang_cmd=np.array([0.05, -0.05, 0.1]),
        joint_targets=np.array([0.35, 0.55]),
        last_action=np.array([0.2, -0.2, 0.5, -0.5, 1.0, -1.0, 0.3, -0.3]),
    )
    d.update(kw)
    return d


def test_shape_and_dtype():
    o = ObsBuilder().update(**_inputs())
    assert o.shape == (OBS_DIM,)
    assert o.dtype == np.float32


def test_current_block_placement_matches_frames():
    inp = _inputs()
    o = ObsBuilder().update(**inp)
    euler, ang_vel = stonefish_odom_to_isaac(inp["odom_quat_xyzw"], inp["odom_angvel_sf"])
    np.testing.assert_allclose(o[0:3], inp["ang_cmd"], atol=1e-6)     # ang_cmd
    np.testing.assert_allclose(o[3:6], euler, atol=1e-5)              # euler (frames)
    np.testing.assert_allclose(o[6:9], ang_vel, atol=1e-5)           # body ang vel (frames)
    np.testing.assert_allclose(o[9:11], inp["joint_pos"], atol=1e-6)  # joint pos
    np.testing.assert_allclose(o[11:13], inp["joint_vel"], atol=1e-6)  # joint vel
    assert o[14:20].shape == (6,)                                     # thruster block


def test_manipulability_formula_and_range():
    for th2 in [0.0, np.pi / 2, -np.pi / 2, 1.3, 3.0, -2.7]:
        o = ObsBuilder().update(**_inputs(joint_pos=np.array([0.9, th2])))
        expected = np.sqrt(abs(LINK1_LEN * LINK2_LEN * np.sin(th2))) / np.sqrt(LINK1_LEN * LINK2_LEN)
        assert -1e-9 <= o[13] <= 1.0 + 1e-6
        np.testing.assert_allclose(o[13], expected, atol=1e-6)
    # singularity (sin=0) -> 0, max (sin=1) -> 1
    np.testing.assert_allclose(ObsBuilder().update(**_inputs(joint_pos=np.array([0.0, 0.0])))[13], 0.0, atol=1e-6)
    np.testing.assert_allclose(
        ObsBuilder().update(**_inputs(joint_pos=np.array([0.0, np.pi / 2])))[13], 1.0, atol=1e-6
    )


def test_thruster_first_order_filter_tau_up_down():
    ob = ObsBuilder()
    up = np.concatenate([[0.0, 0.0], np.ones(6)])       # thruster cmd = +1
    o1 = ob.update(**_inputs(last_action=up))
    # alpha_up = dt/tau_up = 0.02/0.1 = 0.2 ; from 0 -> 0.2
    np.testing.assert_allclose(o1[14:20], 0.2, atol=1e-9)
    o2 = ob.update(**_inputs(last_action=up))
    # 0.2 + 0.2*(1-0.2) = 0.36
    np.testing.assert_allclose(o2[14:20], 0.36, atol=1e-9)
    down = np.concatenate([[0.0, 0.0], -np.ones(6)])    # target -1 < state -> tau_down
    o3 = ob.update(**_inputs(last_action=down))
    # alpha_down = 0.02/0.05 = 0.4 ; 0.36 + 0.4*(-1-0.36) = -0.184
    np.testing.assert_allclose(o3[14:20], -0.184, atol=1e-9)


def test_thruster_command_is_clamped():
    over = np.concatenate([[0.0, 0.0], 5.0 * np.ones(6)])   # out of range -> clamp to +1
    o = ObsBuilder().update(**_inputs(last_action=over))
    np.testing.assert_allclose(o[14:20], 0.2, atol=1e-9)     # same as clamped +1 first step


def test_history_ring_is_zero_before_first_strided_record():
    ob = ObsBuilder()
    for i in range(HIST_STRIDE - 1):
        o = ob.update(**_inputs())
        assert np.allclose(o[20:66], 0.0), f"history must be zero before first record (step {i})"


def test_history_ring_records_newest_slot_on_stride():
    ob = ObsBuilder()
    for _ in range(HIST_STRIDE):          # 3rd update triggers the first record
        o = ob.update(**_inputs())
    jb = o[20:50].reshape(HIST_LEN, 10)
    assert np.allclose(jb[0], 0.0) and np.allclose(jb[1], 0.0)   # older slots still zero
    assert not np.allclose(jb[2], 0.0)                          # newest slot recorded


def test_history_joint_tracking_placement():
    inp = _inputs(joint_targets=np.array([0.4, 0.7]), joint_pos=np.array([0.1, 0.2]))
    ob = ObsBuilder()
    for _ in range(HIST_STRIDE):
        o = ob.update(**inp)
    newest = o[20:50].reshape(HIST_LEN, 10)[2]
    np.testing.assert_allclose(newest[0:2], [0.4 - 0.1, 0.7 - 0.2], atol=1e-6)  # q_des - q_actual
    np.testing.assert_allclose(newest[2:4], inp["joint_vel"], atol=1e-6)        # joint_vel


def test_history_body_tracking_placement():
    inp = _inputs()
    ob = ObsBuilder()
    for _ in range(HIST_STRIDE):
        o = ob.update(**inp)
    euler, ang_vel = stonefish_odom_to_isaac(inp["odom_quat_xyzw"], inp["odom_angvel_sf"])
    att_rp_err = np.arctan2(np.sin(inp["ang_cmd"][:2] - euler[:2]), np.cos(inp["ang_cmd"][:2] - euler[:2]))
    yaw_rate_err = inp["ang_cmd"][2] - ang_vel[2]
    newest = o[20:50].reshape(HIST_LEN, 10)[2]
    np.testing.assert_allclose(newest[4:6], att_rp_err, atol=1e-6)   # att_rp_err (wrapped)
    np.testing.assert_allclose(newest[6], yaw_rate_err, atol=1e-6)   # yaw_rate_err
    np.testing.assert_allclose(newest[7:10], euler, atol=1e-5)       # euler rpy


def test_action_history_lags_by_one_recorded_step():
    ob = ObsBuilder()
    a1 = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, -0.8])
    a2 = np.array([-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7, 0.8])
    a3 = np.array([0.9, -0.9, 0.9, -0.9, 0.9, -0.9, 0.9, -0.9])
    ob.update(**_inputs(last_action=a1))          # counter 1, no record
    ob.update(**_inputs(last_action=a2))          # counter 2, no record
    o = ob.update(**_inputs(last_action=a3))      # counter 3, record; action field = a2
    act = o[50:66].reshape(HIST_ACTION_LEN, 8)
    np.testing.assert_allclose(act[1], a2, atol=1e-6)   # newest action-hist is one behind a3


def test_integral_gate_blocks_large_error():
    ob = ObsBuilder()
    for _ in range(500):
        o = ob.update(**_inputs(ang_cmd=np.array([1.0, 1.0, 1.0])))  # |err| >> 0.10 sigma
    np.testing.assert_allclose(o[66:69], 0.0, atol=1e-12)


def test_integral_accumulates_small_error_to_leaky_steady_state():
    ob = ObsBuilder()
    inp0 = _inputs()
    euler, ang_vel = stonefish_odom_to_isaac(inp0["odom_quat_xyzw"], inp0["odom_angvel_sf"])
    small = 0.05  # < 0.10 sigma -> gate open
    ang_cmd = np.array([euler[0] + small, euler[1] + small, ang_vel[2] + small])
    for _ in range(2000):
        o = ob.update(**_inputs(ang_cmd=ang_cmd))
    integral = o[66:69]
    assert np.all(np.abs(integral) <= INTEGRAL_CLAMP + 1e-9)
    # steady state I* = err*dt/(1-leak) = 0.05*0.02/0.01 = 0.10
    np.testing.assert_allclose(integral, 0.10, atol=1e-3)


def test_integral_stays_within_clamp_under_max_gated_accumulation():
    ob = ObsBuilder()
    inp0 = _inputs()
    euler, ang_vel = stonefish_odom_to_isaac(inp0["odom_quat_xyzw"], inp0["odom_angvel_sf"])
    e = 0.099  # just below sigma -> maximal gated accumulation
    ang_cmd = np.array([euler[0] + e, euler[1] + e, ang_vel[2] + e])
    for _ in range(5000):
        o = ob.update(**_inputs(ang_cmd=ang_cmd))
    assert np.all(np.abs(o[66:69]) <= INTEGRAL_CLAMP + 1e-9)
