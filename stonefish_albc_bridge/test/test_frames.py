"""Tests for the Stonefish->Isaac frame conversion.

The regression cases are the empirical multi-pose spawn characterization
(2026-07-15): known ``world_transform`` rpy tilts vs the measured sim-time-0
odometry quaternion. These lock in that the conversion is the identity for this
robot's odometry (level == identity, roll about X, pitch about Y).
"""
import numpy as np
from stonefish_albc_bridge.frames import stonefish_odom_to_isaac


def test_pure_identity_maps_to_level():
    euler, w = stonefish_odom_to_isaac(np.array([0.0, 0, 0, 1.0]), np.zeros(3))
    assert np.allclose(euler, [0, 0, 0], atol=1e-9)
    assert np.allclose(w, [0, 0, 0], atol=1e-9)
    assert euler.shape == (3,)


def test_ang_vel_passthrough():
    # body-frame angular velocity is already Isaac FLU; passes through unchanged
    _, w = stonefish_odom_to_isaac(np.array([0.0, 0, 0, 1.0]), np.array([0.1, -0.2, 0.3]))
    assert np.allclose(w, [0.1, -0.2, 0.3], atol=1e-12)


def test_spawn_level_is_near_zero_rp():
    # measured spawn quat at world rpy (0,0,0)
    euler, _ = stonefish_odom_to_isaac(
        np.array([0.00142, 0.00166, -0.04529, 0.99897]), np.zeros(3))
    assert abs(euler[0]) < 0.01   # roll ~0
    assert abs(euler[1]) < 0.01   # pitch ~0


def test_spawn_roll03_tracks_roll():
    # measured spawn quat at world rpy (0.3, 0, 0) -> isaac roll ~0.3, pitch ~0
    euler, _ = stonefish_odom_to_isaac(
        np.array([0.15198, 0.00839, -0.04453, 0.98734]), np.zeros(3))
    assert abs(euler[0] - 0.3) < 0.03   # roll tracks the physical tilt
    assert abs(euler[1]) < 0.05         # pitch stays near 0


def test_spawn_pitch03_tracks_pitch():
    # measured spawn quat at world rpy (0, 0.3, 0) -> isaac pitch ~0.3, roll ~0
    euler, _ = stonefish_odom_to_isaac(
        np.array([-0.00534, 0.15102, -0.04474, 0.98750]), np.zeros(3))
    assert abs(euler[1] - 0.3) < 0.03   # pitch tracks the physical tilt
    assert abs(euler[0]) < 0.05         # roll stays near 0


def test_norm_preserved_on_ang_vel():
    # a pure passthrough preserves the angular-velocity norm
    _, w = stonefish_odom_to_isaac(np.array([0.1, 0.2, 0.3, 0.9]), np.array([0.5, 0, 0]))
    assert np.isclose(np.linalg.norm(w), 0.5, atol=1e-12)
