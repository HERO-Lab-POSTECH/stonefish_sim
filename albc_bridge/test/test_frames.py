"""Unit tests for the Stonefish -> Isaac frame conversion (albc_bridge.frames).

Conventions and evidence are documented in frames.py. These tests pin the two
convention decisions that Task 8's end-to-end smoke test would otherwise be the
only guard for: BODY-frame angular velocity (native base_link axes) and the
FLU/Z-up Isaac observation frame.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'albc_bridge'))

from frames import (  # noqa: E402
    R_ENU_NED,
    R_NATIVE_FLU,
    R_SF_TO_ISAAC,
    euler_xyz_from_quat_wxyz,
    rotmat_to_quat_wxyz,
    stonefish_odom_to_isaac,
)

# Physically-derived UPRIGHT native->ned rotation (columns = native axes in NED):
# body-up=-X_native -> world-up=-Z_ned; body-fwd=-Y_native -> North=X_ned;
# body-left=Z_native -> West=-Y_ned. This is "physically level" and must map to
# policy-level (euler roll=pitch=0).
R_NED_NATIVE_UPRIGHT = np.array([[0., -1., 0.],
                                 [0., 0., -1.],
                                 [1., 0., 0.]])


def _quat_wxyz_to_xyzw(q):
    w, x, y, z = q
    return np.array([x, y, z, w])


def _euler_to_rotmat(roll, pitch, yaw):
    # XYZ extrinsic == R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz.dot(ry).dot(rx)


def test_zero_rate_gives_zero_body_rate():
    """Zero Stonefish angular velocity -> zero Isaac body rate (linear map)."""
    _, ang = stonefish_odom_to_isaac(np.array([0., 0., 0., 1.]), np.zeros(3))
    assert np.allclose(ang, np.zeros(3), atol=1e-12)


def test_ang_vel_norm_preserved():
    """A pure axis relabel preserves angular-velocity magnitude (orthonormal R)."""
    rng = np.random.default_rng(0)
    for _ in range(50):
        w_sf = rng.uniform(-5.0, 5.0, size=3)
        _, w_isaac = stonefish_odom_to_isaac(np.array([0., 0., 0., 1.]), w_sf)
        assert np.isclose(np.linalg.norm(w_isaac), np.linalg.norm(w_sf), atol=1e-12)
    # R_SF_TO_ISAAC is a proper rotation
    assert np.isclose(np.linalg.det(R_SF_TO_ISAAC), 1.0, atol=1e-12)
    assert np.allclose(R_SF_TO_ISAAC.dot(R_SF_TO_ISAAC.T), np.eye(3), atol=1e-12)


def test_ang_vel_axis_sign_mapping():
    """Each native base_link axis maps to the expected Isaac FLU axis, with sign.

    native X=down  -> -Z_flu (down) = (0, 0, -1)
    native Y=back  -> -X_flu (back) = (-1, 0, 0)
    native Z=left  -> +Y_flu (left) = (0, 1, 0)
    """
    q_id = np.array([0., 0., 0., 1.])
    _, ax = stonefish_odom_to_isaac(q_id, np.array([1., 0., 0.]))
    assert np.allclose(ax, np.array([0., 0., -1.]), atol=1e-12)
    _, ay = stonefish_odom_to_isaac(q_id, np.array([0., 1., 0.]))
    assert np.allclose(ay, np.array([-1., 0., 0.]), atol=1e-12)
    _, az = stonefish_odom_to_isaac(q_id, np.array([0., 0., 1.]))
    assert np.allclose(az, np.array([0., 1., 0.]), atol=1e-12)


def test_upright_reference_is_policy_level():
    """Physical upright -> euler roll=pitch=0 (yaw is the arbitrary azimuth)."""
    q_wxyz = rotmat_to_quat_wxyz(R_NED_NATIVE_UPRIGHT)
    euler, _ = stonefish_odom_to_isaac(_quat_wxyz_to_xyzw(q_wxyz), np.zeros(3))
    assert np.isclose(euler[0], 0.0, atol=1e-9)   # roll
    assert np.isclose(euler[1], 0.0, atol=1e-9)   # pitch
    # azimuth-aligned to the Isaac training center (yaw=0): the world relabel
    # composes a -90 deg rotation about vertical, see frames.py docstring
    # "World-vertical-axis alignment" (previously +90 deg, un-aligned).
    assert np.isclose(euler[2], 0.0, atol=1e-9)   # yaw


def test_orientation_roundtrip():
    """A known Isaac attitude survives inverse-map -> forward-map recovery."""
    for roll, pitch, yaw in [(0.2, -0.1, 0.5), (-0.3, 0.15, -1.2), (0.05, 0.0, 2.5)]:
        r_zup_flu = _euler_to_rotmat(roll, pitch, yaw)
        # invert the orientation chain to synthesise the Stonefish quaternion
        r_ned_native = R_ENU_NED.T.dot(r_zup_flu).dot(R_NATIVE_FLU.T)
        q_wxyz = rotmat_to_quat_wxyz(r_ned_native)
        euler, _ = stonefish_odom_to_isaac(_quat_wxyz_to_xyzw(q_wxyz), np.zeros(3))
        assert np.allclose(euler, np.array([roll, pitch, yaw]), atol=1e-9)


def test_euler_formula_matches_isaac_reference():
    """euler_xyz_from_quat_wxyz reproduces the Isaac Lab closed form."""
    # 90 deg about world Z (wxyz): roll=0, pitch=0, yaw=pi/2
    q = np.array([np.cos(np.pi / 4), 0., 0., np.sin(np.pi / 4)])
    assert np.allclose(euler_xyz_from_quat_wxyz(q), np.array([0., 0., np.pi / 2]), atol=1e-9)


def test_output_shapes():
    """Return two float ndarrays of shape (3,)."""
    euler, ang = stonefish_odom_to_isaac(np.array([0.1, 0.2, 0.3, 0.9]), np.array([1., 2., 3.]))
    assert euler.shape == (3,) and ang.shape == (3,)
    assert euler.dtype == float and ang.dtype == float
