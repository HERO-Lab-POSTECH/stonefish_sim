"""Frame conversion: Stonefish odometry -> Isaac-policy observation convention.

This is the single most convention-sensitive seam in the bridge: the numpy
student policy (trained in Isaac Lab / MarineGym) consumes ``euler`` at
obs[3:6] and body-frame angular velocity at obs[6:9]. Feeding it the wrong
frame corrupts the whole observation silently.

=== Empirical finding (2026-07-15, multi-pose spawn characterization) ===
The Stonefish odometry for this robot is ALREADY in the Isaac convention:
level == identity quaternion, roll about body X, pitch about body Y. Verified
by spawning at known ``world_transform`` rpy tilts and reading the sim-time-0
(spawn, zero-drift) odometry quaternion, then applying Isaac's
``euler_xyz_from_quat`` directly (NO relabel):

    spawn rpy        odom quat (xyzw)                direct euler_xyz (roll,pitch,yaw)
    (0,0,0) level    ( 0.001, 0.002,-0.045, 0.999)   ( 0.003,  0.003, -0.091)  -> level
    (0.3,0,0) roll   ( 0.152, 0.008,-0.045, 0.987)   ( 0.304,  0.030, -0.086)  -> roll  ~0.3
    (0,0.3,0) pitch  (-0.005, 0.151,-0.045, 0.988)   (-0.025,  0.302, -0.094)  -> pitch ~0.3

(The constant ~-0.09 rad yaw is a small fixed spawn offset; absolute yaw does
not enter attitude control -- the policy regulates yaw-RATE only. A pure-yaw
spawn was dynamically unstable and drifted before capture, so it is omitted;
roll/pitch -- the attitude-hold channels -- track directly and unambiguously.)

Therefore the conversion is the IDENTITY: euler = euler_xyz_from_quat(odom
quat), and body angular velocity passes through unchanged (the twist.angular
body frame coincides with the Isaac FLU body frame, consistent with the
orientation result). No world/body relabel, no NED assumption.

This SUPERSEDES the earlier derivation (NED world + X-down/Y-back/Z-left native
base_link, with R_ENU_NED / R_NATIVE_FLU / R_SF_TO_ISAAC relabels), which was
built on assumed conventions that Task 8's end-to-end smoke test disproved: a
physically level robot mapped to isaac roll = -pi/2, so the policy fought a
phantom 90-deg roll and the whole system diverged. Task 5's math was
self-consistent but calibrated to the wrong "upright" reference.

Isaac euler convention (unchanged, byte-for-byte from
``isaaclab.utils.math.euler_xyz_from_quat``): XYZ extrinsic Tait-Bryan (roll
about X, pitch about Y, yaw about Z), quaternion (w,x,y,z), output wrapped to
(-pi, pi].
"""
import numpy as np


def euler_xyz_from_quat_wxyz(q):
    """Return (roll, pitch, yaw) for a quaternion (w, x, y, z).

    Byte-for-byte port of Isaac Lab ``isaaclab.utils.math.euler_xyz_from_quat``
    (XYZ extrinsic convention, output in (-pi, pi]).
    """
    w, x, y, z = np.asarray(q, dtype=float)
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sin_pitch)
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def stonefish_odom_to_isaac(quat_xyzw, ang_vel_sf):
    """Convert Stonefish odometry into the Isaac policy observation convention.

    The Stonefish odometry is already in the Isaac convention for this robot
    (see module docstring): the conversion is the identity apart from the
    (x,y,z,w)->(w,x,y,z) quaternion element reorder Isaac's euler helper wants.

    Args:
        quat_xyzw: nav_msgs/Odometry pose.pose.orientation as (x, y, z, w).
        ang_vel_sf: twist.twist.angular (x, y, z) -- body-frame angular velocity.

    Returns:
        ``(euler_isaac, ang_vel_body_isaac)``, two float ndarrays shape (3,).
        ``euler_isaac`` = (roll, pitch, yaw) for obs[3:6];
        ``ang_vel_body_isaac`` = (p, q, r) body rates for obs[6:9].
    """
    q = np.asarray(quat_xyzw, dtype=float)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    euler_isaac = euler_xyz_from_quat_wxyz(np.array([w, x, y, z]))
    ang_vel_body_isaac = np.asarray(ang_vel_sf, dtype=float).copy()
    return euler_isaac, ang_vel_body_isaac
