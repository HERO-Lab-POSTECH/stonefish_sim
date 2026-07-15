"""Frame conversion: Stonefish odometry -> Isaac-policy observation convention.

This is the single most convention-sensitive seam in the bridge: the numpy
student policy (trained in Isaac Lab / MarineGym) consumes ``euler`` at
obs[3:6] and body-frame angular velocity at obs[6:9]. Feeding it the wrong
frame corrupts the whole observation silently. Everything below is baked in as
plain-numpy constants (NO runtime TF, NO scipy) so the board runtime has zero
extra dependencies.

Derivation (A -> B -> C), each step backed by evidence gathered on the running
sim + the Isaac training code (see task-5-report.md for full transcripts):

A. Stonefish side (what the odometry gives).
   * ``pose.pose.orientation`` (quat xyzw) is the rotation world_ned <- base_link
     in Stonefish's NATIVE base_link axes (X=down, Y=back, Z=left -- established
     by Task 4's base_link_frd static TF). header.frame_id is ``world_ned``
     (X=North, Y=East, Z=Down).
   * ``twist.twist.angular`` is BODY-frame angular velocity, in those same native
     base_link axes. PROVEN empirically: a tumbling-robot capture was compared
     against the world-frame estimate w_world = 2*qdot*conj(q) and the body-frame
     estimate w_body = 2*conj(q)*qdot reconstructed from the quaternion stream;
     the measured twist matched BODY with relative residual 0.45 vs 1.61 for
     world, and the z-sign flips to world at multiple samples. It is NOT
     world-frame despite header.frame_id=world_ned (Stonefish fills the twist in
     the child/body frame).

B. Isaac side (what the policy expects).
   * euler = ``euler_xyz_from_quat(root_quat_w)`` -- XYZ extrinsic Tait-Bryan
     (roll about X, pitch about Y, yaw about Z), quaternion in (w,x,y,z), output
     wrapped to (-pi, pi]. Implemented byte-for-byte below.
   * ang_vel = ``root_ang_vel_b`` -- angular velocity in the BODY frame.
   * World is Z-up (Isaac SimulationCfg default gravity = -Z; confirmed no NED
     override). Body frame is the USD/URDF base link: agent.urdf mounts the arm
     at +Z (joint1 xyz="0 0 0.1625") extending +X, and ALBC_CFG init_state
     rot=(1,0,0,0) is annotated "Upright orientation" -> body = FLU (X-forward,
     Y-left, Z-up), identity quaternion == physically level.

C. Mapping.
   Body angular velocity is a pure axis relabel native -> FLU. Expressing native
   axes in FLU (X_base=down=-Z_flu, Y_base=back=-X_flu, Z_base=left=+Y_flu):

       R_SF_TO_ISAAC = [[ 0, -1,  0],
                        [ 0,  0,  1],
                        [-1,  0,  0]]      # ang_vel_flu = R_SF_TO_ISAAC @ ang_vel_native

   Orientation goes through both a body relabel and a world relabel:

       R_zup_flu = R_ENU_NED @ R(quat_sf) @ R_NATIVE_FLU
       euler     = euler_xyz_from_quat(quat(R_zup_flu))

   with R_NATIVE_FLU = R_SF_TO_ISAAC.T (FLU->native) and R_ENU_NED the NED->Z-up
   world relabel.

Validated: the physically-derived upright orientation (body-up -> world-up)
maps to euler (roll=0, pitch=0, yaw=+90 deg) -- roll and pitch are EXACTLY zero,
confirming the chain puts "physical level" at policy-level.

Residual assumption (non-blocking): the world azimuth reference is arbitrary.
Any valid NED->Z-up relabel that maps gravity correctly differs from this one
only by a rotation about vertical, i.e. a constant YAW offset (here +90 deg from
the ENU choice). roll/pitch are azimuth-invariant and therefore definitive; yaw
is rate-controlled by the policy (obs uses yaw_RATE, no absolute-yaw target), so
a constant yaw offset does not affect hold-level behaviour. The final physical
arbiter is Task 8's end-to-end smoke test: if the robot holds level under the
policy, the frame is right.
"""
import numpy as np

# native base_link body axes -> Isaac FLU body axes (angular velocity)
R_SF_TO_ISAAC = np.array([[0., -1., 0.],
                          [0., 0., 1.],
                          [-1., 0., 0.]])

# world_ned (N,E,D) -> Isaac Z-up ENU (E,N,U)
R_ENU_NED = np.array([[0., 1., 0.],
                      [1., 0., 0.],
                      [0., 0., -1.]])

# Isaac FLU body axes -> native base_link body axes (inverse of R_SF_TO_ISAAC)
R_NATIVE_FLU = R_SF_TO_ISAAC.T


def quat_xyzw_to_rotmat(q):
    """Return the 3x3 rotation matrix for a quaternion given as (x, y, z, w)."""
    q = np.asarray(q, dtype=float)
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotmat_to_quat_wxyz(R):
    """Return the quaternion (w, x, y, z) for a 3x3 rotation matrix R."""
    R = np.asarray(R, dtype=float)
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


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

    Args:
        quat_xyzw: nav_msgs/Odometry pose.pose.orientation as (x, y, z, w) --
            world_ned <- native base_link.
        ang_vel_sf: twist.twist.angular (x, y, z) -- BODY-frame angular velocity
            in native base_link axes.

    Returns:
        A tuple ``(euler_isaac, ang_vel_body_isaac)`` of two float ndarrays,
        shape (3,). ``euler_isaac`` = (roll, pitch, yaw) for obs[3:6];
        ``ang_vel_body_isaac`` = (p, q, r) body rates for obs[6:9].
    """
    R_ned_native = quat_xyzw_to_rotmat(quat_xyzw)
    R_zup_flu = R_ENU_NED.dot(R_ned_native).dot(R_NATIVE_FLU)
    euler_isaac = euler_xyz_from_quat_wxyz(rotmat_to_quat_wxyz(R_zup_flu))
    ang_vel_body_isaac = R_SF_TO_ISAAC.dot(np.asarray(ang_vel_sf, dtype=float))
    return euler_isaac, ang_vel_body_isaac
