#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Stonefish Control Contributors
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
4DOF Underactuated PID Controller for UUV with Passive Roll/Pitch Stability

Reference:
- Fossen, T.I. (2011). "Handbook of Marine Craft Hydrodynamics and Motion Control"
  Section 8.2.1: PID Control for Station-Keeping
- UUV Simulator: rov_ua_pid_controller.py
- Åström & Hägglund (1995): "PID Controllers: Theory, Design, and Tuning"
  Chapter 3: Anti-Windup and Bumpless Transfer

Architecture:
    Position Error → Force (Single-Layer PID)

Control DOFs:
    [X, Y, Z, Yaw] (Surge, Sway, Heave, Yaw)
    Roll, Pitch: Passive stability via buoyancy/gravity (GM ≥ 0.15m)

Frame Convention:
    World: NED (North-East-Down)
    Body: FRD (Forward-Right-Down)
"""

import numpy as np
from typing import Tuple, Optional
from scipy.spatial.transform import Rotation

from ..control_interfaces.data_types import angle_wrap


class PositionController:
    """
    4DOF PID Controller for Underactuated UUV

    Control Law:
        τ = Kp·e_pos + Kd·(-v_body) + Ki·∫e_pos + M·a_ff

    Where:
        e_pos: Position error in body frame [x, y, z, yaw]
        v_body: Body frame velocity [u, v, w, r]
        a_ff: Feedforward acceleration (position mode only)
        M: Mass matrix (for feedforward scaling)

    Anti-Windup:
        Back-calculation method (Åström & Hägglund 1995)
        integral -= (u - u_sat) / Ki * Kb
        where Kb ∈ [0.5, 1.0] is the back-calculation gain
    """

    def __init__(
        self,
        Kp: np.ndarray,
        Kd: np.ndarray,
        Ki: np.ndarray,
        Kb: np.ndarray,
        mass: float,
        inertia_zz: float,
        max_force: float = 200.0,
        max_torque: float = 50.0,
        integral_safety_factor: float = 2.0,
        control_mode: str = 'position'
    ):
        """
        Initialize 4DOF PID Controller

        Args:
            Kp: Proportional gains [4,] (Surge/X, Sway/Y, Heave/Z, Yaw)
            Kd: Derivative gains [4,]
            Ki: Integral gains [4,]
            Kb: Back-calculation gains [4,] (anti-windup, typically 0.5-1.0)
            mass: Vehicle mass (kg)
            inertia_zz: Yaw axis inertia (kg·m²)
            max_force: Maximum force per axis (N)
            max_torque: Maximum torque for yaw (Nm)
            integral_safety_factor: Multiplier for auto-calculated integral limits
                - Calculation: integral_limit = (sat_limit / Ki) × safety_factor
                - Typical values: 1.0-3.0
                - Higher = more integral authority, risk of overshoot
                - Lower = less integral authority, risk of steady-state error
            control_mode: 'position' (default) or 'velocity' for cascaded architecture

        Control Modes:
            - 'position': Tracks position setpoints (traditional)
              Error: e = pos_des - pos_curr
            - 'velocity': Tracks velocity/heading setpoints (cascaded architecture)
              Error: e = [u_des - u, v_des - v, w_des - w, ψ_des - ψ]
        """
        # Control mode
        self.control_mode = control_mode
        # PID Gains (4×4 diagonal matrices)
        self.Kp = np.diag(Kp)  # [4, 4]
        self.Kd = np.diag(Kd)
        self.Ki = np.diag(Ki)
        self.Kb = np.diag(Kb)  # Back-calculation gain

        # Mass matrix (for feedforward scaling)
        # M_4dof = diag([m, m, m, I_zz])
        self.M = np.diag([mass, mass, mass, inertia_zz])

        # Saturation limits
        self.max_force = max_force
        self.max_torque = max_torque
        self.sat_limit = np.array([max_force, max_force, max_force, max_torque])

        # Integral limit (auto-calculated from saturation limits and gains)
        # Formula: integral_limit = (sat_limit / Ki) × safety_factor
        # This ensures integral can contribute up to (safety_factor × 100)% of sat_limit
        # Example: safety_factor=2.0 → integral can contribute up to 200% of sat limit
        #          (combined with P term, allows temporary overforce during transients)
        self.integral_safety_factor = integral_safety_factor
        Ki_diag = np.diag(self.Ki) + 1e-6  # Avoid division by zero
        self.integral_limit = self.sat_limit / Ki_diag * integral_safety_factor

        # State variables
        self.integral = np.zeros(4)  # Integral of position error
        self.prev_error = np.zeros(4)  # Previous error (for trapezoidal integration)

        # Statistics (for debugging/tuning)
        self.saturated_count = 0
        self.max_error_recorded = np.zeros(4)

    def reset(self):
        """Reset controller state (integral, previous error)"""
        self.integral = np.zeros(4)
        self.prev_error = np.zeros(4)
        self.saturated_count = 0

    def compute_control(
        self,
        pose_des: np.ndarray,
        pose_curr: np.ndarray,
        vel_curr: np.ndarray,
        dt: float,
        vel_ff: Optional[np.ndarray] = None,
        accel_ff: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, dict]:
        """
        Compute 4DOF control forces/torques

        Args:
            pose_des: Desired pose/heading
                - Position mode: [x, y, z, yaw] (World NED, rad)
                - Velocity mode: [0, 0, 0, yaw] (only yaw is used)
            pose_curr: Current pose [x, y, z, roll, pitch, yaw] (World NED, rad)
            vel_curr: Current velocity [u, v, w, p, q, r] (Body FRD)
            dt: Time step (s)
            vel_ff: Velocity argument, mode-dependent [u, v, w, r] (Body FRD)
                - Velocity mode: Desired velocity setpoint
                - Position mode: unused for feedforward (see accel_ff) — kept for
                  backward-compatible call signature
            accel_ff: Optional feedforward acceleration [u̇, v̇, ẇ, ṙ] (Body FRD),
                POSITION MODE ONLY. F_ff = M·a (Newton's 2nd law). None = no
                feedforward. (T1.3: corrects the prior M·velocity dimensional error.)

        Returns:
            tau_6dof: Control wrench [Fx, Fy, Fz, 0, 0, Mz] (Body FRD)
            debug_info: Dictionary with debug information
        """
        # Current orientation (roll, pitch, yaw)
        roll, pitch, yaw = pose_curr[3], pose_curr[4], pose_curr[5]

        # ============================================================
        # 1. Compute 4DOF errors based on control mode
        # ============================================================
        if self.control_mode == 'velocity':
            # ========== VELOCITY MODE (Cascaded Architecture) ==========
            # Error: e = [u_des - u, v_des - v, w_des - w, ψ_des - ψ]

            if vel_ff is None:
                raise ValueError("Velocity mode requires vel_ff (desired velocities)")

            # Velocity errors (body frame)
            v_4dof = np.array([vel_curr[0], vel_curr[1], vel_curr[2], vel_curr[5]])  # [u, v, w, r]
            vel_des = vel_ff[:4]  # vel_ff is actually desired velocity in velocity mode

            e_vel = vel_des[:3] - v_4dof[:3]  # [e_u, e_v, e_w]

            # Heading error (world frame)
            heading_des = pose_des[3]
            e_heading = self._angle_wrap(heading_des - yaw)

            # Combined 4DOF error
            e_4dof = np.array([e_vel[0], e_vel[1], e_vel[2], e_heading])

        else:
            # ========== POSITION MODE (Traditional) ==========
            # Error: e = [e_x, e_y, e_z, e_ψ] in body frame

            # Position error (NED world frame)
            e_pos_world = pose_des[:3] - pose_curr[:3]  # [x, y, z]

            # Yaw error (world frame)
            e_yaw = self._angle_wrap(pose_des[3] - yaw)

            # Rotation matrix: World NED → Body FRD
            R = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_matrix()
            e_pos_body = R.T @ e_pos_world  # Transform to body frame

            # Combined 4DOF error (body frame)
            e_4dof = np.array([e_pos_body[0], e_pos_body[1], e_pos_body[2], e_yaw])

        # Update max error (statistics)
        self.max_error_recorded = np.maximum(self.max_error_recorded, np.abs(e_4dof))

        # ============================================================
        # 3. Extract 4DOF velocities (Body frame)
        # ============================================================
        v_4dof = np.array([vel_curr[0], vel_curr[1], vel_curr[2], vel_curr[5]])  # [u, v, w, r]

        # ============================================================
        # 4. PID Computation
        # ============================================================
        # P term (position error, body frame)
        p_term = self.Kp @ e_4dof

        # D term (velocity damping, body frame)
        # Note: Using -v instead of de/dt (recommended by Fossen 2011)
        d_term = self.Kd @ (-v_4dof)

        # I term (trapezoidal integration for better accuracy)
        # ∫e dt ≈ (e_k + e_{k-1}) * dt / 2
        self.integral += 0.5 * (e_4dof + self.prev_error) * dt

        # Backup integral limit (hard clamping)
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)

        i_term = self.Ki @ self.integral

        # Store current error for next iteration
        self.prev_error = e_4dof.copy()

        # ============================================================
        # 5. Feedforward (optional, POSITION MODE ONLY)
        # ============================================================
        ff_term = np.zeros(4)
        if self.control_mode == 'position' and accel_ff is not None:
            # Feedforward: F_ff = M·a (POSITION MODE, Newton's 2nd law).
            # accel_ff is a body-frame acceleration [u̇, v̇, ẇ, ṙ]; M·a yields the
            # force/torque needed to produce that acceleration. (T1.3: was M·velocity,
            # which is momentum, not force — a dimensional error.)
            ff_term = self.M @ accel_ff
        # VELOCITY MODE: No feedforward (vel_ff contains desired velocities, not feedforward)

        # ============================================================
        # 6. Total control (before saturation)
        # ============================================================
        u_4dof = p_term + d_term + i_term + ff_term

        # ============================================================
        # 7. Saturation with Back-Calculation Anti-Windup
        # ============================================================
        u_sat_4dof = np.clip(u_4dof, -self.sat_limit, self.sat_limit)

        # Check if saturated
        saturated = not np.allclose(u_4dof, u_sat_4dof, atol=0.01)
        if saturated:
            self.saturated_count += 1

            # Back-calculation anti-windup (Åström & Hägglund 1995)
            # Adjust integral to prevent windup
            # integral -= (u - u_sat) / Ki * Kb
            u_excess = u_4dof - u_sat_4dof  # Excess control (saturated part)

            # Back-calculate integral correction
            # Avoid division by zero
            Ki_diag = np.diag(self.Ki) + 1e-9
            Kb_diag = np.diag(self.Kb)

            integral_correction = (u_excess / Ki_diag) * Kb_diag
            self.integral -= integral_correction

        # ============================================================
        # 8. Map 4DOF → 6DOF (zero roll/pitch moments)
        # ============================================================
        tau_6dof = np.array([
            u_sat_4dof[0],  # Fx (surge)
            u_sat_4dof[1],  # Fy (sway)
            u_sat_4dof[2],  # Fz (heave)
            0.0,            # Mx (roll) - passive stability
            0.0,            # My (pitch) - passive stability
            u_sat_4dof[3]   # Mz (yaw)
        ])

        # ============================================================
        # 9. Debug information
        # ============================================================
        debug_info = {
            'control_mode': self.control_mode,
            'e_4dof': e_4dof,
            'v_4dof': v_4dof,
            'p_term': p_term,
            'd_term': d_term,
            'i_term': i_term,
            'ff_term': ff_term,
            'integral': self.integral.copy(),
            'u_4dof': u_4dof,
            'u_sat_4dof': u_sat_4dof,
            'saturated': saturated,
            'saturation_count': self.saturated_count
        }

        # Mode-specific debug info
        if self.control_mode == 'position':
            # Position mode variables (not available in velocity mode)
            e_pos_world = pose_des[:3] - pose_curr[:3]
            e_yaw = self._angle_wrap(pose_des[3] - pose_curr[5])

            R = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_matrix()
            e_pos_body = R.T @ e_pos_world

            debug_info['e_pos_world'] = e_pos_world
            debug_info['e_pos_body'] = e_pos_body
            debug_info['e_yaw'] = e_yaw
        elif self.control_mode == 'velocity':
            # Velocity mode variables
            debug_info['vel_des'] = vel_ff[:3] if vel_ff is not None else np.zeros(3)
            debug_info['heading_des'] = pose_des[3]
            debug_info['heading_curr'] = yaw

        return tau_6dof, debug_info

    @staticmethod
    def _angle_wrap(angle: float) -> float:
        """Wrap angle to [-π, π].

        Delegates to the package SSOT ``control_interfaces.data_types.angle_wrap``
        (identical modulo algorithm) to avoid a duplicated implementation.
        """
        return angle_wrap(angle)

    def get_status(self) -> dict:
        """Get controller status for monitoring"""
        return {
            'integral': self.integral.copy(),
            'prev_error': self.prev_error.copy(),
            'max_error': self.max_error_recorded.copy(),
            'saturation_count': self.saturated_count,
            'gains': {
                'Kp': np.diag(self.Kp),
                'Kd': np.diag(self.Kd),
                'Ki': np.diag(self.Ki),
                'Kb': np.diag(self.Kb)
            }
        }

    def update_gains(self, Kp: np.ndarray, Kd: np.ndarray, Ki: np.ndarray, Kb: np.ndarray):
        """
        Update PID gains (useful for online tuning/optimization)

        Args:
            Kp, Kd, Ki, Kb: New gains [4,]
        """
        self.Kp = np.diag(Kp)
        self.Kd = np.diag(Kd)
        self.Ki = np.diag(Ki)
        self.Kb = np.diag(Kb)

        # Reset integral when gains change (avoid transient issues)
        self.integral = np.zeros(4)
