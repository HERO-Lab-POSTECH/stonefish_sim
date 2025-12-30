#!/usr/bin/env python3
# Copyright (c) 2025 Stonefish Control Contributors
# Licensed under the Apache License, Version 2.0

"""
Unified 6DOF Controller with Cascaded Architecture

Implements cascaded control for 4DOF underactuated UUV:
    Outer Loop: Position → Velocity command (P-control)
    Inner Loop: Velocity → Force/Torque (PI-control)

Control DOFs: [Surge, Sway, Heave, Yaw]
Passive DOFs: Roll, Pitch (stabilized by buoyancy)

Reference:
- Fossen (2011) "Handbook of Marine Craft Hydrodynamics and Motion Control"
  Section 12.1: Cascaded Control
- Aström & Hägglund (1995) "PID Controllers: Theory, Design, and Tuning"
  Chapter 3: Anti-Windup
"""

from enum import Enum
from typing import Tuple, Optional
import numpy as np

from ..control_interfaces.data_types import (
    ControlGains,
    ControlLimits,
    VehicleParams,
    TrajectoryReference,
    VehicleState,
    ControlOutput,
    angle_wrap,
    rotation_matrix_full
)


class ControlMode(Enum):
    """Controller operating modes."""
    POSITION = "position"    # Full position tracking (station-keeping)
    VELOCITY = "velocity"    # Velocity tracking (path following)
    HYBRID = "hybrid"        # Velocity + heading from position


class OuterLoopController:
    """Outer loop: Position error → Velocity command.

    Pure P-control. Generates velocity commands from position error.
    Only active in POSITION and HYBRID modes.

    Control Law:
        vel_cmd = Kp × e_position (in body frame)
    """

    def __init__(self, Kp: np.ndarray, max_velocity: np.ndarray):
        """Initialize outer loop controller.

        Args:
            Kp: Proportional gains [4,]
            max_velocity: Maximum velocity command [4,]
        """
        self.Kp = np.asarray(Kp, dtype=np.float64)
        self.max_velocity = np.asarray(max_velocity, dtype=np.float64)

    def compute(
        self,
        pos_des: np.ndarray,
        state: VehicleState
    ) -> Tuple[np.ndarray, dict]:
        """Compute velocity command from position error.

        Args:
            pos_des: Desired position [x, y, z, yaw] NED world
            state: Current vehicle state

        Returns:
            vel_cmd: Velocity command [u, v, w, r] FRD body
            info: Debug information
        """
        # Position error in world frame
        e_pos_world = pos_des[:3] - state.pose[:3]

        # Heading error (wrapped to [-pi, pi])
        e_yaw = angle_wrap(pos_des[3] - state.pose[5])

        # Transform position error to body frame
        R = rotation_matrix_full(state.pose[3], state.pose[4], state.pose[5])
        e_pos_body = R.T @ e_pos_world

        # Combine 4DOF error
        e_4dof = np.array([e_pos_body[0], e_pos_body[1], e_pos_body[2], e_yaw])

        # P-control
        vel_cmd = self.Kp * e_4dof

        # Saturate velocity command
        vel_cmd = np.clip(vel_cmd, -self.max_velocity, self.max_velocity)

        info = {
            'e_pos_world': e_pos_world,
            'e_pos_body': e_pos_body,
            'e_yaw': e_yaw,
            'vel_cmd_raw': self.Kp * e_4dof,
            'vel_cmd_saturated': vel_cmd
        }

        return vel_cmd, info


class InnerLoopController:
    """Inner loop: Velocity error → Force/Torque.

    PI-control with back-calculation anti-windup.
    No D-term (implicit in cascaded architecture).

    Control Law:
        tau = Kp × e_vel + Ki × integral(e_vel) + M × a_ff

    Anti-windup (Aström & Hägglund 1995):
        if saturated:
            integral -= (u - u_sat) / Ki × Kb
    """

    def __init__(
        self,
        Kp: np.ndarray,
        Ki: np.ndarray,
        Kb: np.ndarray,
        saturation_limits: np.ndarray,
        mass_matrix: np.ndarray,
        integral_safety_factor: float = 1.5
    ):
        """Initialize inner loop controller.

        Args:
            Kp: Proportional gains [4,]
            Ki: Integral gains [4,]
            Kb: Anti-windup back-calculation gains [4,]
            saturation_limits: Force/torque limits [4,]
            mass_matrix: 4x4 mass matrix for feedforward
            integral_safety_factor: Integral limit multiplier
        """
        self.Kp = np.asarray(Kp, dtype=np.float64)
        self.Ki = np.asarray(Ki, dtype=np.float64)
        self.Kb = np.asarray(Kb, dtype=np.float64)
        self.sat_limit = np.asarray(saturation_limits, dtype=np.float64)
        self.M = np.asarray(mass_matrix, dtype=np.float64)

        # Calculate integral limits: limit = sat_limit / Ki × safety_factor
        Ki_safe = np.maximum(self.Ki, 1e-6)  # Avoid division by zero
        self.integral_limit = self.sat_limit / Ki_safe * integral_safety_factor

        # State
        self._integral = np.zeros(4)
        self._prev_error = np.zeros(4)

    def compute(
        self,
        vel_des: np.ndarray,
        vel_curr: np.ndarray,
        dt: float,
        feedforward: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, dict]:
        """Compute force/torque from velocity error.

        Args:
            vel_des: Desired velocity [u, v, w, r] FRD body
            vel_curr: Current velocity [u, v, w, r] FRD body
            dt: Time step (seconds)
            feedforward: Feedforward acceleration [4,] (optional)

        Returns:
            tau_4dof: Control output [Fx, Fy, Fz, Mz]
            info: Debug information
        """
        # Velocity error
        e_vel = vel_des - vel_curr

        # P-term
        p_term = self.Kp * e_vel

        # I-term (trapezoidal integration for better accuracy)
        self._integral += 0.5 * (e_vel + self._prev_error) * dt

        # Hard limit on integral (backup anti-windup)
        self._integral = np.clip(self._integral, -self.integral_limit, self.integral_limit)

        i_term = self.Ki * self._integral

        # Feedforward term (optional)
        ff_term = np.zeros(4)
        if feedforward is not None:
            ff_term = self.M @ feedforward

        # Total control (before saturation)
        u_raw = p_term + i_term + ff_term

        # Saturation
        u_sat = np.clip(u_raw, -self.sat_limit, self.sat_limit)

        # Anti-windup: back-calculation (Aström & Hägglund 1995)
        saturated = not np.allclose(u_raw, u_sat, atol=0.01)
        if saturated:
            u_excess = u_raw - u_sat
            Ki_safe = np.maximum(self.Ki, 1e-9)
            integral_correction = (u_excess / Ki_safe) * self.Kb
            self._integral -= integral_correction

        # Update previous error
        self._prev_error = e_vel.copy()

        info = {
            'e_vel': e_vel,
            'p_term': p_term,
            'i_term': i_term,
            'ff_term': ff_term,
            'u_raw': u_raw,
            'u_sat': u_sat,
            'integral': self._integral.copy(),
            'saturated': saturated
        }

        return u_sat, info

    def reset(self):
        """Reset controller state."""
        self._integral = np.zeros(4)
        self._prev_error = np.zeros(4)


class Unified6DOFController:
    """Unified 6DOF Controller with cascaded architecture.

    Supports three control modes:
        - POSITION: Full position tracking (station-keeping)
        - VELOCITY: Direct velocity tracking (path following)
        - HYBRID: Velocity for translation, position for heading

    Architecture:
        Outer Loop (Position) → Inner Loop (Velocity) → Force/Torque

    4DOF Control: [Surge, Sway, Heave, Yaw]
    Passive: Roll, Pitch (stabilized by metacentric height)
    """

    def __init__(
        self,
        gains: ControlGains,
        limits: ControlLimits,
        vehicle_params: VehicleParams,
        control_mode: str = 'velocity',
        feedforward_gain: float = 0.8
    ):
        """Initialize unified controller.

        Args:
            gains: Control gains (outer + inner loop)
            limits: Control limits (velocity + force/torque)
            vehicle_params: Vehicle dynamics parameters
            control_mode: Initial control mode ('position', 'velocity', 'hybrid')
            feedforward_gain: Feedforward scaling factor (0.0 = disabled)
        """
        # Store parameters
        self._gains = gains
        self._limits = limits
        self._vehicle_params = vehicle_params
        self._feedforward_gain = feedforward_gain

        # Initialize mode
        self._mode = ControlMode(control_mode)

        # Create outer loop controller
        self._outer_loop = OuterLoopController(
            Kp=gains.outer.Kp,
            max_velocity=limits.outer.max_velocity
        )

        # Create inner loop controller
        self._inner_loop = InnerLoopController(
            Kp=gains.inner.Kp,
            Ki=gains.inner.Ki,
            Kb=gains.inner.Kb,
            saturation_limits=limits.inner.saturation_limits,
            mass_matrix=vehicle_params.mass_matrix_4dof
        )

        # Mode switch counter
        self._mode_switch_count = 0

    @property
    def mode(self) -> ControlMode:
        """Get current control mode."""
        return self._mode

    def set_mode(self, mode: str) -> None:
        """Set control mode.

        Args:
            mode: Control mode ('position', 'velocity', 'hybrid')

        Note:
            Mode switch resets inner loop integral to prevent transients.
        """
        new_mode = ControlMode(mode)

        if new_mode != self._mode:
            # Reset inner loop on mode switch
            self._inner_loop.reset()
            self._mode = new_mode
            self._mode_switch_count += 1

    def compute(
        self,
        reference: TrajectoryReference,
        state: VehicleState,
        dt: float
    ) -> ControlOutput:
        """Compute 6DOF control wrench.

        Args:
            reference: Desired trajectory point (position + velocity)
            state: Current vehicle state (pose + velocity)
            dt: Time step (seconds)

        Returns:
            ControlOutput with 6DOF wrench and debug info
        """
        info = {
            'mode': self._mode.value,
            'mode_switches': self._mode_switch_count
        }

        # Determine velocity command based on mode
        if self._mode == ControlMode.POSITION:
            # Full position control: outer loop generates velocity command
            vel_cmd, outer_info = self._outer_loop.compute(
                reference.position,
                state
            )
            info['outer_loop'] = outer_info

        elif self._mode == ControlMode.VELOCITY:
            # Direct velocity control: bypass outer loop
            vel_cmd = reference.velocity.copy()
            info['outer_loop'] = {'bypassed': True}

        elif self._mode == ControlMode.HYBRID:
            # Hybrid: velocity for translation, position for heading
            vel_cmd = reference.velocity.copy()

            # Override yaw rate with outer loop yaw control
            _, outer_info = self._outer_loop.compute(reference.position, state)
            vel_cmd[3] = outer_info['vel_cmd_saturated'][3]  # Yaw rate from position error

            info['outer_loop'] = outer_info
            info['outer_loop']['hybrid_mode'] = True

        else:
            raise ValueError(f"Unknown control mode: {self._mode}")

        # Inner loop: velocity tracking
        # Get current 4DOF velocity
        vel_curr = state.velocity_4dof

        # Feedforward acceleration (if provided)
        ff_accel = None
        if reference.acceleration is not None and self._feedforward_gain > 0:
            ff_accel = reference.acceleration * self._feedforward_gain

        # Compute control
        tau_4dof, inner_info = self._inner_loop.compute(
            vel_des=vel_cmd,
            vel_curr=vel_curr,
            dt=dt,
            feedforward=ff_accel
        )
        info['inner_loop'] = inner_info
        info['vel_cmd'] = vel_cmd
        info['vel_curr'] = vel_curr

        # Map 4DOF → 6DOF (Mx = My = 0 for passive roll/pitch)
        tau_6dof = np.array([
            tau_4dof[0],  # Fx (surge)
            tau_4dof[1],  # Fy (sway)
            tau_4dof[2],  # Fz (heave)
            0.0,          # Mx (roll - passive)
            0.0,          # My (pitch - passive)
            tau_4dof[3]   # Mz (yaw)
        ])

        return ControlOutput(tau_6dof=tau_6dof, info=info)

    def reset(self) -> None:
        """Reset all controller states."""
        self._inner_loop.reset()
        self._mode_switch_count = 0

    def get_status(self) -> dict:
        """Get controller status for monitoring.

        Returns:
            Status dictionary with mode, integral, etc.
        """
        return {
            'mode': self._mode.value,
            'mode_switches': self._mode_switch_count,
            'integral': self._inner_loop._integral.copy(),
            'gains': {
                'outer_Kp': self._gains.outer.Kp.tolist(),
                'inner_Kp': self._gains.inner.Kp.tolist(),
                'inner_Ki': self._gains.inner.Ki.tolist()
            }
        }
