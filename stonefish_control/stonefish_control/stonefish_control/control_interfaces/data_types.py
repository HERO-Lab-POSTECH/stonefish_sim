#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Stonefish Control Contributors
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Data Types for Unified 6DOF Controller

Provides dataclasses for controller configuration, vehicle state,
and trajectory references. Designed for clean separation between
ROS2 layer and pure Python control logic.

Reference:
- Fossen (2011) "Handbook of Marine Craft Hydrodynamics and Motion Control"
"""

from dataclasses import dataclass, field
from typing import Optional
import numpy as np


@dataclass
class OuterLoopGains:
    """Outer loop (position → velocity) gains.

    P-control only. Generates velocity commands from position error.

    Attributes:
        Kp: Proportional gains [surge, sway, heave, yaw]
    """
    Kp: np.ndarray = field(default_factory=lambda: np.array([0.5, 0.5, 0.8, 0.5]))

    def __post_init__(self):
        self.Kp = np.asarray(self.Kp, dtype=np.float64)
        if self.Kp.shape != (4,):
            raise ValueError(f"Kp must be shape (4,), got {self.Kp.shape}")


@dataclass
class InnerLoopGains:
    """Inner loop (velocity → force/torque) gains.

    PI-control with anti-windup. No D-term (implicit in cascaded architecture).

    Attributes:
        Kp: Proportional gains [surge, sway, heave, yaw]
        Ki: Integral gains [surge, sway, heave, yaw]
        Kb: Anti-windup back-calculation gains [surge, sway, heave, yaw]
    """
    Kp: np.ndarray = field(default_factory=lambda: np.array([200.0, 200.0, 250.0, 150.0]))
    Ki: np.ndarray = field(default_factory=lambda: np.array([50.0, 50.0, 60.0, 10.0]))
    Kb: np.ndarray = field(default_factory=lambda: np.array([0.8, 0.8, 0.8, 0.8]))

    def __post_init__(self):
        self.Kp = np.asarray(self.Kp, dtype=np.float64)
        self.Ki = np.asarray(self.Ki, dtype=np.float64)
        self.Kb = np.asarray(self.Kb, dtype=np.float64)

        for name, arr in [('Kp', self.Kp), ('Ki', self.Ki), ('Kb', self.Kb)]:
            if arr.shape != (4,):
                raise ValueError(f"{name} must be shape (4,), got {arr.shape}")


@dataclass
class ControlGains:
    """Combined gains for cascaded controller.

    Attributes:
        outer: Outer loop (position) gains
        inner: Inner loop (velocity) gains
    """
    outer: OuterLoopGains = field(default_factory=OuterLoopGains)
    inner: InnerLoopGains = field(default_factory=InnerLoopGains)


@dataclass
class OuterLoopLimits:
    """Outer loop velocity command limits.

    Attributes:
        max_velocity: Maximum velocity [surge, sway, heave, yaw_rate] in m/s and rad/s
    """
    max_velocity: np.ndarray = field(default_factory=lambda: np.array([0.5, 0.5, 0.4, 0.5]))

    def __post_init__(self):
        self.max_velocity = np.asarray(self.max_velocity, dtype=np.float64)
        if self.max_velocity.shape != (4,):
            raise ValueError(f"max_velocity must be shape (4,), got {self.max_velocity.shape}")


@dataclass
class InnerLoopLimits:
    """Inner loop force/torque limits.

    Attributes:
        max_force: Maximum force per linear axis (N)
        max_torque: Maximum torque for yaw axis (Nm)
        integral_limit: Maximum integral value (auto-calculated if None)
    """
    max_force: float = 200.0
    max_torque: float = 50.0
    integral_limit: Optional[np.ndarray] = None

    def __post_init__(self):
        if self.integral_limit is not None:
            self.integral_limit = np.asarray(self.integral_limit, dtype=np.float64)

    @property
    def saturation_limits(self) -> np.ndarray:
        """Get saturation limits as [force, force, force, torque]."""
        return np.array([self.max_force, self.max_force, self.max_force, self.max_torque])


@dataclass
class ControlLimits:
    """Combined limits for cascaded controller.

    Attributes:
        outer: Outer loop (velocity command) limits
        inner: Inner loop (force/torque) limits
    """
    outer: OuterLoopLimits = field(default_factory=OuterLoopLimits)
    inner: InnerLoopLimits = field(default_factory=InnerLoopLimits)


@dataclass
class VehicleParams:
    """Vehicle dynamics parameters for feedforward control.

    Attributes:
        mass: Vehicle mass (kg)
        inertia_zz: Yaw axis moment of inertia (kg·m²)
    """
    mass: float = 20.0
    inertia_zz: float = 0.13

    @property
    def mass_matrix_4dof(self) -> np.ndarray:
        """Get 4DOF mass matrix diag([m, m, m, Izz])."""
        return np.diag([self.mass, self.mass, self.mass, self.inertia_zz])


@dataclass
class TrajectoryReference:
    """Reference trajectory point from path following.

    Coordinate frames:
        - position: NED world frame [x, y, z, yaw]
        - velocity: FRD body frame [u, v, w, r]
        - acceleration: FRD body frame [ax, ay, az, ar]

    Attributes:
        position: Desired position [x, y, z, yaw] in meters and radians
        velocity: Desired velocity [u, v, w, r] in m/s and rad/s
        acceleration: Desired acceleration for feedforward (optional)
    """
    position: np.ndarray = field(default_factory=lambda: np.zeros(4))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(4))
    acceleration: Optional[np.ndarray] = None

    def __post_init__(self):
        self.position = np.asarray(self.position, dtype=np.float64)
        self.velocity = np.asarray(self.velocity, dtype=np.float64)
        if self.acceleration is not None:
            self.acceleration = np.asarray(self.acceleration, dtype=np.float64)


@dataclass
class VehicleState:
    """Current vehicle state from odometry.

    Coordinate frames:
        - pose: NED world frame [x, y, z, roll, pitch, yaw]
        - velocity: FRD body frame [u, v, w, p, q, r]

    Attributes:
        pose: Current pose [x, y, z, roll, pitch, yaw] in meters and radians
        velocity: Current velocity [u, v, w, p, q, r] in m/s and rad/s
    """
    pose: np.ndarray = field(default_factory=lambda: np.zeros(6))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(6))

    def __post_init__(self):
        self.pose = np.asarray(self.pose, dtype=np.float64)
        self.velocity = np.asarray(self.velocity, dtype=np.float64)

    @property
    def position_4dof(self) -> np.ndarray:
        """Get 4DOF position [x, y, z, yaw]."""
        return np.array([self.pose[0], self.pose[1], self.pose[2], self.pose[5]])

    @property
    def velocity_4dof(self) -> np.ndarray:
        """Get 4DOF velocity [u, v, w, r]."""
        return np.array([self.velocity[0], self.velocity[1], self.velocity[2], self.velocity[5]])

    @property
    def orientation(self) -> np.ndarray:
        """Get orientation [roll, pitch, yaw]."""
        return self.pose[3:6]


@dataclass
class ControlOutput:
    """Controller output with debug information.

    Attributes:
        tau_6dof: 6DOF wrench [Fx, Fy, Fz, Mx, My, Mz] (Mx=My=0 for 4DOF)
        info: Debug information dictionary
    """
    tau_6dof: np.ndarray = field(default_factory=lambda: np.zeros(6))
    info: dict = field(default_factory=dict)


def angle_wrap(angle: float) -> float:
    """Wrap angle to [-pi, pi].

    Args:
        angle: Angle in radians

    Returns:
        Wrapped angle in [-pi, pi]
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


def rotation_matrix_z(yaw: float) -> np.ndarray:
    """Create 3x3 rotation matrix for yaw (Z-axis rotation).

    Args:
        yaw: Yaw angle in radians

    Returns:
        3x3 rotation matrix
    """
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])


def rotation_matrix_full(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Create 3x3 rotation matrix from Euler angles (ZYX convention).

    Transforms from body frame (FRD) to world frame (NED).

    Args:
        roll: Roll angle (rad)
        pitch: Pitch angle (rad)
        yaw: Yaw angle (rad)

    Returns:
        3x3 rotation matrix R_b2w
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ])
