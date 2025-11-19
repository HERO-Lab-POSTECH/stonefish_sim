# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ROS2 Port: Copyright 2025

"""
PID Regulator

A robust 1D PID controller with anti-windup and derivative filtering.
"""

import numpy as np


class PIDRegulator:
    """A robust 1D PID controller with advanced features.

    Features:
    - Trapezoidal integration for better accuracy
    - Anti-windup through output saturation
    - Low-pass filtering on derivative term to reduce noise amplification
    - Works with both scalar and vector errors

    The controller implements the standard PID control law:
        u(t) = Kp * e(t) + Ki * ∫e(τ)dτ + Kd * de(t)/dt

    Args:
        p (float): Proportional gain (Kp)
        i (float): Integral gain (Ki)
        d (float): Derivative gain (Kd)
        sat (float): Output saturation limit
        derivative_filter_tau (float, optional): Time constant for derivative
            low-pass filter in seconds. Smaller values = more filtering.
            Typical range: 0.01 - 0.1 seconds. Default: 0.05

    Example:
        >>> pid = PIDRegulator(p=10.0, i=0.5, d=2.0, sat=100.0)
        >>> error = 1.5  # Target - current
        >>> time = 0.01  # Current time
        >>> control_output = pid.regulate(error, time)
    """

    def __init__(self, p: float, i: float, d: float, sat: float,
                 derivative_filter_tau: float = 0.05):
        """Initialize PID regulator with gains and saturation."""
        self.p = p
        self.i = i
        self.d = d
        self.sat = sat
        self.derivative_filter_tau = derivative_filter_tau

        # Internal state
        self.integral = 0.0
        self.prev_err = 0.0
        self.prev_t = -1.0
        self.filtered_derivative = 0.0

    def __str__(self) -> str:
        """String representation of PID parameters."""
        return (f'PID controller:\n'
                f'  Kp = {self.p:.4f}\n'
                f'  Ki = {self.i:.4f}\n'
                f'  Kd = {self.d:.4f}\n'
                f'  Saturation = {self.sat:.4f}\n'
                f'  Derivative filter tau = {self.derivative_filter_tau:.4f}s')

    def __repr__(self) -> str:
        """Developer-friendly representation."""
        return (f'PIDRegulator(p={self.p}, i={self.i}, d={self.d}, '
                f'sat={self.sat}, derivative_filter_tau={self.derivative_filter_tau})')

    def regulate(self, err, t: float):
        """Compute PID control output.

        Args:
            err: Error signal (target - current). Can be scalar or numpy array.
            t (float): Current time in seconds

        Returns:
            Control output (same type as err). Saturated to ±self.sat.

        Note:
            First call initializes the controller. Subsequent calls must have
            monotonically increasing time values for proper integration and
            differentiation.
        """
        derr_dt = 0.0
        dt = t - self.prev_t

        # Compute derivative and integral only after first timestep
        if self.prev_t > 0.0 and dt > 0.0:
            # Raw derivative
            raw_derivative = (err - self.prev_err) / dt

            # Apply first-order low-pass filter to derivative
            # Discrete implementation: α = dt / (dt + τ)
            # filtered[k] = α * raw[k] + (1-α) * filtered[k-1]
            alpha = dt / (dt + self.derivative_filter_tau)
            self.filtered_derivative = (alpha * raw_derivative +
                                        (1 - alpha) * self.filtered_derivative)

            derr_dt = self.filtered_derivative

            # Trapezoidal integration for better accuracy
            # ∫e dt ≈ (e[k] + e[k-1]) * dt / 2
            self.integral += 0.5 * (err + self.prev_err) * dt

        # PID control law
        u = self.p * err + self.d * derr_dt + self.i * self.integral

        # Update state
        self.prev_err = err
        self.prev_t = t

        # Saturation with anti-windup
        # If output exceeds limit, clamp it and reset integral
        u_norm = np.linalg.norm(u)
        if u_norm > self.sat:
            # Normalize to saturation limit
            u = self.sat * u / u_norm
            # Anti-windup: reset integral to prevent wind-up
            self.integral = 0.0

        return u

    def reset(self):
        """Reset the PID controller state.

        Clears integral accumulator, derivative filter, and error history.
        Use this when:
        - Switching between different control objectives
        - Starting a new maneuver
        - Recovering from a fault or discontinuity
        """
        self.integral = 0.0
        self.prev_err = 0.0
        self.prev_t = -1.0
        self.filtered_derivative = 0.0

    def get_state(self) -> dict:
        """Get current internal state.

        Returns:
            dict: Dictionary with keys 'integral', 'prev_err', 'prev_t',
                'filtered_derivative'

        Useful for debugging and monitoring controller behavior.
        """
        return {
            'integral': self.integral,
            'prev_err': self.prev_err,
            'prev_t': self.prev_t,
            'filtered_derivative': self.filtered_derivative
        }
