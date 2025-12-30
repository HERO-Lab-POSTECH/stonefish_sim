#!/usr/bin/env python3
# Copyright 2025
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

"""
ALOS (Adaptive Line-of-Sight) Guidance for 4DOF Path Following

Implements ALOS guidance law with direct sideslip estimation for time-varying
current compensation. Extension of ILOS for more robust disturbance rejection.

Key Features:
- Direct sideslip angle estimation (β̂) instead of integral action
- Better adaptation to time-varying currents
- Same 4DOF output as ILOS

Reference:
- Fossen et al. (2015). "Direct and Indirect Adaptive Integral Line-of-Sight
  Path-Following Controllers for Marine Craft Exposed to Ocean Currents"
- Fossen (2023). "An Adaptive LOS Guidance Law for Path Following of
  Underactuated Marine Craft"

Comparison with ILOS:
    ILOS: χ_d = χ_p + arctan(-e_y/Δ) - arctan(κ∫e_y dt/Δ)
    ALOS: χ_d = χ_p + arctan(-e_y/Δ) + β̂
          β̂_dot = γ × e_y × Δ / (Δ² + e_y²)

    ALOS directly estimates sideslip, while ILOS uses integral approximation.
    ALOS converges faster for time-varying currents.
"""

import numpy as np

from .ilos_guidance import ILOSGuidance, PathFollowingMode, angle_wrap


class ALOSGuidance(ILOSGuidance):
    """ALOS guidance law for 4DOF underactuated UUV.

    ALOS formula (Fossen 2015, 2023):
        χ_d = χ_p + arctan(-e_y / Δ) + β̂

    Adaptation law:
        β̂_dot = γ × e_y × Δ / (Δ² + e_y²)

    Where:
        χ_d: Desired heading (yaw)
        χ_p: Path tangent angle
        e_y: Cross-track error (lateral deviation from path)
        Δ: Lookahead distance
        β̂: Estimated sideslip angle (directly estimated)
        γ: Adaptation gain

    Inherits all functionality from ILOSGuidance, overriding only:
        - Heading command computation (ALOS vs ILOS)
        - Reset (additional β̂ state)
    """

    def __init__(
        self,
        lookahead_distance=5.0, cruise_speed=1.0,
        curvature_gain=6.0, lateral_gain=0.6,
        adaptation_gain=0.1, beta_limit=0.52,  # ALOS-specific
        # === Auto-calculated if None ===
        lateral_kd=None, lateral_ki=None,
        depth_gain=None, depth_kd=None, depth_ki=None,
        min_speed=None, min_lookahead=None,
        # === Fixed defaults ===
        heading_align_threshold=np.deg2rad(10.0),
        derivative_filter_tau=0.1,
        max_lateral_velocity=None, max_heave_velocity=None,
        adaptive_lookahead=True,
        curvature_preview_enabled=True, curvature_preview_samples=8
    ):
        """Initialize ALOS guidance.

        Primary Parameters (same as ILOS):
            lookahead_distance, cruise_speed, curvature_gain, lateral_gain

        ALOS-specific:
            adaptation_gain: Sideslip adaptation rate γ (0.05-0.2)
            beta_limit: Max sideslip estimate (rad), ~30° = 0.52
        """
        # Initialize parent ILOS (integral_gain will be ignored)
        super().__init__(
            lookahead_distance=lookahead_distance,
            cruise_speed=cruise_speed,
            curvature_gain=curvature_gain,
            lateral_gain=lateral_gain,
            lateral_kd=lateral_kd,
            lateral_ki=lateral_ki,
            depth_gain=depth_gain,
            depth_kd=depth_kd,
            depth_ki=depth_ki,
            min_speed=min_speed,
            min_lookahead=min_lookahead,
            integral_gain=0.0,  # ALOS doesn't use ILOS integral
            heading_align_threshold=heading_align_threshold,
            derivative_filter_tau=derivative_filter_tau,
            max_lateral_velocity=max_lateral_velocity,
            max_heave_velocity=max_heave_velocity,
            adaptive_lookahead=adaptive_lookahead,
            curvature_preview_enabled=curvature_preview_enabled,
            curvature_preview_samples=curvature_preview_samples
        )

        # ALOS-specific parameters
        self._adaptation_gain = adaptation_gain  # γ
        self._beta_limit = beta_limit  # |β̂| max

        # ALOS state: estimated sideslip angle
        self._beta_hat = 0.0

    def compute_guidance(self, dt):
        """Compute ALOS guidance command.

        Overrides ILOS heading computation with ALOS adaptation law.
        All other functionality (velocity profiling, lateral/depth control)
        is inherited from ILOSGuidance.

        Args:
            dt: Time step (s)

        Returns:
            tuple: (desired_position, desired_heading, desired_velocities)
        """
        if self._path_poses is None or len(self._path_poses) == 0:
            return self._desired_pos, self._desired_yaw, self._desired_velocity

        # 0. Adaptive Lookahead (curvature-based, double-filtered for smoothness)
        if self._adaptive_lookahead:
            # Use max curvature over a window ahead
            preview_dist = self._lookahead_distance_base
            s_preview_end = min(self._path_parameter_s + preview_dist, self._total_path_length)
            curvature_raw = self._estimate_max_curvature_preview(
                self._path_parameter_s, s_preview_end
            )

            # Step 1: Filter the curvature itself (prevents sudden jumps)
            # Asymmetric filter: fast increase, slow decrease
            tau_up = 0.3
            tau_down = 1.5
            if curvature_raw > self._curvature_for_lookahead_filtered:
                alpha_curv = dt / (tau_up + dt)
            else:
                alpha_curv = dt / (tau_down + dt)
            self._curvature_for_lookahead_filtered = (
                alpha_curv * curvature_raw +
                (1 - alpha_curv) * self._curvature_for_lookahead_filtered
            )

            # Step 2: Compute lookahead from filtered curvature
            curvature_factor = 1.0 / (1.0 + 3.0 * self._curvature_for_lookahead_filtered)
            lookahead_target = (
                self._min_lookahead +
                (self._lookahead_distance_base - self._min_lookahead) * curvature_factor
            )

            # Step 3: Filter the lookahead distance
            alpha_la = dt / (self._lookahead_filter_tau + dt)
            self._lookahead_filtered = (
                alpha_la * lookahead_target + (1 - alpha_la) * self._lookahead_filtered
            )
            self._lookahead_distance = self._lookahead_filtered
        else:
            self._lookahead_distance = self._lookahead_distance_base

        # 1. Compute lookahead arc-length parameter
        s_lookahead = self._path_parameter_s + self._lookahead_distance
        s_lookahead = min(s_lookahead, self._total_path_length)

        # 2. Interpolate lookahead point
        p_lookahead = self._interpolate_from_parameter(s_lookahead)

        # 3. Get path tangent
        ds_tangent = 0.1
        s_tangent_ahead = min(self._path_parameter_s + ds_tangent, self._total_path_length)
        p_tangent_ahead = self._interpolate_from_parameter(s_tangent_ahead)

        p_current = self._interpolate_from_parameter(self._path_parameter_s)
        tangent = p_tangent_ahead - p_current
        tangent_norm = np.linalg.norm(tangent)

        if tangent_norm > 1e-6:
            tangent = tangent / tangent_norm
        else:
            tangent = np.array([1.0, 0.0, 0.0])

        chi_p = np.arctan2(tangent[1], tangent[0])

        # 4. Calculate cross-track error e_y
        p_closest = self._interpolate_from_parameter(self._path_parameter_s)
        e_vec = self._vehicle_pos - p_closest
        e_y = -e_vec[0] * np.sin(chi_p) + e_vec[1] * np.cos(chi_p)

        self._cross_track_error = e_y
        self._max_cte = max(self._max_cte, abs(e_y))

        # 5. ALOS adaptation law (Fossen 2015, 2023)
        # β̂_dot = γ × e_y × Δ / (Δ² + e_y²)
        if self._mode == PathFollowingMode.FOLLOW:
            Delta = self._lookahead_distance
            denominator = Delta * Delta + e_y * e_y
            beta_dot = self._adaptation_gain * e_y * Delta / denominator

            # Update sideslip estimate
            self._beta_hat += beta_dot * dt

            # Anti-windup: limit sideslip estimate
            self._beta_hat = np.clip(self._beta_hat, -self._beta_limit, self._beta_limit)

        # 6. ALOS heading command
        if self._mode == PathFollowingMode.ALIGN:
            # ALIGN mode: Use fixed path start tangent
            tangent_start = self._get_path_tangent(0)
            chi_d = np.arctan2(tangent_start[1], tangent_start[0])
        else:
            # FOLLOW mode: ALOS heading with sideslip compensation + curvature FF
            # χ_d = χ_p + arctan(-e_y/Δ) + β̂ + curvature_ff
            signed_curvature = self._estimate_signed_curvature(
                min(self._path_parameter_s + self._lookahead_distance, self._total_path_length)
            )
            curvature_ff = self._curvature_ff_gain * signed_curvature

            chi_d = chi_p + np.arctan(-e_y / self._lookahead_distance) + self._beta_hat + curvature_ff

        chi_d = angle_wrap(chi_d)

        # 7. Check mode transition (ALIGN → FOLLOW)
        heading_error = abs(angle_wrap(chi_d - self._vehicle_yaw))
        if self._mode == PathFollowingMode.ALIGN:
            if heading_error < self._heading_align_threshold:
                self._mode = PathFollowingMode.FOLLOW

        # 8. Estimate curvature for velocity profiling (with extended preview)
        preview_time = 4.0  # Look 4 seconds ahead
        current_speed = max(np.linalg.norm(self._vehicle_velocity[:2]), self._cruise_speed)
        speed_preview_dist = current_speed * preview_time
        min_preview_dist = 5.0  # Always look at least 5m ahead
        preview_dist = max(self._lookahead_distance, speed_preview_dist, min_preview_dist)
        s_preview = min(self._path_parameter_s + preview_dist, self._total_path_length)

        self._current_curvature = self._estimate_max_curvature_preview(
            self._path_parameter_s, s_preview
        )

        # 9. Compute desired speed (velocity profiler)
        curvature_speed = self._compute_speed(self._current_curvature)

        # 9.5. Lookahead distance-based speed reduction
        lookahead_dist = np.linalg.norm(p_lookahead - self._vehicle_pos)
        speed_factor = min(1.0, lookahead_dist / self._lookahead_distance)
        speed_factor = max(speed_factor, 0.1)
        lookahead_speed = self._cruise_speed * speed_factor

        desired_speed = min(curvature_speed, lookahead_speed)
        self._current_desired_speed = desired_speed

        # 10. Desired position (lookahead point on path)
        self._desired_pos = p_lookahead
        self._desired_yaw = chi_d

        # 11. Desired velocity (FRD body frame)
        # Lateral correction velocity (sway) - PID control
        de_y_raw = (e_y - self._prev_ey) / dt if dt > 1e-6 else 0.0
        alpha = dt / (self._filter_tau + dt)
        self._de_y_filtered = alpha * de_y_raw + (1 - alpha) * self._de_y_filtered

        if self._mode == PathFollowingMode.FOLLOW:
            self._integral_ey_lateral += e_y * dt
            self._integral_ey_lateral = np.clip(
                self._integral_ey_lateral,
                -self._integral_limit,
                self._integral_limit
            )

        v_lateral = (
            -self._lateral_gain * e_y
            - self._lateral_ki * self._integral_ey_lateral
            - self._lateral_kd * self._de_y_filtered
        )
        v_lateral = np.clip(v_lateral, -self._max_lateral_velocity, self._max_lateral_velocity)
        self._prev_ey = e_y

        # Heave velocity
        tangent_xy = tangent[:2]
        tangent_xy_norm = np.linalg.norm(tangent_xy)

        if tangent_xy_norm > 1e-6:
            w_path = self._cruise_speed * (tangent[2] / tangent_xy_norm)
        else:
            w_path = self._cruise_speed * np.sign(tangent[2]) if abs(tangent[2]) > 1e-6 else 0.0

        # Depth error correction (feedback) - PID control
        e_z = p_lookahead[2] - self._vehicle_pos[2]
        de_z_raw = (e_z - self._prev_ez) / dt if dt > 1e-6 else 0.0
        alpha_z = dt / (self._filter_tau + dt)
        self._de_z_filtered = alpha_z * de_z_raw + (1 - alpha_z) * self._de_z_filtered

        if self._mode == PathFollowingMode.FOLLOW:
            self._integral_ez += e_z * dt
            self._integral_ez = np.clip(
                self._integral_ez,
                -self._integral_limit,
                self._integral_limit
            )

        w_correction = (
            self._depth_gain * e_z
            + self._depth_ki * self._integral_ez
            + self._depth_kd * self._de_z_filtered
        )
        self._prev_ez = e_z

        w_d = w_path + w_correction
        w_d = np.clip(w_d, -self._max_heave_velocity, self._max_heave_velocity)

        # Yaw rate from curvature
        if tangent_xy_norm > 1e-6:
            speed_xy = desired_speed * tangent_xy_norm
            r_d = speed_xy * self._current_curvature
        else:
            r_d = 0.0

        # 12. ALIGN mode: Slow 3D path tracking
        if self._mode == PathFollowingMode.ALIGN:
            desired_speed = 0.3

        self._desired_velocity = np.array([desired_speed, v_lateral, w_d, r_d])

        return self._desired_pos, self._desired_yaw, self._desired_velocity

    def get_guidance_command(self):
        """Get current guidance command with ALOS-specific info.

        Returns:
            dict: Guidance command with additional beta_hat field
        """
        cmd = super().get_guidance_command()
        cmd['beta_hat'] = self._beta_hat  # ALOS sideslip estimate
        cmd['adaptation_gain'] = self._adaptation_gain
        return cmd

    def reset(self):
        """Reset guidance state including ALOS sideslip estimate."""
        super().reset()
        self._beta_hat = 0.0
