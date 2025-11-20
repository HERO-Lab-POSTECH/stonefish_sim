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
ILOS (Integral Line-of-Sight) Guidance for 4DOF Path Following

Implements ILOS guidance law with integral action for sideslip compensation.
Based on Lekkas & Fossen (2014).

Key Features:
- Integral cross-track error compensation
- 4DOF output (surge, sway, heave, yaw)
- Curvature-based velocity profiling
- Frame: desired_pose (NED world), desired_velocity (FRD body)

Reference:
- Lekkas & Fossen (2014). "Integral LOS Path Following for Curved Paths
  Based on a Monotone Cubic Hermite Spline Parametrization"
- Fossen (2011). "Handbook of Marine Craft Hydrodynamics and Motion Control"
"""

import numpy as np
from transforms3d.euler import quat2euler


def angle_wrap(angle):
    """Wrap angle to [-pi, pi].

    Args:
        angle: Angle in radians

    Returns:
        float: Wrapped angle in [-pi, pi]
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


class ILOSGuidance:
    """ILOS guidance law for 4DOF underactuated UUV.

    ILOS formula (Lekkas & Fossen 2014):
        χ_d = χ_p + arctan(-e_y / Δ) - arctan(κ_ILOS * ∫e_y dt / Δ)

    Where:
        χ_d: Desired heading (yaw)
        χ_p: Path tangent angle
        e_y: Cross-track error (lateral deviation from path)
        Δ: Lookahead distance
        κ_ILOS: Integral gain
        ∫e_y dt: Integral of cross-track error
    """

    def __init__(self, lookahead_distance=3.0, integral_gain=0.05,
                 integral_limit=5.0, cruise_speed=0.5,
                 min_speed=0.2, curvature_gain=2.0, lateral_gain=0.5,
                 depth_gain=0.8, use_alos=True, lookahead_min=1.0,
                 lookahead_max=3.0, k_lookahead_cte=1.0,
                 k_lookahead_curv=2.0, k_lookahead_vel=0.5,
                 cte_threshold=1.0, k_cte_slowdown=0.4):
        """Initialize ILOS guidance.

        Args:
            lookahead_distance: Base lookahead distance Δ (m) - used if ALOS disabled
            integral_gain: ILOS integral gain κ_ILOS
            integral_limit: Anti-windup limit for integral
            cruise_speed: Cruise speed on straight segments (m/s)
            min_speed: Minimum speed in curves (m/s)
            curvature_gain: Curvature-based speed reduction gain
            lateral_gain: Lateral correction gain for cross-track error
            depth_gain: Depth error correction gain (for heave velocity)
            use_alos: Enable ALOS (Adaptive Line-of-Sight)
            lookahead_min: Minimum lookahead distance (m) for ALOS
            lookahead_max: Maximum lookahead distance (m) for ALOS
            k_lookahead_cte: ALOS CTE sensitivity gain
            k_lookahead_curv: ALOS curvature sensitivity gain
            k_lookahead_vel: ALOS velocity coupling gain
            cte_threshold: CTE threshold for speed reduction (m)
            k_cte_slowdown: CTE-based slowdown factor (0-1)
        """
        # ILOS parameters
        self._lookahead_distance = lookahead_distance
        self._integral_gain = integral_gain
        self._integral_limit = integral_limit

        # ALOS parameters (Fossen & Lekkas 2023)
        self._use_alos = use_alos
        self._lookahead_min = lookahead_min
        self._lookahead_max = lookahead_max
        self._k_lookahead_cte = k_lookahead_cte
        self._k_lookahead_curv = k_lookahead_curv
        self._k_lookahead_vel = k_lookahead_vel

        # Velocity profiling parameters
        self._cruise_speed = cruise_speed
        self._min_speed = min_speed
        self._curvature_gain = curvature_gain
        self._lateral_gain = lateral_gain
        self._depth_gain = depth_gain

        # CTE-based velocity reduction (KIOST RPM constraint method)
        self._cte_threshold = cte_threshold
        self._k_cte_slowdown = k_cte_slowdown

        # Vehicle state
        self._vehicle_pos = np.zeros(3)  # [x, y, z] NED world
        self._vehicle_quat = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
        self._vehicle_yaw = 0.0
        self._vehicle_velocity = np.zeros(3)  # [u, v, w] body frame

        # Path management (dense array from nav_msgs/Path)
        self._path_poses = None  # np.array, shape (N, 3)
        self._path_finished = False

        # Path parameter tracking
        self._path_parameter = 0.0  # s ∈ [0, 1]
        self._total_path_length = 0.0
        self._closest_point_idx = 0

        # ILOS integral state
        self._integral_ey = 0.0  # Cross-track error integral

        # Guidance outputs
        self._desired_pos = np.zeros(3)
        self._desired_yaw = 0.0
        self._desired_velocity = np.zeros(4)  # [u, v, w, r]
        self._cross_track_error = 0.0
        self._path_progress = 0.0
        self._current_curvature = 0.0
        self._current_desired_speed = 0.0  # Actual computed speed (for logging)

        # Max cross-track error tracking
        self._max_cte = 0.0

    def set_path(self, path_poses):
        """Set path from dense array of poses.

        Args:
            path_poses: List or array of [x, y, z] positions (NED world frame)
                       Sequential single-direction path
        """
        self._path_poses = np.array(path_poses)
        self._path_finished = False
        self._path_parameter = 0.0
        self._closest_point_idx = 0
        self._integral_ey = 0.0  # Reset integral
        self._max_cte = 0.0

        # Compute total path length
        self._total_path_length = 0.0
        for i in range(1, len(self._path_poses)):
            self._total_path_length += np.linalg.norm(
                self._path_poses[i] - self._path_poses[i-1]
            )

    def update_vehicle_state(self, position, orientation_quat, velocity_world):
        """Update vehicle state from odometry.

        Args:
            position: [x, y, z] NED world frame
            orientation_quat: [w, x, y, z]
            velocity_world: [vx, vy, vz] world frame (NED)
        """
        self._vehicle_pos = np.array(position)
        self._vehicle_quat = np.array(orientation_quat)

        # Extract yaw from quaternion
        roll, pitch, yaw = quat2euler(orientation_quat, 'sxyz')
        self._vehicle_yaw = yaw

        # Convert velocity from world frame to body frame (FRD)
        # Rotation matrix from world (NED) to body (FRD)
        cy = np.cos(yaw)
        sy = np.sin(yaw)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cr = np.cos(roll)
        sr = np.sin(roll)

        # Full rotation matrix (world to body)
        R_wb = np.array([
            [cy*cp, sy*cp, -sp],
            [cy*sp*sr - sy*cr, sy*sp*sr + cy*cr, cp*sr],
            [cy*sp*cr + sy*sr, sy*sp*cr - cy*sr, cp*cr]
        ])

        # Convert velocity to body frame
        velocity_body = R_wb @ np.array(velocity_world)
        self._vehicle_velocity = velocity_body  # [u, v, w] in body frame

    def update(self, dt):
        """Update guidance (find closest point on path).

        Args:
            dt: Time step (s)

        Returns:
            bool: True if successful
        """
        if self._path_poses is None or len(self._path_poses) == 0:
            return False

        # Find closest point on path
        self._find_closest_point()

        # Update path parameter based on closest point
        if self._total_path_length > 0:
            # Calculate accumulated distance to closest point
            accumulated_dist = 0.0
            for i in range(1, self._closest_point_idx + 1):
                accumulated_dist += np.linalg.norm(
                    self._path_poses[i] - self._path_poses[i-1]
                )

            self._path_parameter = accumulated_dist / self._total_path_length
        else:
            self._path_parameter = 0.0

        # Check if path is finished
        total_points = len(self._path_poses)
        near_end = self._closest_point_idx >= total_points - 2
        traveled_enough = self._path_parameter > 0.95

        goal_pos = self._path_poses[-1]
        distance_to_goal = np.linalg.norm(self._vehicle_pos - goal_pos)
        goal_reached = distance_to_goal < self._lookahead_distance

        if near_end and traveled_enough and goal_reached:
            self._path_finished = True
            self._path_parameter = 1.0

        self._path_progress = self._path_parameter

        return True

    def _find_closest_point(self):
        """Find closest point on path to vehicle position."""
        if self._path_poses is None or len(self._path_poses) == 0:
            return

        total_points = len(self._path_poses)

        max_jump = 20
        search_start_idx = self._closest_point_idx
        search_end_idx = min(self._closest_point_idx + max_jump, total_points)

        distances = np.linalg.norm(
            self._path_poses[search_start_idx:search_end_idx] - self._vehicle_pos,
            axis=1
        )

        closest_idx_relative = np.argmin(distances)
        candidate_idx = search_start_idx + closest_idx_relative

        max_increment = 5
        if candidate_idx > self._closest_point_idx + max_increment:
            self._closest_point_idx = self._closest_point_idx + max_increment
        else:
            self._closest_point_idx = candidate_idx


    def _estimate_curvature(self, idx):
        """Estimate curvature at path point using 3-point method.

        Args:
            idx: Index of path point

        Returns:
            float: Curvature (1/m)
        """
        if self._path_poses is None or len(self._path_poses) < 3:
            return 0.0

        # Get 3 consecutive points (handle boundaries)
        idx_prev = max(0, idx - 1)
        idx_curr = idx
        idx_next = min(len(self._path_poses) - 1, idx + 1)

        p_prev = self._path_poses[idx_prev]
        p_curr = self._path_poses[idx_curr]
        p_next = self._path_poses[idx_next]

        # Vectors
        v1 = p_curr - p_prev
        v2 = p_next - p_curr

        # Lengths
        l1 = np.linalg.norm(v1)
        l2 = np.linalg.norm(v2)

        if l1 < 1e-9 or l2 < 1e-9:
            return 0.0

        # Angle between vectors
        cos_angle = np.dot(v1, v2) / (l1 * l2)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.arccos(cos_angle)

        # Curvature approximation: κ ≈ 2 * sin(θ/2) / L
        # where L is average segment length
        L_avg = (l1 + l2) / 2.0
        curvature = 2.0 * np.sin(angle / 2.0) / L_avg

        return curvature

    def compute_guidance(self, dt):
        """Compute ILOS guidance command (hybrid velocity + position architecture).

        Args:
            dt: Time step (s)

        Returns:
            tuple: (desired_position, desired_heading, desired_velocities)
                desired_position: [x, y, z] lookahead point (m, NED world frame)
                desired_heading: ψ_d (rad, world NED frame)
                desired_velocities: [u, v, w, r] (m/s, m/s, m/s, rad/s, FRD body frame)

        Note:
            This is a HYBRID architecture combining:
            - Position control: lookahead point for outer loop stability
            - Velocity control: desired velocities for path tracking
            Both are commanded simultaneously for optimal performance.
        """
        if self._path_poses is None or len(self._path_poses) == 0:
            return self._desired_pos, self._desired_yaw, self._desired_velocity

        # 1. Compute adaptive lookahead distance (ALOS)
        if self._use_alos:
            # Need preliminary curvature and CTE for lookahead calculation
            prelim_lookahead_idx = self._find_lookahead_point(
                self._closest_point_idx, self._lookahead_distance
            )
            prelim_curvature = self._estimate_curvature(prelim_lookahead_idx)

            # Get preliminary CTE (using current closest point)
            p_closest = self._path_poses[self._closest_point_idx]
            prelim_tangent = self._get_path_tangent(self._closest_point_idx)
            chi_p_prelim = np.arctan2(prelim_tangent[1], prelim_tangent[0])
            e_vec = self._vehicle_pos - p_closest
            prelim_cte = -e_vec[0] * np.sin(chi_p_prelim) + e_vec[1] * np.cos(chi_p_prelim)

            # Current surge speed (use cruise_speed if not available)
            current_surge = self._vehicle_velocity[0] if np.linalg.norm(self._vehicle_velocity) > 0.01 else self._cruise_speed

            # Compute adaptive lookahead
            adaptive_lookahead = self._compute_adaptive_lookahead(
                prelim_cte, prelim_curvature, current_surge
            )
        else:
            adaptive_lookahead = self._lookahead_distance

        # 2. Find lookahead point on path
        lookahead_idx = self._find_lookahead_point(
            self._closest_point_idx, adaptive_lookahead
        )
        p_lookahead = self._path_poses[lookahead_idx]

        # 2. Get path tangent at lookahead point
        tangent = self._get_path_tangent(lookahead_idx)
        chi_p = np.arctan2(tangent[1], tangent[0])  # Path angle

        # 3. Calculate cross-track error e_y
        # Find closest point on path
        p_closest = self._path_poses[self._closest_point_idx]

        # Error vector (vehicle - closest point)
        e_vec = self._vehicle_pos - p_closest

        # Cross-track error (perpendicular to path)
        # e_y = lateral component in path frame
        e_y = -e_vec[0] * np.sin(chi_p) + e_vec[1] * np.cos(chi_p)

        self._cross_track_error = e_y
        self._max_cte = max(self._max_cte, abs(e_y))

        # 4. Update integral with anti-windup
        self._integral_ey += e_y * dt
        self._integral_ey = np.clip(self._integral_ey,
                                     -self._integral_limit,
                                     self._integral_limit)

        # 5. ILOS heading command (with adaptive lookahead)
        # Formula: χ_d = χ_p + arctan(-e_y / Δ) - arctan(κ_ILOS * ∫e_y dt / Δ)
        # Both arctan terms use adaptive_lookahead distance for consistent heading computation
        chi_d = chi_p \
                + np.arctan(-e_y / adaptive_lookahead) \
                - np.arctan(self._integral_gain * self._integral_ey / adaptive_lookahead)

        chi_d = angle_wrap(chi_d)

        # 6. Estimate curvature for velocity profiling
        self._current_curvature = self._estimate_curvature(lookahead_idx)

        # 7. Compute desired speed (velocity profiler with CTE reduction)
        desired_speed = self._compute_speed(self._current_curvature, e_y)

        # Store for debugging/logging
        self._current_desired_speed = desired_speed

        # 8. Desired position (lookahead point on path)
        self._desired_pos = p_lookahead
        self._desired_yaw = chi_d

        # 9. Desired velocity (FRD body frame)
        # Surge: desired speed
        # Sway: lateral correction for cross-track error (Lekkas & Fossen 2014)
        # Heave: from path slope
        # Yaw rate: from curvature

        # Lateral correction velocity (sway) - proportional to cross-track error
        # This provides immediate response when robot deviates from path
        # Formula: v_lateral = -K_lateral * e_y
        v_lateral = -self._lateral_gain * e_y

        # Heave velocity: Path-based + Depth error correction
        # Two components:
        # 1. Path-based: Follow the path slope (feedforward)
        # 2. Depth correction: Fix depth error (feedback)
        tangent_xy = tangent[:2]
        tangent_xy_norm = np.linalg.norm(tangent_xy)

        # Path-based heave velocity (feedforward)
        if tangent_xy_norm > 1e-6:
            # 3D path with horizontal component
            # Use cruise_speed scaled by path slope
            w_path = self._cruise_speed * (tangent[2] / tangent_xy_norm)
        else:
            # Pure vertical path
            w_path = self._cruise_speed * np.sign(tangent[2]) if abs(tangent[2]) > 1e-6 else 0.0

        # Depth error correction (feedback)
        # e_z = desired_z - actual_z (NED: positive = need to go down)
        # In NED, Z is down, so positive error means we need to descend
        e_z = p_lookahead[2] - self._vehicle_pos[2]
        w_correction = self._depth_gain * e_z

        # Combined heave velocity
        w_d = w_path + w_correction

        # Yaw rate from curvature (kinematic relationship: r = v * κ)
        # For 3D path, use horizontal curvature only
        if tangent_xy_norm > 1e-6:
            # Horizontal speed component (affected by curvature)
            speed_xy = desired_speed * tangent_xy_norm
            r_d = speed_xy * self._current_curvature
        else:
            # Vertical path, no yaw rate
            r_d = 0.0

        self._desired_velocity = np.array([desired_speed, v_lateral, w_d, r_d])

        # Hybrid architecture: return position, heading, and velocities
        # Position: lookahead point for outer loop control
        # Velocities: desired body velocities for feedforward control
        return self._desired_pos, self._desired_yaw, self._desired_velocity

    def _find_lookahead_point(self, start_idx, lookahead_distance):
        """Find point on path that is lookahead_distance ahead.

        Args:
            start_idx: Starting index
            lookahead_distance: Desired lookahead distance (m)

        Returns:
            int: Index of lookahead point
        """
        if self._path_poses is None or len(self._path_poses) == 0:
            return 0

        total_points = len(self._path_poses)
        accumulated_dist = 0.0
        lookahead_idx = start_idx

        for i in range(start_idx, total_points - 1):
            segment_dist = np.linalg.norm(self._path_poses[i + 1] - self._path_poses[i])
            accumulated_dist += segment_dist

            if accumulated_dist >= lookahead_distance:
                lookahead_idx = i + 1
                break
        else:
            # Reached end of path
            lookahead_idx = total_points - 1

        return lookahead_idx

    def _get_path_tangent(self, idx):
        """Get path tangent at given index.

        Args:
            idx: Path point index

        Returns:
            np.ndarray: Normalized tangent vector [tx, ty, tz]
        """
        if self._path_poses is None or len(self._path_poses) < 2:
            return np.array([1.0, 0.0, 0.0])

        total_points = len(self._path_poses)

        # Use forward difference for tangent
        if idx < total_points - 1:
            tangent = self._path_poses[idx + 1] - self._path_poses[idx]
        else:
            # At end, use backward difference
            tangent = self._path_poses[idx] - self._path_poses[idx - 1]

        # Normalize
        tangent_norm = np.linalg.norm(tangent)
        if tangent_norm > 1e-9:
            tangent = tangent / tangent_norm
        else:
            tangent = np.array([1.0, 0.0, 0.0])

        return tangent

    def _compute_adaptive_lookahead(self, cross_track_error, curvature, surge_speed):
        """Compute adaptive lookahead distance (ALOS guidance law).

        Based on Fossen & Lekkas (2023) "An Adaptive Line-of-sight (ALOS)
        Guidance Law for Path Following of Aircraft and Marine Craft".

        Formula:
            Δ(t) = Δ_min + k_cte|e_y| + k_κ|κ(s)| + k_v·U

        Args:
            cross_track_error: Cross-track error e_y (m)
            curvature: Path curvature κ (1/m)
            surge_speed: Current surge speed U (m/s)

        Returns:
            float: Adaptive lookahead distance Δ(t) (m)
        """
        delta = self._lookahead_min + \
                self._k_lookahead_cte * abs(cross_track_error) + \
                self._k_lookahead_curv * abs(curvature) + \
                self._k_lookahead_vel * surge_speed

        # Saturate to safe limits
        delta = np.clip(delta, self._lookahead_min, self._lookahead_max)

        return delta

    def _compute_speed(self, curvature, cross_track_error):
        """Compute desired speed based on path curvature and CTE.

        Velocity profiling with two components:
        1. Curvature-based reduction (Nature Scientific Reports 2022)
        2. CTE-based reduction (KIOST RPM constraint method)

        Formula:
            u_d = u_cruise / (1.0 + k_curve * |κ|)
            if |CTE| > threshold: u_d *= (1 - k_cte * min(|CTE|/2, 1))
            u_d = max(u_d, u_min)

        Args:
            curvature: Path curvature (1/m)
            cross_track_error: Cross-track error (m)

        Returns:
            float: Desired speed (m/s)
        """
        # 1. Curvature-based reduction (기존)
        speed = self._cruise_speed / (1.0 + self._curvature_gain * abs(curvature))

        # 2. CTE-based reduction (NEW)
        if abs(cross_track_error) > self._cte_threshold:
            cte_penalty = self._k_cte_slowdown * min(
                abs(cross_track_error) / 2.0, 1.0
            )
            speed *= (1.0 - cte_penalty)

        # Clamp to minimum speed
        speed = max(speed, self._min_speed)

        return speed

    def get_guidance_command(self):
        """Get current guidance command.

        Returns:
            dict: Guidance command with keys:
                - desired_yaw: Desired heading (rad)
                - cross_track_error: Cross-track error (m)
                - path_progress: Path completion progress [0, 1]
                - desired_speed: Desired speed (m/s) - actual computed speed
                - cruise_speed: Cruise speed parameter (m/s)
                - current_curvature: Current path curvature (1/m)
        """
        return {
            'desired_yaw': self._desired_yaw,
            'cross_track_error': self._cross_track_error,
            'path_progress': self._path_progress,
            'desired_speed': self._current_desired_speed,  # Actual computed speed
            'cruise_speed': self._cruise_speed,  # Parameter for comparison
            'current_curvature': self._current_curvature,
        }

    def is_path_finished(self):
        """Check if path following is complete.

        Returns:
            bool: True if finished
        """
        return self._path_finished

    def get_max_cross_track_error(self):
        """Get maximum cross-track error encountered.

        Returns:
            float: Max cross-track error (m)
        """
        return self._max_cte

    def reset(self):
        """Reset guidance state."""
        self._path_parameter = 0.0
        self._closest_point_idx = 0
        self._integral_ey = 0.0
        self._path_finished = False
        self._max_cte = 0.0
        self._desired_pos = np.zeros(3)
        self._desired_yaw = 0.0
        self._desired_velocity = np.zeros(4)
