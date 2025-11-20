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
from enum import Enum
from transforms3d.euler import quat2euler


class PathFollowingMode(Enum):
    """Path following mode state machine."""
    ALIGN = "align"      # Initial heading alignment (stationary)
    FOLLOW = "follow"    # Normal path following (full motion)


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
                 depth_gain=0.8, heading_align_threshold=np.deg2rad(10.0)):
        """Initialize ILOS guidance.

        Args:
            lookahead_distance: Lookahead distance Δ (m)
            integral_gain: ILOS integral gain κ_ILOS
            integral_limit: Anti-windup limit for integral
            cruise_speed: Cruise speed on straight segments (m/s)
            min_speed: Minimum speed in curves (m/s)
            curvature_gain: Curvature-based speed reduction gain
            lateral_gain: Lateral correction gain for cross-track error
            depth_gain: Depth error correction gain (for heave velocity)
            heading_align_threshold: Heading error threshold for ALIGN→FOLLOW transition (rad)
        """
        # ILOS parameters
        self._lookahead_distance = lookahead_distance
        self._integral_gain = integral_gain
        self._integral_limit = integral_limit

        # Velocity profiling parameters
        self._cruise_speed = cruise_speed
        self._min_speed = min_speed
        self._curvature_gain = curvature_gain
        self._lateral_gain = lateral_gain
        self._depth_gain = depth_gain

        # Heading alignment parameters
        self._heading_align_threshold = heading_align_threshold
        self._mode = PathFollowingMode.ALIGN  # Start in ALIGN mode

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
        """Find closest point using adaptive search window based on cross-track error.

        Adaptive Strategy (Literature-based):
        - Uses distance-based threshold (Fossen 2021, ROS2 Nav2)
        - Converts to index-based window for computational efficiency
        - Large CTE → wider search for recovery
        - Small CTE → narrow search for precision

        Reference:
        - "Time-Varying Lookahead Distance Guidance Law" (IFAC 2016)
        - ROS2 Nav2 Regulated Pure Pursuit Controller
        """
        if self._path_poses is None or len(self._path_poses) == 0:
            return

        total_points = len(self._path_poses)

        # Step 1: Calculate preliminary CTE for adaptive window sizing
        # (Same logic as compute_guidance, but simplified for performance)
        if self._closest_point_idx < total_points:
            p_closest_prelim = self._path_poses[self._closest_point_idx]
            tangent_prelim = self._get_path_tangent(self._closest_point_idx)
            chi_p_prelim = np.arctan2(tangent_prelim[1], tangent_prelim[0])

            e_vec_prelim = self._vehicle_pos - p_closest_prelim
            e_y_prelim = -e_vec_prelim[0] * np.sin(chi_p_prelim) + e_vec_prelim[1] * np.cos(chi_p_prelim)
            cte_prelim = abs(e_y_prelim)
        else:
            cte_prelim = 0.0

        # Step 2: Determine search distance based on CTE (distance-based threshold)
        # Large deviation → wider search for path recovery
        # Small deviation → narrow search for computational efficiency
        if cte_prelim > 2.0:
            search_distance = 2.5  # meters - large deviation recovery
        elif cte_prelim > 1.0:
            search_distance = 1.5  # meters - moderate deviation
        else:
            search_distance = 0.8  # meters - normal tracking

        # Step 3: Convert distance to index window (assumes uniform path spacing)
        # Estimated avg spacing from path generator config: 0.01m
        avg_path_spacing = 0.01  # meters per index
        window_indices = int(search_distance / avg_path_spacing)

        # Step 4: Forward-only search within adaptive window
        search_start = self._closest_point_idx
        search_end = min(self._closest_point_idx + window_indices, total_points)

        # Step 5: Find closest point in window
        distances = np.linalg.norm(
            self._path_poses[search_start:search_end] - self._vehicle_pos,
            axis=1
        )

        closest_idx_relative = np.argmin(distances)
        new_closest_idx = search_start + closest_idx_relative

        # Step 6: Update (no max_increment constraint for immediate response)
        self._closest_point_idx = new_closest_idx


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

        # 1. Find lookahead point on path (continuous interpolation)
        p_lookahead = self._find_lookahead_point_continuous(
            self._closest_point_idx, self._lookahead_distance
        )

        # 2. Get path tangent at lookahead point
        # Find nearest index to continuous lookahead point for tangent calculation
        lookahead_idx = self._find_nearest_index(p_lookahead)
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

        # 4. Update integral with anti-windup (FOLLOW mode only)
        if self._mode == PathFollowingMode.FOLLOW:
            self._integral_ey += e_y * dt
            self._integral_ey = np.clip(self._integral_ey,
                                         -self._integral_limit,
                                         self._integral_limit)

        # 5. ILOS heading command (mode-dependent)
        if self._mode == PathFollowingMode.ALIGN:
            # ALIGN mode: Use fixed path start tangent (no CTE correction)
            # This provides stable heading reference for initial alignment
            tangent_start = self._get_path_tangent(0)  # Path start tangent
            chi_d = np.arctan2(tangent_start[1], tangent_start[0])
        else:
            # FOLLOW mode: Full ILOS with CTE correction and integral action
            chi_d = chi_p \
                    + np.arctan(-e_y / self._lookahead_distance) \
                    - np.arctan(self._integral_gain * self._integral_ey / self._lookahead_distance)

        chi_d = angle_wrap(chi_d)

        # 6. Check mode transition (ALIGN → FOLLOW)
        heading_error = abs(angle_wrap(chi_d - self._vehicle_yaw))

        if self._mode == PathFollowingMode.ALIGN:
            # ALIGN mode: check if heading alignment is complete
            if heading_error < self._heading_align_threshold:
                # Transition to FOLLOW mode
                self._mode = PathFollowingMode.FOLLOW

        # 7. Estimate curvature for velocity profiling
        self._current_curvature = self._estimate_curvature(lookahead_idx)

        # 8. Compute desired speed (velocity profiler)
        desired_speed = self._compute_speed(self._current_curvature)

        # Store for debugging/logging
        self._current_desired_speed = desired_speed

        # 9. Desired position (lookahead point on path)
        self._desired_pos = p_lookahead
        self._desired_yaw = chi_d

        # 10. Desired velocity (FRD body frame)
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

        # 11. ALIGN mode: Slow 3D path tracking for safe path entry
        if self._mode == PathFollowingMode.ALIGN:
            # Track lookahead point with slow speed (smooth 3D approach)
            # This allows gradual depth change along path (NOT vertical drop)
            # p_lookahead already assigned to _desired_pos (line 428), so keep it
            desired_speed = 0.3  # Slow approach speed (m/s) - literature-based safe entry
            # v_lateral: Keep CTE correction (lateral tracking)
            # w_d: Keep path-based heave (gradual Z change along path)
            # r_d: Keep yaw rate (heading alignment)

        self._desired_velocity = np.array([desired_speed, v_lateral, w_d, r_d])

        # Hybrid architecture: return position, heading, and velocities
        # Position: lookahead point for outer loop control
        # Velocities: desired body velocities for feedforward control
        return self._desired_pos, self._desired_yaw, self._desired_velocity

    def _find_lookahead_point_continuous(self, start_idx, lookahead_distance):
        """Find continuous lookahead point using linear interpolation.

        Returns actual 3D point (not index) for smooth controller target updates.
        Eliminates discrete jumps that cause oscillation.

        Args:
            start_idx: Starting index (closest point)
            lookahead_distance: Desired lookahead distance (m)

        Returns:
            np.ndarray: Lookahead point [x, y, z] (continuous, interpolated)
        """
        if self._path_poses is None or len(self._path_poses) == 0:
            return np.zeros(3)

        total_points = len(self._path_poses)

        # Accumulate distance along path from start_idx
        accumulated_dist = 0.0

        for i in range(start_idx, total_points - 1):
            # Distance of this segment
            segment_vec = self._path_poses[i + 1] - self._path_poses[i]
            segment_dist = np.linalg.norm(segment_vec)

            # Check if lookahead point lies within this segment
            if accumulated_dist + segment_dist >= lookahead_distance:
                # Linear interpolation within segment
                remaining_dist = lookahead_distance - accumulated_dist
                alpha = remaining_dist / segment_dist if segment_dist > 1e-9 else 0.0  # [0, 1]

                # Interpolated point
                p_lookahead = self._path_poses[i] + alpha * segment_vec

                return p_lookahead

            accumulated_dist += segment_dist

        # If lookahead exceeds path length, return end point
        return self._path_poses[-1]

    def _find_nearest_index(self, point):
        """Find nearest path index to given point.

        Args:
            point: np.ndarray [x, y, z]

        Returns:
            int: Nearest path index
        """
        if self._path_poses is None or len(self._path_poses) == 0:
            return 0

        distances = np.linalg.norm(self._path_poses - point, axis=1)
        return np.argmin(distances)

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

    def _compute_speed(self, curvature):
        """Compute desired speed based on path curvature.

        Velocity profiling: slow down in curves, speed up on straight segments.

        Formula:
            u_d = u_cruise / (1.0 + k_curve * |κ|)
            u_d = max(u_d, u_min)

        Args:
            curvature: Path curvature (1/m)

        Returns:
            float: Desired speed (m/s)
        """
        # Speed reduction based on curvature
        speed = self._cruise_speed / (1.0 + self._curvature_gain * abs(curvature))

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

    def get_mode(self):
        """Get current path following mode.

        Returns:
            PathFollowingMode: Current mode (ALIGN or FOLLOW)
        """
        return self._mode

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
        self._mode = PathFollowingMode.ALIGN  # Reset to ALIGN mode
