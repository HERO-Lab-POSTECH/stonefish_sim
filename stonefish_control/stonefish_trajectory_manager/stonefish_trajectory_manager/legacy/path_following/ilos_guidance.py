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
                 depth_gain=0.8, heading_align_threshold=np.deg2rad(10.0),
                 lateral_kd=0.3, depth_kd=0.5,
                 max_lateral_velocity=0.5, max_heave_velocity=0.4):
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
            lateral_kd: Lateral velocity derivative gain (damping)
            depth_kd: Depth velocity derivative gain (damping)
            max_lateral_velocity: Maximum sway velocity limit (m/s)
            max_heave_velocity: Maximum heave velocity limit (m/s)
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

        # PD control parameters
        self._lateral_kd = lateral_kd
        self._depth_kd = depth_kd
        self._max_lateral_velocity = max_lateral_velocity
        self._max_heave_velocity = max_heave_velocity

        # Previous errors for derivative calculation
        self._prev_ey = 0.0
        self._prev_ez = 0.0

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

        # Path parameter tracking (arc-length based)
        self._path_parameter_s = 0.0  # Arc-length parameter (m)
        self._total_path_length = 0.0
        self._arc_lengths = None  # Cumulative arc-length for each path point

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
        """Set path with arc-length parametrization.

        Args:
            path_poses: Array of [x, y, z] positions (NED world frame)
        """
        self._path_poses = np.array(path_poses)
        self._path_finished = False
        self._integral_ey = 0.0
        self._max_cte = 0.0

        # Compute cumulative arc-length for each path point
        n_points = len(self._path_poses)
        self._arc_lengths = np.zeros(n_points)

        for i in range(1, n_points):
            segment_dist = np.linalg.norm(self._path_poses[i] - self._path_poses[i-1])
            self._arc_lengths[i] = self._arc_lengths[i-1] + segment_dist

        self._total_path_length = self._arc_lengths[-1]

        # Initialize continuous path parameter (arc-length in meters)
        self._path_parameter_s = 0.0

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
        """Update path parameter using continuous projection.

        Args:
            dt: Time step (s)

        Returns:
            bool: True if successful
        """
        if self._path_poses is None or len(self._path_poses) == 0:
            return False

        # Step 1: Determine forward search window (self-crossing prevention)
        robot_speed = np.linalg.norm(self._vehicle_velocity[:2])  # Horizontal speed
        time_horizon = 2.0  # seconds (look ahead in time)
        speed_based_window = robot_speed * time_horizon
        lookahead_based_window = self._lookahead_distance * 3.0
        search_window = max(speed_based_window, lookahead_based_window, 2.0)  # Min 2m

        # Step 2: Small backward tolerance for path crossings
        backward_tolerance = 0.2  # meters
        search_start_s = max(0.0, self._path_parameter_s - backward_tolerance)

        # Step 3: Project robot onto path (continuous parameter)
        projected_s = self._project_to_path(
            self._vehicle_pos,
            search_start_s,
            search_window
        )

        # Step 4: Soft monotonicity constraint (prevent large backward jumps)
        if projected_s < self._path_parameter_s - backward_tolerance:
            projected_s = self._path_parameter_s  # Freeze at current position

        # Step 5: Update path parameter
        self._path_parameter_s = projected_s

        # Step 6: Update progress
        if self._total_path_length > 0:
            self._path_progress = projected_s / self._total_path_length
        else:
            self._path_progress = 0.0

        # Step 7: Check path completion
        distance_to_end = self._total_path_length - projected_s
        near_end = distance_to_end < self._lookahead_distance

        goal_pos = self._path_poses[-1]
        distance_to_goal = np.linalg.norm(self._vehicle_pos - goal_pos)
        goal_reached = distance_to_goal < self._lookahead_distance

        if near_end and goal_reached and self._path_progress > 0.95:
            self._path_finished = True
            self._path_progress = 1.0

        return True

    def _arc_length_to_index(self, s):
        """Convert arc-length parameter to nearest path index.

        Args:
            s: Arc-length parameter (m)

        Returns:
            int: Nearest path index
        """
        if self._arc_lengths is None or len(self._arc_lengths) == 0:
            return 0

        # Binary search for efficient lookup
        idx = np.searchsorted(self._arc_lengths, s, side='right') - 1
        return np.clip(idx, 0, len(self._path_poses) - 1)

    def _interpolate_from_parameter(self, s):
        """Interpolate 3D point from arc-length parameter.

        Uses linear interpolation between path points for continuous positioning.

        Args:
            s: Arc-length parameter (m)

        Returns:
            np.ndarray: Interpolated point [x, y, z]
        """
        if self._path_poses is None or len(self._path_poses) == 0:
            return np.zeros(3)

        # Clamp to valid range
        s = np.clip(s, 0.0, self._total_path_length)

        # Find segment containing s (binary search)
        idx = np.searchsorted(self._arc_lengths, s, side='right') - 1
        idx = np.clip(idx, 0, len(self._path_poses) - 2)

        # Arc-lengths of segment endpoints
        s1 = self._arc_lengths[idx]
        s2 = self._arc_lengths[idx + 1]

        # Handle degenerate segment
        if abs(s2 - s1) < 1e-9:
            return self._path_poses[idx].copy()

        # Linear interpolation parameter alpha ∈ [0, 1]
        alpha = (s - s1) / (s2 - s1)

        # Interpolate position
        p1 = self._path_poses[idx]
        p2 = self._path_poses[idx + 1]

        return p1 + alpha * (p2 - p1)

    def _project_to_path(self, robot_pos, search_start_s, search_window):
        """Project robot position onto path using forward search.

        Finds continuous arc-length parameter by projecting robot onto path segments
        within search window. Prevents self-crossing issues.

        Args:
            robot_pos: Robot position [x, y, z] (NED)
            search_start_s: Start arc-length for search (m)
            search_window: Forward search distance (m)

        Returns:
            float: Projected arc-length parameter s (m)
        """
        if self._path_poses is None or len(self._path_poses) == 0:
            return 0.0

        # Convert search range to indices
        start_idx = self._arc_length_to_index(search_start_s)
        end_s = min(search_start_s + search_window, self._total_path_length)
        end_idx = self._arc_length_to_index(end_s)
        end_idx = min(end_idx + 1, len(self._path_poses) - 1)  # Include end segment

        # Project onto each segment in search window
        min_dist = float('inf')
        best_s = search_start_s

        for i in range(start_idx, end_idx):
            # Segment endpoints
            p1 = self._path_poses[i]
            p2 = self._path_poses[i + 1]

            # Vector projection onto segment
            v = p2 - p1  # Segment vector
            w = robot_pos - p1  # Robot relative to segment start

            c1 = np.dot(w, v)
            c2 = np.dot(v, v)

            # Handle degenerate segment
            if c2 < 1e-9:
                continue

            # Projection parameter (clamped to [0, 1])
            alpha = np.clip(c1 / c2, 0.0, 1.0)

            # Projected point on segment
            p_proj = p1 + alpha * v

            # Distance from robot to projection
            dist = np.linalg.norm(robot_pos - p_proj)

            # Update best projection
            if dist < min_dist:
                min_dist = dist
                # Arc-length of projection
                segment_length = self._arc_lengths[i + 1] - self._arc_lengths[i]
                best_s = self._arc_lengths[i] + alpha * segment_length

        return best_s


    def _estimate_curvature(self, s):
        """Estimate curvature at arc-length parameter s using 3-point method.

        Args:
            s: Arc-length parameter (m)

        Returns:
            float: Curvature (1/m)
        """
        if self._path_poses is None or len(self._path_poses) < 3:
            return 0.0

        # Sample 3 points: s-0.1m, s, s+0.1m
        ds = 0.1  # 10cm spacing for curvature estimation
        s_prev = max(0.0, s - ds)
        s_curr = s
        s_next = min(self._total_path_length, s + ds)

        # Interpolate 3 points
        p_prev = self._interpolate_from_parameter(s_prev)
        p_curr = self._interpolate_from_parameter(s_curr)
        p_next = self._interpolate_from_parameter(s_next)

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
        L_avg = (l1 + l2) / 2.0
        curvature = 2.0 * np.sin(angle / 2.0) / L_avg if L_avg > 1e-9 else 0.0

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

        # 1. Compute lookahead arc-length parameter
        s_lookahead = self._path_parameter_s + self._lookahead_distance
        s_lookahead = min(s_lookahead, self._total_path_length)

        # 2. Interpolate lookahead point (continuous)
        p_lookahead = self._interpolate_from_parameter(s_lookahead)

        # 3. Get path tangent (from nearby point for numerical derivative)
        ds_tangent = 0.1  # 10cm ahead for tangent estimation
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
        # Use interpolated closest point (continuous)
        p_closest = self._interpolate_from_parameter(self._path_parameter_s)

        # Error vector (vehicle - closest point)
        e_vec = self._vehicle_pos - p_closest

        # Cross-track error (perpendicular to path)
        # e_y = lateral component in path frame
        e_y = -e_vec[0] * np.sin(chi_p) + e_vec[1] * np.cos(chi_p)

        self._cross_track_error = e_y
        self._max_cte = max(self._max_cte, abs(e_y))

        # 5. Update integral with anti-windup (FOLLOW mode only)
        if self._mode == PathFollowingMode.FOLLOW:
            self._integral_ey += e_y * dt
            self._integral_ey = np.clip(self._integral_ey,
                                         -self._integral_limit,
                                         self._integral_limit)

        # 6. ILOS heading command (mode-dependent)
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

        # 7. Check mode transition (ALIGN → FOLLOW)
        heading_error = abs(angle_wrap(chi_d - self._vehicle_yaw))

        if self._mode == PathFollowingMode.ALIGN:
            # ALIGN mode: check if heading alignment is complete
            if heading_error < self._heading_align_threshold:
                # Transition to FOLLOW mode
                self._mode = PathFollowingMode.FOLLOW

        # 8. Estimate curvature for velocity profiling (at lookahead)
        self._current_curvature = self._estimate_curvature(s_lookahead)

        # 9. Compute desired speed (velocity profiler)
        curvature_speed = self._compute_speed(self._current_curvature)

        # 9.5. Lookahead distance-based speed reduction (cruise_speed 기준, 독립적)
        # Reduce speed proportionally to distance from lookahead point
        # This prevents overshooting when lookahead point slows down or stops
        lookahead_dist = np.linalg.norm(p_lookahead - self._vehicle_pos)

        # Simple linear reduction based on actual distance to lookahead
        # Uses cruise_speed as base (independent from curvature-based speed)
        # Example: cruise_speed=1m/s, lookahead_distance=1m
        #          actual_distance=0.9m → lookahead_speed = 1.0 × 0.9 = 0.9 m/s
        #          actual_distance=0.1m → lookahead_speed = 1.0 × 0.1 = 0.1 m/s
        #          actual_distance=0m   → lookahead_speed = 0 m/s (stop)
        speed_factor = min(1.0, lookahead_dist / self._lookahead_distance)
        speed_factor = max(speed_factor, 0.1)  # Minimum 10% speed to maintain control
        lookahead_speed = self._cruise_speed * speed_factor

        # Final speed: minimum of curvature-based and lookahead-based
        # This ensures both constraints are respected without double-penalizing
        desired_speed = min(curvature_speed, lookahead_speed)

        # Store for debugging/logging
        self._current_desired_speed = desired_speed

        # 10. Desired position (lookahead point on path)
        self._desired_pos = p_lookahead
        self._desired_yaw = chi_d

        # 11. Desired velocity (FRD body frame)
        # Surge: desired speed
        # Sway: lateral correction for cross-track error (Lekkas & Fossen 2014)
        # Heave: from path slope
        # Yaw rate: from curvature

        # Lateral correction velocity (sway) - PD control for cross-track error
        # P term: proportional to error (immediate response)
        # D term: damping to reduce oscillations and overshoot
        # Formula: v_lateral = -K_p * e_y - K_d * (de_y/dt)
        # Reference: Fossen (2011) recommends K_d = 0.5~1.0 × K_p for critical damping
        de_y = (e_y - self._prev_ey) / dt if dt > 1e-6 else 0.0
        v_lateral = -self._lateral_gain * e_y - self._lateral_kd * de_y

        # Apply velocity saturation
        v_lateral = np.clip(v_lateral, -self._max_lateral_velocity, self._max_lateral_velocity)

        # Update previous error
        self._prev_ey = e_y

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

        # Depth error correction (feedback) - PD control
        # e_z = desired_z - actual_z (NED: positive = need to go down)
        # In NED, Z is down, so positive error means we need to descend
        # PD control provides better depth tracking with reduced oscillations
        e_z = p_lookahead[2] - self._vehicle_pos[2]
        de_z = (e_z - self._prev_ez) / dt if dt > 1e-6 else 0.0
        w_correction = self._depth_gain * e_z + self._depth_kd * de_z

        # Update previous error
        self._prev_ez = e_z

        # Combined heave velocity
        w_d = w_path + w_correction

        # Apply velocity saturation
        w_d = np.clip(w_d, -self._max_heave_velocity, self._max_heave_velocity)

        # Yaw rate from curvature (kinematic relationship: r = v * κ)
        # For 3D path, use horizontal curvature only
        if tangent_xy_norm > 1e-6:
            # Horizontal speed component (affected by curvature)
            speed_xy = desired_speed * tangent_xy_norm
            r_d = speed_xy * self._current_curvature
        else:
            # Vertical path, no yaw rate
            r_d = 0.0

        # 12. ALIGN mode: Slow 3D path tracking for safe path entry
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
        self._path_parameter_s = 0.0  # Arc-length parameter
        self._integral_ey = 0.0
        self._path_finished = False
        self._max_cte = 0.0
        self._desired_pos = np.zeros(3)
        self._desired_yaw = 0.0
        self._desired_velocity = np.zeros(4)
        self._mode = PathFollowingMode.ALIGN
        self._prev_ey = 0.0
        self._prev_ez = 0.0
