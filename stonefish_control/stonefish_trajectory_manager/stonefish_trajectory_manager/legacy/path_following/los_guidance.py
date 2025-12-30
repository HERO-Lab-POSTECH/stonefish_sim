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
Line-of-Sight (LOS) Guidance for Continuous Path Following

Implementation of LOS guidance law with virtual vehicle advancement.
Based on Fossen (2011) and Lekkas & Fossen (2014).

Key Features:
- Virtual vehicle advancement along path (time integration)
- Closest point search (local window around virtual vehicle)
- Cross-track error minimization
- Curvature-based velocity profiling

Reference:
- Fossen, T. I. (2011). "Handbook of Marine Craft Hydrodynamics and Motion Control"
- Lekkas & Fossen (2014). "Line-of-Sight Guidance for Path Following of Marine Vehicles"
- Lekkas & Fossen (2014). "Minimization of Cross-track and Along-track Errors for
  Path Tracking of Marine Underactuated Vehicles"
"""

import numpy as np
import time
from transforms3d.euler import quat2euler


class LOSGuidance:
    """LOS guidance law with virtual vehicle advancement.

    This implementation follows Lekkas & Fossen (2014):
    - Virtual vehicle navigates on the desired path
    - Path parameter s(t) advances with time integration
    - Closest point search is local (near virtual vehicle)

    LOS formula (Fossen 2011, Eq. 10.12):
        χ_d = χ_p - arctan(y_e / Δ)
    """

    def __init__(self, lookahead_distance=2.5,
                 robot_max_speed=1.0, max_lateral_accel=0.3,
                 min_speed_factor=0.3, path_interpolator=None):
        """Initialize LOS guidance.

        Args:
            lookahead_distance: Lookahead distance (m), typically 2-5m for ROV
            robot_max_speed: Maximum robot speed (m/s)
            max_lateral_accel: Maximum lateral acceleration (m/s²)
            min_speed_factor: Minimum speed as factor of max (0-1)
            path_interpolator: Not used (kept for compatibility)
        """
        # LOS parameters
        self._lookahead_distance = lookahead_distance

        # Velocity parameters
        self._robot_max_speed = robot_max_speed
        self._max_lateral_accel = max_lateral_accel
        self._min_speed_factor = min_speed_factor
        self._desired_speed = robot_max_speed

        # Vehicle state
        self._vehicle_pos = np.zeros(3)
        self._vehicle_quat = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
        self._vehicle_yaw = 0.0
        self._vehicle_velocity = np.zeros(3)

        # Path management (dense array of poses)
        self._path_poses = None  # np.array, shape (N, 3)
        self._path_finished = False

        # Virtual vehicle (Lekkas & Fossen 2014)
        self._path_parameter = 0.0  # s ∈ [0, 1] for entire path
        self._virtual_vehicle_idx = 0  # Index on path where virtual vehicle is
        self._total_path_length = 0.0  # Total arc length of path

        # Closest point (for cross-track error calculation)
        self._closest_point_idx = 0

        # Guidance outputs
        self._desired_pos = np.zeros(3)
        self._desired_yaw = 0.0
        self._cross_track_error = 0.0
        self._path_progress = 0.0

        # Path tangent vector (for velocity reference)
        self._path_tangent_vec = np.array([1.0, 0.0, 0.0])

        # Curvature estimation
        self._current_curvature = 0.0

        # Heading rate limiter state
        self._last_desired_yaw = None

        # Statistics
        self._max_cross_track_error = 0.0

    def set_path(self, path_poses):
        """Set path for following (dense array of poses from path generator).

        Automatically resets guidance state to start from beginning.

        Args:
            path_poses: List or array of [x, y, z] positions
                       Example: [[0,0,2], [0.1,0,1.99], [0.2,0,1.98], ...]
        """
        self._path_poses = np.array(path_poses)

        if len(self._path_poses) < 3:
            raise ValueError(f'Path must have at least 3 points, got {len(self._path_poses)}')

        # Calculate total path arc length
        self._total_path_length = 0.0
        for i in range(len(self._path_poses) - 1):
            segment_dist = np.linalg.norm(self._path_poses[i + 1] - self._path_poses[i])
            self._total_path_length += segment_dist

        # Reset all state variables to start fresh
        self._path_parameter = 0.0  # Virtual vehicle at path start
        self._virtual_vehicle_idx = 0
        self._closest_point_idx = 0
        self._path_finished = False
        self._path_progress = 0.0
        self._last_desired_yaw = None
        self._max_cross_track_error = 0.0

    def update_vehicle_state(self, position, orientation_quat, linear_velocity):
        """Update vehicle state from odometry.

        Args:
            position: np.array([x, y, z]) - world frame (NED)
            orientation_quat: np.array([w, x, y, z])
            linear_velocity: np.array([vx, vy, vz]) - BODY frame (FRD)
        """
        self._vehicle_pos = np.array(position)
        self._vehicle_quat = np.array(orientation_quat)
        self._vehicle_velocity = np.array(linear_velocity)  # BODY frame!

        # Extract yaw
        _, _, self._vehicle_yaw = quat2euler(self._vehicle_quat, 'sxyz')

    def update(self, dt):
        """Update LOS guidance law with virtual vehicle advancement.

        Args:
            dt: Time step (seconds)

        Returns:
            bool: True if successful
        """
        if self._path_poses is None or len(self._path_poses) == 0:
            return False

        if self._path_finished:
            # Hold final position
            self._desired_pos = self._path_poses[-1]
            self._desired_speed = 0.0
            return True

        # 1. Advance virtual vehicle (Lekkas & Fossen 2014 + Geometric GLOS 2024)
        # Ref: "Minimization of cross-track and along-track errors" (Lekkas & Fossen 2014)
        # Ref: "Geometric line-of-sight guidance law" (2024)
        #
        # Path parameter update law:
        #   ϖ̇ = (Ud cos χr + ks·s) / ||ṗd||
        #
        # Components:
        #   - Ud cos χr: Actual robot speed projected onto path
        #   - ks·s: Along-track error correction (prevents virtual from running away)
        #   - ||ṗd||: Path derivative (approximately total_path_length)

        # Transform body velocity to world frame
        from transforms3d.quaternions import quat2mat
        R_BtoW = quat2mat(self._vehicle_quat)  # Body to World
        velocity_world = R_BtoW @ self._vehicle_velocity

        # Get path tangent at current virtual vehicle position
        if hasattr(self, '_path_tangent_vec'):
            path_tangent = self._path_tangent_vec
        else:
            # First iteration: use approximate tangent
            if self._virtual_vehicle_idx < len(self._path_poses) - 1:
                tangent_vec = self._path_poses[self._virtual_vehicle_idx + 1] - self._path_poses[self._virtual_vehicle_idx]
                path_tangent = tangent_vec / (np.linalg.norm(tangent_vec) + 1e-6)
            else:
                path_tangent = np.array([1.0, 0.0, 0.0])

        # Project velocity onto path tangent (only forward progress counts!)
        speed_along_path = np.dot(velocity_world, path_tangent)
        actual_robot_speed = max(0.0, speed_along_path)  # Only positive (no backward)

        # Along-track error correction (Lekkas & Fossen 2014)
        # Prevents virtual vehicle from running too far ahead of robot
        # Ref: ϖ̇ = (U·cos(ψ-πh) + ks·xe) / ||ṗd||
        #      where xe = along-track error (virtual ahead → positive)
        #      ks > 0 → positive xe slows virtual (negative contribution to ṡ)
        if self._total_path_length > 1e-6:
            # Calculate along-track distance
            virtual_arc_length = self._path_parameter * self._total_path_length

            # Robot's arc length (approximate from closest point)
            closest_arc_length = (self._closest_point_idx / (len(self._path_poses) - 1)) * self._total_path_length

            # Along-track error
            # Definition: Robot - Virtual (negative if virtual ahead, intuitive for feedback)
            along_track_error = closest_arc_length - virtual_arc_length

            # Correction gain (Lekkas & Fossen 2014: ks > 0)
            k_along = 0.5  # Feedback strength (increased from 0.3)

            # Path parameter update (actual speed + along-track correction)
            # When virtual ahead: along_track_error < 0 → slows virtual ✓
            # When robot ahead: along_track_error > 0 → speeds virtual ✓
            s_dot = (actual_robot_speed + k_along * along_track_error) / self._total_path_length
            s_dot = max(0.0, s_dot)  # No backward motion

            self._path_parameter += s_dot * dt
            self._path_parameter = np.clip(self._path_parameter, 0.0, 1.0)
            self._virtual_vehicle_idx = int(self._path_parameter * (len(self._path_poses) - 1))

        # 2. Desired position = Virtual vehicle position
        self._desired_pos = self._path_poses[self._virtual_vehicle_idx].copy()

        # 3. Find closest point on path (local search around virtual vehicle)
        # Use local search to avoid cross-path jumping (Lekkas & Fossen 2014)
        self._closest_point_idx = self._find_closest_point_local(self._virtual_vehicle_idx)

        # 3b. STRICT MONOTONIC constraint
        # Index can ONLY advance or stay (NEVER decrease)
        # This prevents oscillation between waypoints (e.g., jumping between index 10 and 100)
        if hasattr(self, '_prev_closest_idx'):
            self._closest_point_idx = max(self._closest_point_idx, self._prev_closest_idx)

        # Save for next iteration
        self._prev_closest_idx = self._closest_point_idx

        # 4. Compute path tangent at closest point (for cross-track error)
        path_tangent_vec, path_tangent_angle = self._compute_path_tangent(
            self._closest_point_idx
        )
        self._path_tangent_vec = path_tangent_vec

        # 5. Compute cross-track error
        self._cross_track_error = self._compute_cross_track_error(
            self._closest_point_idx, path_tangent_vec
        )

        # Update statistics
        self._max_cross_track_error = max(
            self._max_cross_track_error,
            abs(self._cross_track_error)
        )

        # 6. LOS angle (Fossen 2011, Eq. 10.12)
        chi_los = np.arctan2(-self._cross_track_error, self._lookahead_distance)

        # 7. Desired heading
        raw_desired_yaw = path_tangent_angle + chi_los

        # 8. Smooth angle wrapping
        angle_diff = self._normalize_angle(raw_desired_yaw - self._vehicle_yaw)
        smooth_desired_yaw = self._vehicle_yaw + angle_diff

        # 9. Apply heading rate limiter (60°/s)
        if self._last_desired_yaw is None:
            self._last_desired_yaw = self._vehicle_yaw

        max_heading_rate = np.deg2rad(60.0) * dt
        heading_change = self._normalize_angle(smooth_desired_yaw - self._last_desired_yaw)

        if abs(heading_change) > max_heading_rate:
            heading_change = np.sign(heading_change) * max_heading_rate

        self._desired_yaw = self._last_desired_yaw + heading_change

        # Normalize desired_yaw to [-pi, pi]
        self._desired_yaw = self._normalize_angle(self._desired_yaw)

        self._last_desired_yaw = self._desired_yaw

        # 10. Velocity profiling (curvature-based speed control)
        # Use virtual vehicle position for curvature estimation
        self._current_curvature = self._compute_path_curvature(self._virtual_vehicle_idx)
        self._desired_speed = self._compute_safe_speed_from_curvature(
            self._current_curvature
        )

        # Velocity profiling is active (logging handled by path_following_node)

        # 11. Path progress = path parameter
        self._path_progress = self._path_parameter

        # 12. Mission complete check
        # Path parameter >= 1.0 → path is complete
        # Distance check removed for optimizer efficiency
        # Cost function will penalize if robot is far from goal
        distance_to_goal = np.linalg.norm(self._vehicle_pos - self._path_poses[-1])

        # Debug: Log progress near completion (throttled)
        if self._path_parameter >= 0.9 and not self._path_finished:
            if not hasattr(self, '_last_progress_log_time'):
                self._last_progress_log_time = 0.0
            current_time = time.time()
            if current_time - self._last_progress_log_time > 1.0:
                import logging
                logger = logging.getLogger('los_guidance')
                logger.info(
                    f'[Progress] param:{self._path_parameter:.3f} (need:≥1.00) | '
                    f'dist:{distance_to_goal:.3f}m'
                )
                self._last_progress_log_time = current_time

        if self._path_parameter >= 1.0:
            self._path_finished = True
            # Path completion logged by path_following_node

        return True

    def _find_closest_point_local(self, virtual_vehicle_idx):
        """Find closest point on path (local search around virtual vehicle).

        CRITICAL FIX for closed-loop paths:
        - Only search FORWARD from virtual vehicle (prevent backward jumps)
        - For circular paths where start ≈ end, prevents wrapping to start

        Args:
            virtual_vehicle_idx: Index where virtual vehicle is located

        Returns:
            int: Index of closest point
        """
        # Forward-only search window from virtual vehicle
        # Prevents backward jumps to path start in closed-loop paths
        window_size = 50  # Forward only (was ±100)

        # Search FROM virtual vehicle forward (no backward!)
        search_start = virtual_vehicle_idx
        search_end = min(len(self._path_poses), virtual_vehicle_idx + window_size)

        # Handle edge case: near path end with small window
        if search_end - search_start < 10 and virtual_vehicle_idx > window_size:
            # Expand backward only if very close to end
            search_start = max(0, search_end - window_size)

        # Compute distances within search window
        search_poses = self._path_poses[search_start:search_end]
        distances = np.linalg.norm(search_poses - self._vehicle_pos, axis=1)

        # Find minimum distance index (relative to search window)
        relative_idx = np.argmin(distances)

        # Convert to absolute index
        closest_idx = search_start + relative_idx

        return closest_idx

    def _compute_path_tangent(self, idx):
        """Compute path tangent at index.

        Args:
            idx: Path point index

        Returns:
            tuple: (tangent_vec, tangent_angle)
                   tangent_vec: Unit vector in path direction
                   tangent_angle: Angle in radians
        """
        # Boundary handling
        if idx == 0:
            tangent_vec = self._path_poses[1] - self._path_poses[0]
        elif idx >= len(self._path_poses) - 1:
            tangent_vec = self._path_poses[-1] - self._path_poses[-2]
        else:
            # Central difference for better accuracy
            tangent_vec = self._path_poses[idx + 1] - self._path_poses[idx - 1]

        # Normalize
        tangent_length = np.linalg.norm(tangent_vec)
        if tangent_length < 1e-6:
            tangent_vec = np.array([1.0, 0.0, 0.0])
        else:
            tangent_vec = tangent_vec / tangent_length

        # Compute angle
        tangent_angle = np.arctan2(tangent_vec[1], tangent_vec[0])

        return tangent_vec, tangent_angle

    def _compute_cross_track_error(self, closest_idx, path_tangent_vec):
        """Compute signed cross-track error.

        Args:
            closest_idx: Index of closest point on path
            path_tangent_vec: Path tangent unit vector at closest point

        Returns:
            float: Signed cross-track error (m), positive = right of path
        """
        closest_pos = self._path_poses[closest_idx]
        rel_pos = self._vehicle_pos - closest_pos

        # Project onto path tangent (along-track component)
        along_track = np.dot(rel_pos, path_tangent_vec)
        along_track_vec = along_track * path_tangent_vec

        # Cross-track vector (perpendicular component)
        cross_track_vec = rel_pos - along_track_vec

        # Signed distance using cross product
        # Positive = right of path, Negative = left of path
        cross_product = np.cross(path_tangent_vec, rel_pos)
        sign = 1 if cross_product[2] > 0 else -1

        cross_track_error = sign * np.linalg.norm(cross_track_vec)

        return cross_track_error

    def _compute_path_curvature(self, idx):
        """Compute local curvature at path point using 3-point Menger curvature.

        Args:
            idx: Path point index

        Returns:
            float: Curvature (1/m), 0 if straight or at boundary
        """
        # Boundary check
        if idx == 0 or idx >= len(self._path_poses) - 1:
            return 0.0

        # Get 3 consecutive points
        p1 = self._path_poses[idx - 1]
        p2 = self._path_poses[idx]
        p3 = self._path_poses[idx + 1]

        # Calculate vectors
        v1 = p2 - p1
        v2 = p3 - p1
        v3 = p3 - p2

        # Side lengths
        a = np.linalg.norm(v1)  # |P1P2|
        b = np.linalg.norm(v3)  # |P2P3|
        c = np.linalg.norm(v2)  # |P1P3|

        # Check for degenerate case
        if a < 1e-6 or b < 1e-6 or c < 1e-6:
            return 0.0

        # Triangle area using cross product
        cross_prod = np.cross(v1, v2)
        area = 0.5 * np.linalg.norm(cross_prod)

        # Menger curvature: κ = 4A / (abc)
        curvature = 4.0 * area / (a * b * c)

        # Clamp to reasonable values (min radius = 0.1m)
        curvature = min(curvature, 10.0)

        return curvature

    def _compute_safe_speed_from_curvature(self, curvature):
        """Compute safe speed based on path curvature and lateral acceleration constraint.

        Uses lateral acceleration constraint: a_lat = v² × κ
        Safe speed: v_safe = sqrt(a_lat_max / κ)

        Args:
            curvature: Path curvature (1/m)

        Returns:
            float: Safe speed (m/s)
        """
        # Straight path: use maximum speed
        if curvature < 1e-6:
            return self._robot_max_speed

        # Compute speed limit from lateral acceleration constraint
        v_lateral = np.sqrt(self._max_lateral_accel / curvature)

        # Apply minimum speed constraint
        v_min = self._robot_max_speed * self._min_speed_factor

        # Final speed: max(v_min, min(v_lateral, robot_max_speed))
        v_safe = max(v_min, min(v_lateral, self._robot_max_speed))

        return v_safe

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi].

        Args:
            angle: Angle in radians

        Returns:
            float: Normalized angle in [-pi, pi]
        """
        return np.arctan2(np.sin(angle), np.cos(angle))

    def get_guidance_command(self):
        """Get current guidance command.

        Returns:
            dict: {
                'desired_pos': np.array([x, y, z]),  # Virtual vehicle position
                'desired_yaw': float (rad),          # LOS heading
                'desired_speed': float (m/s),        # Target forward speed
                'path_tangent_vec': np.array([x, y, z]),  # Path tangent direction
                'cross_track_error': float (m),      # Perpendicular distance from path
                'path_progress': float (0-1)         # Overall path completion
            }
        """
        return {
            'desired_pos': self._desired_pos.copy(),
            'desired_yaw': self._desired_yaw,
            'desired_speed': self._desired_speed,
            'path_tangent_vec': self._path_tangent_vec.copy(),
            'cross_track_error': self._cross_track_error,
            'path_progress': self._path_progress
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
            float: Maximum cross-track error (m)
        """
        return self._max_cross_track_error

    def reset_statistics(self):
        """Reset tracking statistics."""
        self._max_cross_track_error = 0.0

    def reset(self):
        """Reset guidance state for restarting path following."""
        self._path_parameter = 0.0
        self._virtual_vehicle_idx = 0
        self._closest_point_idx = 0
        self._path_finished = False
        self._path_progress = 0.0
        self._last_desired_yaw = None
        self._max_cross_track_error = 0.0

        # Reset monotonic constraint state
        if hasattr(self, '_prev_closest_idx'):
            delattr(self, '_prev_closest_idx')

        # Reset initialization flag to trigger global search on next update
        if hasattr(self, '_initialized_closest'):
            delattr(self, '_initialized_closest')
