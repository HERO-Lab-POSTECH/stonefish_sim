#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Path-tangent Heading + Depth Guidance for 4DOF Path Following

[축소 §4] 원래 ILOS (Integral Line-of-Sight, Lekkas & Fossen 2014) 가이던스의
cross-track 보정(heading arctan 항 + sway 적분)을 제거하고 path-tangent heading만
출력하도록 축소했다. cross-track 보정은 별도 CascadeController(outer position-P →
inner velocity-PI)가 단일 채널로 전담한다(이중보정 제거). 클래스명 ILOSGuidance는
호환을 위해 유지하나, 잔존 적분 항은 depth 채널(_integral_ez) 하나뿐이다.

Key Features:
- Path-tangent heading 출력 (χ_d = χ_p, FOLLOW 모드)
- Depth integral 보정 유지 (_integral_ez)
- 4DOF output (surge, sway=0, heave, yaw) — sway는 cascade outer로 이관
- Curvature-based velocity profiling (_signed_curvature_filtered → speed profiler)
- Frame: desired_pose (NED world), desired_velocity (FRD body)

Reference:
- Lekkas & Fossen (2014). "Integral LOS Path Following for Curved Paths
  Based on a Monotone Cubic Hermite Spline Parametrization" (축소 전 근거)
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
    """Path-tangent heading + depth guidance for 4DOF UUV (축소된 ILOS).

    현재 heading 법칙 (FOLLOW 모드, §4 축소 후):
        χ_d = χ_p          (path tangent만 — cross-track 보정은 cascade outer)

    [deprecated §4] 축소 전 ILOS 공식 (Lekkas & Fossen 2014, 더 이상 미구현):
        χ_d = χ_p + arctan(-e_y / Δ) - arctan(κ_ILOS * ∫e_y dt / Δ)
    arctan(-e_y/Δ)(cross-track heading)와 ∫e_y(heading 적분) 항을 제거했다.
    e_y는 여전히 계산·로깅되나(self._cross_track_error) heading에는 미반영.

    Where:
        χ_d: Desired heading (yaw)
        χ_p: Path tangent angle = arctan2(tangent[1], tangent[0])
        e_y: Cross-track error (로깅·진단용 — cascade outer가 보정 전담)
        Δ: Lookahead distance
    """

    def __init__(self, lookahead_distance=5.0, cruise_speed=1.0,
                 curvature_gain=6.0, lateral_gain=0.6,
                 # === Auto-calculated if None ===
                 lateral_kd=None, lateral_ki=None,
                 depth_gain=None, depth_kd=None, depth_ki=None,
                 min_speed=None, min_lookahead=None,
                 # === Fixed defaults (rarely changed) ===
                 integral_gain=0.0, integral_limit=5.0,
                 heading_align_threshold=np.deg2rad(10.0),
                 derivative_filter_tau=0.1,
                 max_lateral_velocity=None, max_heave_velocity=None,
                 adaptive_lookahead=True,
                 curvature_preview_enabled=True, curvature_preview_samples=8,
                 curvature_ff_gain=None,
                 sway_ff_gain=0.1):
        """Initialize ILOS guidance.

        Primary Parameters (tune these):
            lookahead_distance: How far ahead to look (m). Rule: 3-5 × cruise_speed
            cruise_speed: Target speed on straights (m/s). 1 knot ≈ 0.5 m/s
            curvature_gain: Curve slowdown strength. Higher = slower in curves
            lateral_gain: Cross-track correction strength (P-gain)

        Auto-calculated (override if needed):
            lateral_kd: D-gain = 0.8 × lateral_gain
            lateral_ki: I-gain = 0.2 × lateral_gain
            depth_gain: Depth P-gain = 1.3 × lateral_gain
            depth_kd: Depth D-gain = 0.6 × depth_gain
            depth_ki: Depth I-gain = 0.12 × depth_gain
            min_speed: Min curve speed = 0.2 × cruise_speed
            min_lookahead: Min lookahead = 0.4 × lookahead_distance
        """
        # === Auto-calculate derived parameters ===
        if lateral_kd is None:
            lateral_kd = 0.8 * lateral_gain
        if lateral_ki is None:
            lateral_ki = 0.2 * lateral_gain
        if depth_gain is None:
            depth_gain = 1.3 * lateral_gain
        if depth_kd is None:
            depth_kd = 0.6 * depth_gain
        if depth_ki is None:
            depth_ki = 0.12 * depth_gain
        if min_speed is None:
            min_speed = 0.2 * cruise_speed
        if min_lookahead is None:
            min_lookahead = 0.7 * lookahead_distance  # Min 70% of base (don't shrink too much)
        if max_lateral_velocity is None:
            max_lateral_velocity = 0.3 * cruise_speed  # 30% of cruise speed
        if max_heave_velocity is None:
            max_heave_velocity = 0.25 * cruise_speed   # 25% of cruise speed
        if curvature_ff_gain is None:
            # Curvature feedforward: anticipate heading change based on upcoming curvature
            # Keep small to not interfere with CTE correction
            # Typical curvature 0.1-0.3 → FF should be ~5-15 degrees max
            curvature_ff_gain = 0.8  # Fixed, not scaled with lookahead

        # ILOS parameters
        self._lookahead_distance_base = lookahead_distance  # Base lookahead
        self._lookahead_distance = lookahead_distance       # Effective (may be adaptive)
        self._integral_gain = integral_gain
        self._integral_limit = integral_limit

        # Adaptive lookahead parameters (for curve overshooting fix)
        self._adaptive_lookahead = adaptive_lookahead
        self._min_lookahead = min_lookahead
        self._lookahead_filter_tau = 1.0  # Lookahead smoothing (s)
        self._lookahead_filtered = lookahead_distance  # Filtered lookahead
        self._curvature_for_lookahead_filtered = 0.0  # Filtered curvature for adaptive lookahead

        # Curvature preview parameters (for early curve detection)
        self._curvature_preview_enabled = curvature_preview_enabled
        self._curvature_preview_samples = curvature_preview_samples
        self._curvature_ff_gain = curvature_ff_gain  # Heading feedforward from curvature

        # [P6] 곡률 sway feedforward 게인 (≈m/Kp_inner=20.131/200≈0.1 s).
        # v_sway_ff = -sway_ff_gain · v² · κ_signed 로 코너 원심력 선제 상쇄.
        self._sway_ff_gain = sway_ff_gain

        # Velocity profiling parameters
        self._cruise_speed = cruise_speed
        self._min_speed = min_speed
        self._curvature_gain = curvature_gain
        self._lateral_gain = lateral_gain
        self._depth_gain = depth_gain

        # PID control parameters
        self._lateral_kd = lateral_kd
        self._depth_kd = depth_kd
        self._lateral_ki = lateral_ki       # Phase 1.2: Lateral I term
        self._depth_ki = depth_ki           # Phase 1.1: Depth I term
        self._max_lateral_velocity = max_lateral_velocity
        self._max_heave_velocity = max_heave_velocity

        # Previous errors for derivative calculation
        self._prev_ey = 0.0        # [deprecated §4: cross-track 제거됨, 초기화만 잔존]
        self._prev_ez = 0.0

        # Phase 1: Integral states for lateral/depth PID
        self._integral_ey_lateral = 0.0  # [deprecated §4: cross-track 제거됨, 초기화만 잔존]
        self._integral_ez = 0.0          # Depth I term

        # Phase 1.3: Low-pass filter for derivative noise reduction
        self._filter_tau = derivative_filter_tau  # Time constant (s)
        self._de_y_filtered = 0.0  # [deprecated §4: cross-track 제거됨, 초기화만 잔존]
        self._de_z_filtered = 0.0  # Filtered depth derivative

        # Curvature filtering (prevent heading/lookahead oscillation)
        # Use asymmetric filter: fast response when curvature decreases
        self._curvature_filter_tau_up = 0.3    # Slow filter when curvature increases
        self._curvature_filter_tau_down = 0.05  # Fast filter when curvature decreases
        self._signed_curvature_filtered = 0.0  # For heading feedforward

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
        self._integral_ey = 0.0  # [deprecated §4: cross-track 제거됨, 초기화만 잔존]

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


    def _estimate_curvature(self, s, use_3d=True):
        """Estimate curvature at arc-length parameter s using 3-point method.

        Args:
            s: Arc-length parameter (m)
            use_3d: If True, compute full 3D curvature; if False, horizontal only

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

        # Vectors (tangent approximations)
        v1 = p_curr - p_prev
        v2 = p_next - p_curr

        # Phase 3.2: Choose 2D or 3D curvature
        if not use_3d:
            # 2D curvature (horizontal plane only)
            v1 = v1[:2]  # [x, y] only
            v2 = v2[:2]

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

    def _estimate_signed_curvature(self, s):
        """Estimate signed curvature at arc-length parameter s.

        Returns signed curvature where:
            Positive = left turn (counterclockwise in NED top-down view)
            Negative = right turn (clockwise in NED top-down view)

        Args:
            s: Arc-length parameter (m)

        Returns:
            float: Signed curvature (1/m)
        """
        if self._path_poses is None or len(self._path_poses) < 3:
            return 0.0

        ds = 0.1
        s_prev = max(0.0, s - ds)
        s_curr = s
        s_next = min(self._total_path_length, s + ds)

        p_prev = self._interpolate_from_parameter(s_prev)
        p_curr = self._interpolate_from_parameter(s_curr)
        p_next = self._interpolate_from_parameter(s_next)

        v1 = p_curr - p_prev
        v2 = p_next - p_curr

        l1 = np.linalg.norm(v1[:2])
        l2 = np.linalg.norm(v2[:2])

        if l1 < 1e-9 or l2 < 1e-9:
            return 0.0

        # Cross product z-component determines turn direction
        # In NED: positive z is down, so cross product sign is flipped
        cross_z = v1[0] * v2[1] - v1[1] * v2[0]  # x1*y2 - y1*x2
        sign = 1.0 if cross_z > 0 else -1.0

        # Curvature magnitude
        cos_angle = np.dot(v1[:2], v2[:2]) / (l1 * l2)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.arccos(cos_angle)

        L_avg = (l1 + l2) / 2.0
        curvature = 2.0 * np.sin(angle / 2.0) / L_avg if L_avg > 1e-9 else 0.0

        return sign * curvature

    def _estimate_curvature_3d_frenet(self, s):
        """Estimate 3D curvature using Frenet-Serret formula (Phase 3.2).

        Uses cross product method for accurate 3D curvature:
            κ = |T' × T''| / |T'|³

        This is more accurate than angle-based method for complex 3D paths.

        Args:
            s: Arc-length parameter (m)

        Returns:
            float: 3D curvature (1/m)
        """
        if self._path_poses is None or len(self._path_poses) < 3:
            return 0.0

        # Sample 4 points for second derivative estimation
        ds = 0.1  # 10cm spacing
        s_m1 = max(0.0, s - ds)
        s_0 = s
        s_p1 = min(self._total_path_length, s + ds)
        s_p2 = min(self._total_path_length, s + 2 * ds)

        p_m1 = self._interpolate_from_parameter(s_m1)
        p_0 = self._interpolate_from_parameter(s_0)
        p_p1 = self._interpolate_from_parameter(s_p1)
        p_p2 = self._interpolate_from_parameter(s_p2)

        # First derivatives (tangent vectors)
        T1 = (p_0 - p_m1) / ds if s > ds else (p_p1 - p_0) / ds
        T2 = (p_p1 - p_0) / ds
        T3 = (p_p2 - p_p1) / ds if s_p2 > s_p1 else T2

        # Second derivatives (curvature vectors)
        T_prime = (T2 - T1) / ds
        T_double_prime = (T3 - T2) / ds

        # Cross product magnitude
        cross = np.cross(T_prime, T_double_prime)
        cross_norm = np.linalg.norm(cross)

        # Curvature κ = |T' × T''| / |T'|³
        T_prime_norm = np.linalg.norm(T_prime)
        if T_prime_norm < 1e-9:
            return 0.0

        curvature = cross_norm / (T_prime_norm ** 3 + 1e-9)

        return curvature

    def _estimate_max_curvature_preview(self, s_start, s_end):
        """Estimate maximum curvature between s_start and s_end (curvature preview).

        Samples curvature at multiple points to detect curves early.
        This allows the vehicle to slow down BEFORE entering a curve.

        Args:
            s_start: Start arc-length (current position)
            s_end: End arc-length (lookahead position)

        Returns:
            float: Maximum curvature in the preview window (1/m)
        """
        if not self._curvature_preview_enabled:
            return self._estimate_curvature(s_end)

        if self._path_poses is None or len(self._path_poses) < 3:
            return 0.0

        # Sample curvature at multiple points
        max_curvature = 0.0
        n_samples = max(2, self._curvature_preview_samples)

        for i in range(n_samples):
            # Sample from current position to lookahead
            alpha = i / (n_samples - 1)
            s_sample = s_start + alpha * (s_end - s_start)
            s_sample = np.clip(s_sample, 0.0, self._total_path_length)

            curvature = self._estimate_curvature(s_sample)
            max_curvature = max(max_curvature, curvature)

        return max_curvature

    def _compute_lookahead_geometry(self, dt):
        """Steps 0-3: adaptive lookahead distance, lookahead point, path tangent.

        Returns:
            tuple: (p_lookahead, p_current, tangent, tangent_norm, chi_p)
        """
        # 0. Adaptive Lookahead (curvature-based, double-filtered for smoothness)
        # High curvature = shorter lookahead for tighter control
        # Low curvature = longer lookahead for stability
        if self._adaptive_lookahead:
            # Use max curvature over a window ahead
            preview_dist = self._lookahead_distance_base
            s_preview_end = min(self._path_parameter_s + preview_dist, self._total_path_length)
            curvature_raw = self._estimate_max_curvature_preview(
                self._path_parameter_s, s_preview_end
            )

            # Step 1: Filter the curvature itself (prevents sudden jumps)
            # Use asymmetric filter: fast increase (react to curves), slow decrease (smooth exit)
            tau_up = 0.3    # Fast reaction to upcoming curves
            tau_down = 1.5  # Slow return after curves (prevents oscillation)
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

            # Step 3: Filter the lookahead distance (additional smoothing)
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

        return p_lookahead, p_current, tangent, tangent_norm, chi_p

    def _compute_heading_command(self, chi_p, dt):
        """Steps 4-6: cross-track error, ILOS integral update, heading command.

        Returns:
            tuple: (chi_d, e_y)
        """
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

        # 5. [축소 §4] _integral_ey 갱신 제거: cross-track heading 채널 제거됨.
        # cascade outer가 e_y 보정을 전담하므로 ILOS heading 적분은 불필요.

        # 6. ILOS heading command (mode-dependent)
        if self._mode == PathFollowingMode.ALIGN:
            # ALIGN mode: Use fixed path start tangent (no CTE correction)
            # This provides stable heading reference for initial alignment
            tangent_start = self._get_path_tangent(0)  # Path start tangent
            chi_d = np.arctan2(tangent_start[1], tangent_start[0])
        else:
            # FOLLOW mode: path-tangent heading (§4 축소 — CTE 보정·heading 적분 제거).
            # 아래 곡률 필터링은 heading FF가 아니라 _signed_curvature_filtered를
            # 갱신하기 위함이다 — 이 상태는 r_d·speed profiler가 소비한다(미dead).
            signed_curvature_raw = self._estimate_signed_curvature(
                min(self._path_parameter_s + self._lookahead_distance, self._total_path_length)
            )
            # Asymmetric low-pass filter: slow up, fast down
            # This prevents oscillation on curve entry, but allows quick recovery on exit
            if abs(signed_curvature_raw) > abs(self._signed_curvature_filtered):
                # Curvature increasing → slow filter (smooth entry)
                tau = self._curvature_filter_tau_up
            else:
                # Curvature decreasing → fast filter (quick exit recovery)
                tau = self._curvature_filter_tau_down

            alpha_curv = dt / (tau + dt)
            self._signed_curvature_filtered = (
                alpha_curv * signed_curvature_raw +
                (1 - alpha_curv) * self._signed_curvature_filtered
            )
            # [deprecated §4] curvature_ff = _curvature_ff_gain * _signed_curvature_filtered
            # 는 heading FF였으나 chi_d=chi_p 축소로 미소비. _curvature_ff_gain 파라미터는
            # 생성자 시그니처 호환 위해 유지(미사용). 제거는 호출부·YAML 영향이라 P5 범위 밖.

            # [축소 §4] cross-track heading 채널 제거: 순수 path-tangent.
            # e_y 보정은 cascade outer가 전담 → ILOS heading은 χ_p만 출력.
            chi_d = chi_p

        chi_d = angle_wrap(chi_d)

        return chi_d, e_y

    def _compute_desired_speed(self, p_lookahead):
        """Steps 8-9.5: curvature preview + velocity profiler → desired speed.

        Returns:
            float: desired_speed
        """
        # 8. Estimate curvature for velocity profiling (with extended preview)
        # Must look far enough ahead to slow down before curves
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

        return desired_speed

    def _compute_body_velocities(self, e_y, tangent, p_lookahead, desired_speed, dt):
        """Step 11: FRD body-frame velocities (sway, heave, yaw rate).

        Returns:
            tuple: (v_lateral, w_d, r_d)
        """
        # 11. Desired velocity (FRD body frame)
        # Surge: desired speed
        # Sway: lateral correction for cross-track error (Lekkas & Fossen 2014)
        # Heave: from path slope
        # Yaw rate: from curvature

        # [P6] 곡률 sway feedforward: 코너 원심력 선제 상쇄 (FOLLOW만).
        # v_sway_ff = +sway_ff_gain · v² · κ_signed.
        # ※구현 부호 관례: _estimate_signed_curvature는 우회전→κ>0(+), 좌회전→κ<0(-).
        #   (docstring L503-505는 반대로 적혀있으나 구현이 SSOT). 우회전 κ>0 → +sway=오른쪽=안쪽 ✓.
        # feedback(e_y 보정)은 cascade outer가 전담 — 여기는 예측만(이중보정 아님).
        if self._mode == PathFollowingMode.FOLLOW:
            v_lateral = (self._sway_ff_gain * desired_speed * desired_speed
                         * self._signed_curvature_filtered)
        else:
            v_lateral = 0.0

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

        # Depth error correction (feedback) - PID control (Phase 1.1)
        # e_z = desired_z - actual_z (NED: positive = need to go down)
        # In NED, Z is down, so positive error means we need to descend
        # PID control: I term eliminates steady-state error from vertical currents
        # Reference: Fossen (2011), Lekkas & Fossen (2014)
        e_z = p_lookahead[2] - self._vehicle_pos[2]

        # Derivative with low-pass filter (Phase 1.3)
        de_z_raw = (e_z - self._prev_ez) / dt if dt > 1e-6 else 0.0
        alpha_z = dt / (self._filter_tau + dt)  # Same filter time constant
        self._de_z_filtered = alpha_z * de_z_raw + (1 - alpha_z) * self._de_z_filtered

        # Integral with anti-windup (Phase 1.1) - FOLLOW mode only
        if self._mode == PathFollowingMode.FOLLOW:
            self._integral_ez += e_z * dt
            self._integral_ez = np.clip(
                self._integral_ez,
                -self._integral_limit,
                self._integral_limit
            )

        # PID control
        w_correction = (
            self._depth_gain * e_z
            + self._depth_ki * self._integral_ez
            + self._depth_kd * self._de_z_filtered
        )

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

        return v_lateral, w_d, r_d

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

        # 0-3. Lookahead geometry (adaptive lookahead, lookahead point, path tangent)
        p_lookahead, p_current, tangent, tangent_norm, chi_p = (
            self._compute_lookahead_geometry(dt)
        )

        # 4-6. Cross-track error, integral update, ILOS heading command
        chi_d, e_y = self._compute_heading_command(chi_p, dt)

        # 7. Check mode transition (ALIGN → FOLLOW)
        heading_error = abs(angle_wrap(chi_d - self._vehicle_yaw))

        if self._mode == PathFollowingMode.ALIGN:
            # ALIGN mode: check if heading alignment is complete
            if heading_error < self._heading_align_threshold:
                # Transition to FOLLOW mode
                self._mode = PathFollowingMode.FOLLOW

        # 8-9.5. Curvature preview + velocity profiler → desired speed
        desired_speed = self._compute_desired_speed(p_lookahead)

        # 10. Desired position (lookahead point on path)
        self._desired_pos = p_lookahead
        self._desired_yaw = chi_d

        # 11. Body-frame velocities (sway, heave, yaw rate)
        v_lateral, w_d, r_d = self._compute_body_velocities(
            e_y, tangent, p_lookahead, desired_speed, dt
        )

        # 12. ALIGN mode: Slow 3D path tracking for safe path entry
        if self._mode == PathFollowingMode.ALIGN:
            # Track lookahead point with slow speed (smooth 3D approach)
            # This allows gradual depth change along path (NOT vertical drop)
            # p_lookahead already assigned to _desired_pos (line 428), so keep it
            desired_speed = 0.3  # Slow approach speed (m/s) - literature-based safe entry
            # [축소] v_lateral은 이미 0 (sway 채널 제거). w_d/r_d는 유지.
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
        self._integral_ey = 0.0       # [deprecated §4: cross-track 제거됨, 초기화만 잔존]
        self._path_finished = False
        self._max_cte = 0.0
        self._desired_pos = np.zeros(3)
        self._desired_yaw = 0.0
        self._desired_velocity = np.zeros(4)
        self._mode = PathFollowingMode.ALIGN
        self._prev_ey = 0.0            # [deprecated §4: cross-track 제거됨, 초기화만 잔존]
        self._prev_ez = 0.0
        # Phase 1: Reset lateral/depth PID integrals and filters
        self._integral_ey_lateral = 0.0  # [deprecated §4: cross-track 제거됨, 초기화만 잔존]
        self._integral_ez = 0.0
        self._de_y_filtered = 0.0        # [deprecated §4: cross-track 제거됨, 초기화만 잔존]
        self._de_z_filtered = 0.0
        # Adaptive lookahead filters
        self._lookahead_filtered = self._lookahead_distance_base
        self._curvature_for_lookahead_filtered = 0.0
