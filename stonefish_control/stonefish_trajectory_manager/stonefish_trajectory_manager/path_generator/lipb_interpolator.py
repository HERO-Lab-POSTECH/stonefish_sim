# SPDX-FileCopyrightText: 2016-2019 The UUV Simulator Authors
#
# SPDX-License-Identifier: GPL-3.0-or-later
from copy import deepcopy
from scipy.interpolate import splrep, splev, interp1d
import numpy as np
from visualization_msgs.msg import MarkerArray
from transforms3d.quaternions import axangle2quat, qmult

from ..common.waypoint import Waypoint
from ..common.waypoint_set import WaypointSet
from ..common.trajectory_point import TrajectoryPoint
from .line_segment import LineSegment
from .bezier_curve import BezierCurve
from .path_generator import PathGenerator


class LIPBInterpolator(PathGenerator):
    """
    Linear interpolator with polynomial blends.

    This interpolator creates straight line segments between waypoints
    and uses 5th-order Bezier curves to smoothly blend the corners
    near each waypoint. This results in mostly straight paths with
    smooth turns at waypoints.

    !!! note

        Biagiotti, Luigi, and Claudio Melchiorri. Trajectory planning for
        automatic machines and robots. Springer Science & Business Media, 2008.
    """
    LABEL = 'lipb'

    def __init__(self):
        super(LIPBInterpolator, self).__init__()

        self._radius = 5  # Default radius for corner blending
        # Set of interpolation functions for each degree of freedom
        # The heading function interpolates the given heading offset and its
        # value is added to the heading computed from the trajectory
        self._interp_fcns = dict(pos=None,
                                 heading=None)
        self._heading_spline = None

        # Velocity profiler for curvature-based speed control
        self._velocity_profiler = None
        self._use_velocity_profiler = False
        self._total_path_length = 0.0

    def init_interpolator(self):
        """Initialize the interpolator. To have the path segments generated,
        `init_waypoints()` must be called beforehand by providing a set of
        waypoints as `WaypointSet` type.

        > *Returns*

        `True` if the path segments were successfully generated.
        """
        if self._waypoints is None:
            return False

        self._markers_msg = MarkerArray()
        self._marker_id = 0

        self._interp_fcns['pos'] = list()
        self._segment_to_wp_map = [0]

        if self._waypoints.num_waypoints == 2:
            # Simple case: just one line segment
            self._interp_fcns['pos'].append(
                LineSegment(self._waypoints.get_waypoint(0).pos,
                            self._waypoints.get_waypoint(1).pos))
            self._segment_to_wp_map.append(1)
            # Set a simple spline to interpolate heading offset, if existent
            heading = [self._waypoints.get_waypoint(k).heading_offset for k in range(self._waypoints.num_waypoints)]

        elif self._waypoints.num_waypoints > 2:
            # Multiple waypoints: create line segments with blended corners
            q_seg = self._waypoints.get_waypoint(0).pos
            q_start_line = q_seg
            heading = [self._waypoints.get_waypoint(0).heading_offset]

            for i in range(1, self._waypoints.num_waypoints):
                # Create line from current start to waypoint i
                first_line = LineSegment(q_start_line, self._waypoints.get_waypoint(i).pos)
                # Determine blend radius (limited by line length)
                radius = min(self._radius, first_line.get_length() / 2)

                if i + 1 < self._waypoints.num_waypoints:
                    # Check next line segment length
                    second_line = LineSegment(self._waypoints.get_waypoint(i).pos,
                                              self._waypoints.get_waypoint(i + 1).pos)
                    radius = min(radius, second_line.get_length() / 2)

                if i < self._waypoints.num_waypoints - 1:
                    # Add point before waypoint i (start of blend)
                    q_seg = np.vstack(
                        (q_seg, first_line.interpolate((first_line.get_length() - radius) / first_line.get_length())))
                    # Add line segment
                    self._interp_fcns['pos'].append(LineSegment(q_start_line, q_seg[-1, :]))
                    heading.append(self._waypoints.get_waypoint(i).heading_offset)
                    self._segment_to_wp_map.append(i)

                if i == self._waypoints.num_waypoints - 1:
                    # Last waypoint: just add final line segment
                    q_seg = np.vstack((q_seg, self._waypoints.get_waypoint(i).pos))
                    self._interp_fcns['pos'].append(LineSegment(q_seg[-2, :], q_seg[-1, :]))
                    heading.append(self._waypoints.get_waypoint(i).heading_offset)
                    self._segment_to_wp_map.append(i)
                elif i + 1 < self._waypoints.num_waypoints:
                    # Add point after waypoint i (end of blend)
                    q_seg = np.vstack((q_seg, second_line.interpolate(radius / second_line.get_length())))
                    # Add Bezier curve for smooth corner
                    self._interp_fcns['pos'].append(
                        BezierCurve([q_seg[-2, :], self._waypoints.get_waypoint(i).pos, q_seg[-1, :]], 5))
                    heading.append(self._waypoints.get_waypoint(i).heading_offset)
                    self._segment_to_wp_map.append(i)
                    q_start_line = deepcopy(q_seg[-1, :])
        else:
            return False

        # Reparametrizing the curves
        lengths = [seg.get_length() for seg in self._interp_fcns['pos']]
        lengths = [0] + lengths
        self._s = np.cumsum(lengths) / np.sum(lengths)
        self._total_path_length = np.sum(lengths)

        mean_vel = np.mean(
            [self._waypoints.get_waypoint(k).max_forward_speed for k in range(self._waypoints.num_waypoints)])
        if self._duration is None:
            self._duration = self._total_path_length / mean_vel
        if self._start_time is None:
            self._start_time = 0.0

        # Generate velocity profile if enabled
        if self._use_velocity_profiler and self._velocity_profiler is not None:
            velocity_profile, curvature_profile = self._velocity_profiler.generate_velocity_profile(
                self._interp_fcns['pos'],
                self._waypoints,
                self._s,
                self._total_path_length
            )

            # CRITICAL: Recalculate duration based on actual velocity profile!
            # Original duration was based on mean_vel, but actual speeds vary
            estimated_duration = self._estimate_duration_from_velocity_profile(
                velocity_profile, self._s, lengths[1:])

            # Add safety buffer (20%) for velocity profiler overhead
            self._duration = estimated_duration * 1.2

            # Log profile statistics
            stats = self._velocity_profiler.get_profile_statistics()
            if stats:
                import sys
                # Force print to stdout AND stderr for visibility
                msg = '╔═══════════════════════════════════════════════════════════╗\n'
                msg += '║  VELOCITY PROFILER ACTIVATED                              ║\n'
                msg += '╠═══════════════════════════════════════════════════════════╣\n'
                msg += f'║  Speed range: {stats["v_min"]:.2f} - {stats["v_max"]:.2f} m/s                            ║\n'
                msg += f'║  Max curvature: {stats["k_max"]:.4f} (1/m)                      ║\n'
                msg += f'║  Mean curvature: {stats["k_mean"]:.4f} (1/m)                     ║\n'
                msg += f'║  Samples: {stats["num_samples"]}                                        ║\n'
                msg += f'║  Original duration: {self._total_path_length / mean_vel:.1f}s → Adjusted: {self._duration:.1f}s       ║\n'
                msg += '╚═══════════════════════════════════════════════════════════╝'
                print(msg, file=sys.stderr, flush=True)
                print(msg, file=sys.stdout, flush=True)

        # Check if all heading offsets are zero (auto-heading mode)
        import sys
        all_headings_zero = all(abs(h) < 1e-6 for h in heading)

        if all_headings_zero:
            # All waypoints use auto-heading (use_fixed_heading=false)
            # No need for heading offset interpolation - just follow path tangent
            print('[LIPB] Auto-heading mode: all heading_offsets are 0, using path tangent only', file=sys.stderr, flush=True)
            self._interp_fcns['heading'] = lambda x: 0.0
        elif self._waypoints.num_waypoints == 2:
            head_offset_line = deepcopy(self._waypoints.get_waypoint(1).heading_offset)
            print(f'[LIPB] Fixed heading mode (2 waypoints): offset = {np.rad2deg(head_offset_line):.1f}°', file=sys.stderr, flush=True)
            self._interp_fcns['heading'] = lambda x: head_offset_line
        else:
            # Set a simple spline to interpolate heading offset, if existent
            # Ensure heading and _s have same length
            if len(heading) != len(self._s):
                print(f'[LIPB ERROR] Length mismatch: heading={len(heading)}, _s={len(self._s)}', file=sys.stderr, flush=True)
                print(f'[LIPB ERROR] Using constant heading offset = 0.0', file=sys.stderr, flush=True)
                self._interp_fcns['heading'] = lambda x: 0.0
            else:
                try:
                    # Use lower order spline if not enough points
                    k = min(3, len(heading) - 1)
                    self._heading_spline = splrep(self._s, heading, k=k, per=False)
                    self._interp_fcns['heading'] = lambda x: splev(x, self._heading_spline)
                    print(f'[LIPB] Heading spline created successfully (order={k})', file=sys.stderr, flush=True)
                except Exception as e:
                    print(f'[LIPB ERROR] splrep failed: {e}', file=sys.stderr, flush=True)
                    print(f'[LIPB ERROR] Using constant heading offset = 0.0', file=sys.stderr, flush=True)
                    self._interp_fcns['heading'] = lambda x: 0.0
        return True

    def set_parameters(self, params):
        """Set interpolator's parameters. All the options
        for the `params` input can be seen below:

        ```python
        params=dict(
            radius=0.0,
            use_velocity_profiler=False,
            max_lateral_accel=0.2,
            speed_reduction_factor=0.3,
            min_speed_factor=0.2
            )
        ```

        * `radius` (*type:* `float`): Radius of the corners modeled
        as fifth-order Bezier curves. Also determines transition distance
        for velocity changes (transition = 2 × radius).
        * `use_velocity_profiler` (*type:* `bool`): Enable curvature-based
        velocity profiling for smooth cornering
        * `max_lateral_accel` (*type:* `float`): Maximum lateral acceleration
        for ROV (m/s²), typical: 0.2-0.5
        * `speed_reduction_factor` (*type:* `float`): Speed reduction factor
        for corners (0-1), lower = more deceleration
        * `min_speed_factor` (*type:* `float`): Minimum speed as fraction of
        max speed (0-1), default: 0.2

        > *Input arguments*

        * `params` (*type:* `dict`): `dict` containing interpolator's
        configurable elements.
        """
        if 'radius' in params:
            assert params['radius'] > 0, 'Radius must be greater than zero'
            self._radius = params['radius']

        if 'use_velocity_profiler' in params:
            self._use_velocity_profiler = bool(params['use_velocity_profiler'])

            # Initialize velocity profiler if enabled
            if self._use_velocity_profiler:
                max_lateral_accel = params.get('max_lateral_accel', 0.2)
                speed_reduction_factor = params.get('speed_reduction_factor', 0.3)
                min_speed_factor = params.get('min_speed_factor', 0.2)

                self._velocity_profiler = VelocityProfiler(
                    max_lateral_accel=max_lateral_accel,
                    speed_reduction_factor=speed_reduction_factor,
                    min_speed_factor=min_speed_factor
                )

        return True

    def get_samples(self, max_time, step=0.001):
        """Sample the full path with constant SPATIAL resolution (0.2m).

        > *Input arguments*

        * `step` (*type:* `float`, *default:* `0.001`): NOT USED

        > *Returns*

        List of `TrajectoryPoint` with 0.2m spatial resolution.
        """
        if self._waypoints is None:
            return None
        if self._interp_fcns['pos'] is None:
            return None

        # SPATIAL RESAMPLING: Constant 0.2m resolution
        spatial_resolution = 0.2  # meters

        # Step 1: Fine parametric sampling
        fine_step = 0.001
        s_fine = np.arange(0, 1 + fine_step, fine_step)

        fine_pnts = []
        for i in s_fine:
            pos = self.generate_pos(i)
            if pos is not None:
                fine_pnts.append(pos)

        if len(fine_pnts) < 2:
            return None

        fine_pnts = np.array(fine_pnts)

        # Step 2: Resample with constant spatial resolution
        resampled_pnts = [fine_pnts[0]]
        accumulated_dist = 0.0
        last_added_idx = 0

        for i in range(1, len(fine_pnts)):
            segment_dist = np.linalg.norm(fine_pnts[i] - fine_pnts[i-1])
            accumulated_dist += segment_dist

            if accumulated_dist >= spatial_resolution:
                resampled_pnts.append(fine_pnts[i])
                accumulated_dist = 0.0
                last_added_idx = i

        # Always include last point
        if last_added_idx != len(fine_pnts) - 1:
            resampled_pnts.append(fine_pnts[-1])

        # Step 3: Convert to TrajectoryPoint
        result = []
        for pos in resampled_pnts:
            dists = np.linalg.norm(fine_pnts - pos, axis=1)
            closest_fine_idx = np.argmin(dists)
            s_approx = closest_fine_idx * fine_step

            pnt = TrajectoryPoint()
            pnt.pos = pos.tolist()
            pnt.rotq = self.generate_quat(s_approx)
            pnt.t = 0.0
            result.append(pnt)

        return result

    def generate_pos(self, s, *args):
        """Generate a position vector for the path sampled point
        interpolated on the position related to `s`, `s` being
        represented in the curve's parametric space.

        > *Input arguments*

        * `s` (*type:* `float`): Curve's parametric input expressed in the
        interval of [0, 1]

        > *Returns*

        3D position vector as a `numpy.array`.
        """
        if self._interp_fcns['pos'] is None:
            return None
        idx = self.get_segment_idx(s)
        if idx == 0:
            u_k = 0
            pos = self._interp_fcns['pos'][idx].interpolate(u_k)
        else:
            u_k = (s - self._s[idx - 1]) / (self._s[idx] - self._s[idx - 1])
            pos = self._interp_fcns['pos'][idx - 1].interpolate(u_k)
        return pos

    def generate_pnt(self, s, t=0.0, *args):
        """Compute a point that belongs to the path on the
        interpolated space related to `s`, `s` being represented
        in the curve's parametric space.

        > *Input arguments*

        * `s` (*type:* `float`): Curve's parametric input expressed in the
        interval of [0, 1]
        * `t` (*type:* `float`): Trajectory point's timestamp

        > *Returns*

        `TrajectoryPoint` including position
        and quaternion vectors.
        """
        pnt = TrajectoryPoint()
        # Trajectory time stamp
        pnt.t = t
        # Set position vector
        pnt.pos = self.generate_pos(s).tolist()
        # Set rotation quaternion
        pnt.rotq = self.generate_quat(s)
        return pnt

    def generate_quat(self, s):
        """Compute the quaternion of the path reference for a interpolated
        point related to `s`, `s` being represented in the curve's parametric
        space.
        The quaternion is computed assuming the heading follows the direction
        of the path towards the target. Roll and pitch can also be computed
        in case the `full_dof` is set to `True`.

        > *Input arguments*

        * `s` (*type:* `float`): Curve's parametric input expressed in the
        interval of [0, 1]

        > *Returns*

        Rotation quaternion as a `numpy.array` as `(w, x, y, z)`
        """
        s = max(0, s)
        s = min(s, 1)

        last_s = s - self._s_step
        if last_s <= 0:
            last_s = 0

        if s == 0:
            self._last_rot = deepcopy(self._init_rot)
            return self._init_rot

        this_pos = self.generate_pos(s)
        last_pos = self.generate_pos(last_s)
        dx = this_pos[0] - last_pos[0]
        dy = this_pos[1] - last_pos[1]
        dz = this_pos[2] - last_pos[2]

        rotq = self._compute_rot_quat(dx, dy, dz)
        self._last_rot = deepcopy(rotq)

        # Calculating the step for the heading offset
        heading_offset_value = self._interp_fcns['heading'](s)

        # Check for NaN
        if np.isnan(heading_offset_value) or np.isinf(heading_offset_value):
            import sys
            print(f'[LIPB ERROR] Invalid heading_offset_value={heading_offset_value} at s={s}, using 0.0', file=sys.stderr, flush=True)
            heading_offset_value = 0.0

        q_step = axangle2quat([0, 0, 1], heading_offset_value, is_normalized=True)

        # Adding the heading offset to the rotation quaternion
        rotq = qmult(rotq, q_step)

        # Final NaN check
        if np.any(np.isnan(rotq)) or np.any(np.isinf(rotq)):
            import sys
            print(f'[LIPB ERROR] Final rotq is invalid: {rotq}, using last_rot', file=sys.stderr, flush=True)
            rotq = self._last_rot

        return rotq

    def get_velocity_at_s(self, s):
        """Get the velocity at parameter s from the velocity profiler.

        > *Input arguments*

        * `s` (*type:* `float`): Curve's parametric input expressed in the
        interval of [0, 1]

        > *Returns*

        Velocity at s (m/s) as `float`. If velocity profiler is not enabled,
        returns None.
        """
        if self._use_velocity_profiler and self._velocity_profiler is not None:
            return self._velocity_profiler.get_velocity_at_s(s)
        return None

    def get_curvature_at_s(self, s):
        """Get the curvature at parameter s from the velocity profiler.

        > *Input arguments*

        * `s` (*type:* `float`): Curve's parametric input expressed in the
        interval of [0, 1]

        > *Returns*

        Curvature at s (1/m) as `float`. If velocity profiler is not enabled,
        returns None.
        """
        if self._use_velocity_profiler and self._velocity_profiler is not None:
            return self._velocity_profiler.get_curvature_at_s(s)
        return None

    def _estimate_duration_from_velocity_profile(self, velocity_profile, s_values, segment_lengths):
        """Estimate trajectory duration from velocity profile.

        This calculates the actual time needed to traverse the path
        given the varying velocities from the velocity profiler.

        > *Input arguments*

        * `velocity_profile` (*type:* `list`): List of (s, velocity) tuples
        * `s_values` (*type:* `numpy.array`): Normalized path parameters
        * `segment_lengths` (*type:* `list`): Physical lengths of segments (m)

        > *Returns*

        Estimated duration in seconds as `float`.
        """
        if not velocity_profile or len(velocity_profile) < 2:
            return self._duration

        total_time = 0.0

        # Integrate time = distance / velocity over the profile
        for i in range(len(velocity_profile) - 1):
            s1, v1 = velocity_profile[i]
            s2, v2 = velocity_profile[i + 1]

            # Distance in this segment (parametric → physical)
            ds = s2 - s1
            distance = ds * self._total_path_length

            # Average velocity in this segment
            v_avg = (v1 + v2) / 2.0

            # Time = distance / velocity
            if v_avg > 1e-6:
                dt = distance / v_avg
                total_time += dt

        return total_time
