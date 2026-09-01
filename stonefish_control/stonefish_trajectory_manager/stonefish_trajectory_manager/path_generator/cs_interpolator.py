# SPDX-FileCopyrightText: 2016-2019 The UUV Simulator Authors
#
# SPDX-License-Identifier: GPL-3.0-or-later

from scipy.interpolate import splrep, splev
import numpy as np
from copy import deepcopy
from visualization_msgs.msg import MarkerArray
from transforms3d.quaternions import axangle2quat, qmult

from ..common.waypoint import Waypoint
from ..common.waypoint_set import WaypointSet
from ..common.trajectory_point import TrajectoryPoint
from .line_segment import LineSegment
from .bezier_curve import BezierCurve
from .path_generator import PathGenerator


class CSInterpolator(PathGenerator):
    """Interpolator that will generate [cubic Bezier curve](https://en.wikipedia.org/wiki/B%C3%A9zier_curve)
    segments for a set of waypoints. The full algorithm can
    be seen in `Biagiotti and Melchiorri, 2008`.

    !!! note

        Biagiotti, Luigi, and Claudio Melchiorri. Trajectory planning for
        automatic machines and robots. Springer Science & Business Media, 2008.
    """
    LABEL = 'cubic'

    def __init__(self):
        super(CSInterpolator, self).__init__()

        # Set of interpolation functions for each degree of freedom
        # The heading function interpolates the given heading offset and its
        # value is added to the heading computed from the trajectory
        self._interp_fcns = dict(pos=None,
                                 heading=None)
        self._heading_spline = None

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
            self._interp_fcns['pos'].append(
                LineSegment(self._waypoints.get_waypoint(0).pos,
                            self._waypoints.get_waypoint(1).pos))
            self._segment_to_wp_map.append(1)
        elif self._waypoints.num_waypoints > 2:
            self._interp_fcns['pos'], tangents = BezierCurve.generate_cubic_curve(
                [self._waypoints.get_waypoint(i).pos for i in range(self._waypoints.num_waypoints)])
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

        if self._waypoints.num_waypoints == 2:
            head_offset_line = deepcopy(self._waypoints.get_waypoint(1).heading_offset)
            self._interp_fcns['heading'] = lambda x: head_offset_line
        else:
            # Set a simple spline to interpolate heading offset, if existent
            heading = [self._waypoints.get_waypoint(i).heading_offset for i in range(self._waypoints.num_waypoints)]
            self._heading_spline = splrep(self._s, heading, k=3, per=False)
            self._interp_fcns['heading'] = lambda x: splev(x, self._heading_spline)

        return True

    def set_parameters(self, params):
        """Set interpolator's parameters.

        The cubic interpolator takes no configurable parameters today; the
        method exists because `WPTrajectoryGenerator` calls it on every
        interpolator uniformly.
        """
        return True

    def get_samples(self, max_time, step=0.001):
        """Sample the full path for position and quaternion vectors.
        Uses constant SPATIAL resolution (not parametric).

        > *Input arguments*

        * `step` (*type:* `float`, *default:* `0.001`): NOT USED - kept for compatibility
                                                         Spatial resolution is hardcoded to 0.2m

        > *Returns*

        List of `TrajectoryPoint` with constant 0.2m spatial resolution.
        """
        if self._waypoints is None:
            return None
        if self._interp_fcns['pos'] is None:
            return None

        # SPATIAL RESAMPLING: Constant 0.2m resolution
        spatial_resolution = 0.2  # meters

        # Step 1: Fine sampling in parametric space
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
        resampled_pnts = [fine_pnts[0]]  # Start with first point
        accumulated_dist = 0.0
        last_added_idx = 0

        for i in range(1, len(fine_pnts)):
            segment_dist = np.linalg.norm(fine_pnts[i] - fine_pnts[i-1])
            accumulated_dist += segment_dist

            # Add point when accumulated distance exceeds resolution
            if accumulated_dist >= spatial_resolution:
                resampled_pnts.append(fine_pnts[i])
                accumulated_dist = 0.0
                last_added_idx = i

        # Always include last point
        if last_added_idx != len(fine_pnts) - 1:
            resampled_pnts.append(fine_pnts[-1])

        # Step 3: Convert to TrajectoryPoint with quaternions
        result = []
        for pos in resampled_pnts:
            # Find parametric value for this position (approximate)
            # Use closest point in fine sampling
            dists = np.linalg.norm(fine_pnts - pos, axis=1)
            closest_fine_idx = np.argmin(dists)
            s_approx = closest_fine_idx * fine_step

            pnt = TrajectoryPoint()
            pnt.pos = pos.tolist()
            pnt.rotq = self.generate_quat(s_approx)
            pnt.t = 0.0
            result.append(pnt)

        return result

    def generate_pos(self, s):
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

    def generate_pnt(self, s, t, *args):
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

        if s == 0:
            self._last_rot = deepcopy(self._init_rot)
            return self._init_rot

        last_s = max(0, s - self._s_step)

        this_pos = self.generate_pos(s)
        last_pos = self.generate_pos(last_s)

        dx = this_pos[0] - last_pos[0]
        dy = this_pos[1] - last_pos[1]
        dz = this_pos[2] - last_pos[2]

        rotq = self._compute_rot_quat(dx, dy, dz)
        self._last_rot = rotq
        # Calculating the step for the heading offset
        q_step = axangle2quat([0, 0, 1], self._interp_fcns['heading'](s), is_normalized=True)
        # Adding the heading offset to the rotation quaternion
        rotq = qmult(rotq, q_step)
        return rotq

