# SPDX-FileCopyrightText: 2016-2019 The UUV Simulator Authors
#
# SPDX-License-Identifier: GPL-3.0-or-later
import numpy as np
from copy import deepcopy
import logging

from visualization_msgs.msg import MarkerArray
from ..common.waypoint import Waypoint
from ..common.waypoint_set import WaypointSet
from ..common.trajectory_point import TrajectoryPoint

# transforms3d functions for quaternion operations
from transforms3d.quaternions import axangle2quat, qmult
from transforms3d.euler import euler2quat, mat2euler
from transforms3d.affines import compose


class PathGenerator(object):
    """Base class to be inherited by custom path generator
    to generate paths from interpolated waypoints.

    > *Attributes*

    * `LABEL` (*type:* `str`): Name of the path generator

    > *Input arguments*

    * `full_dof` (*type:* `bool`, *default:* `False`): If `True`, generate
    6 DoF paths, if `False`, roll and pitch are set to zero.
    """
    LABEL = ''

    def __init__(self, full_dof=False):
        self._logger = logging.getLogger(self.__class__.__name__)
        # Waypoint set
        self._waypoints = None

        # True if the path is generated for all degrees of freedom, otherwise
        # the path will be generated for (x, y, z, yaw) only
        self._is_full_dof = full_dof

        # The parametric variable to use as input for the interpolator
        self._s = list()
        self._segment_to_wp_map = list()
        self._cur_s = 0
        self._s_step = 0.0001

        self._start_time = None
        self._duration = None

        self._termination_by_time = True

        self._final_pos_tolerance = 0.1

        # Initial rotation: (w, x, y, z) = (1, 0, 0, 0) identity
        self._init_rot = axangle2quat([0, 0, 1], 0.0, is_normalized=True)  # identity quat
        self._last_rot = axangle2quat([0, 0, 1], 0.0, is_normalized=True)

        self._markers_msg = MarkerArray()
        self._marker_id = 0

    @staticmethod
    def get_generator(name, *args, **kwargs):
        """Factory method for all derived path generators.

        > *Input arguments*

        * `name` (*type:* `str`): Name identifier of the path generator
        * `args` (*type:* `list`): List of arguments for the path generator constructor
        * `kwargs` (*type:* `dict`): Keyword arguments for the path generator constructor

        > *Returns*

        An instance of the desired path generator. If the `name` input
        does not describe any of the derived path generator classes, an
        `ValueError` will be raised.
        """
        for gen in PathGenerator.__subclasses__():
            if name == gen.LABEL:
                return gen(*args, **kwargs)

        msg = 'Invalid path generator method'
        raise ValueError(msg)

    @staticmethod
    def get_all_generators():
        """Get the name identifiers of all path generator classes.

        > *Returns*

        List of path generator instances
        """
        generators = list()
        for gen in PathGenerator.__subclasses__():
            generators.append(gen())
        return generators

    @property
    def waypoints(self):
        """`WaypointSet`: Set of waypoints"""
        return self._waypoints

    @property
    def max_time(self):
        """`float`: Absolute final timestamp assigned to the path in seconds"""
        if self._duration is None or self._start_time is None:
            return None
        return self._duration + self._start_time

    @property
    def duration(self):
        """`float`: Duration in seconds for the whole path"""
        return self._duration

    @duration.setter
    def duration(self, t):
        assert t > 0, 'Duration must be a positive value'
        self._duration = t

    @property
    def start_time(self):
        """`float`: Start timestamp assigned to the first waypoint"""
        return self._start_time

    @start_time.setter
    def start_time(self, time):
        assert time >= 0, 'Invalid negative time'
        self._start_time = time

    @property
    def closest_waypoint(self):
        """`Waypoint`: Return the closest waypoint
        to the current position on the path.
        """
        return self._waypoints.get_waypoint(self.closest_waypoint_idx)

    @property
    def closest_waypoint_idx(self):
        """Return the index of the closest waypoint to the current
        position on the path.
        """

        if self._cur_s == 0:
            return 0
        if self._cur_s == 1:
            return len(self._s) - 1
        # Nearest waypoint = smallest *absolute* distance along the path.
        # (T1.4: argmin of the signed difference always picked the earliest
        # waypoint, since _s is monotonically increasing from 0.)
        v = np.abs(np.array(self._s) - self._cur_s)
        idx = np.argmin(v)
        return idx

    @property
    def s_step(self):
        """`float`: Value of the step size for the path's parametric
        variable
        """
        return self._s_step

    @s_step.setter
    def s_step(self, step):
        assert 0 < step < 1
        self._s_step = step

    @property
    def termination_by_time(self):
        """`bool`: Termination condition based on time"""
        return self._termination_by_time

    def reset(self):
        self._s = list()
        self._segment_to_wp_map = list()
        self._cur_s = 0
        self._s_step = 0.0001

        self._start_time = None
        self._duration = None

    def get_segment_idx(self, s):
        if len(self._s) == 0:
            return 0
        # Ensure the parameter s is 0 <= s <= 1
        s = max(0, s)
        s = min(s, 1)

        if s == 1:
            idx = self._s.size - 1
        else:
            idx = (self._s - s >= 0).nonzero()[0][0]
        return idx

    def get_remaining_waypoints_idx(self, s):
        idx = self.get_segment_idx(s)
        try:
            wps = self._segment_to_wp_map[idx::]
            return np.unique(wps)
        except:
            self._logger.error('Invalid segment index')
            return None

    def is_full_dof(self):
        return self._is_full_dof

    def set_full_dof(self, flag):
        self._is_full_dof = flag

    def get_label(self):
        return self.LABEL

    def init_interpolator(self):
        raise NotImplementedError()

    def get_samples(self, max_time, step=0.005):
        raise NotImplementedError()

    def get_visual_markers(self):
        return self._markers_msg

    def add_waypoint(self, waypoint, add_to_beginning=False):
        """Add waypoint to the existing waypoint set. If no waypoint set has
        been initialized, create new waypoint set structure and add the given
        waypoint."""
        if self._waypoints is None:
            self._waypoints = WaypointSet()
        self._waypoints.add_waypoint(waypoint, add_to_beginning)
        return self.init_interpolator()

    def init_waypoints(self, waypoints=None, init_rot=np.array([1, 0, 0, 0])):
        if waypoints is not None:
            self._waypoints = deepcopy(waypoints)

        if self._waypoints is None:
            self._logger.error('Waypoint list has not been initialized')
            return False

        self._init_rot = init_rot
        self._logger.info('Setting initial rotation as={}'.format(init_rot))
        return True

    def interpolate(self, tag, s):
        return self._interp_fcns[tag](s)

    def has_finished(self, t):
        if self._termination_by_time:
            return t > self.max_time
        else:
            return True

    def has_started(self, t):
        if self._termination_by_time:
            return t - self.start_time > 0
        else:
            return True

    def generate_pnt(self, s):
        raise NotImplementedError()

    def generate_pos(self, s):
        raise NotImplementedError()

    def generate_quat(self, s):
        raise NotImplementedError()

    def set_parameters(self, params):
        raise NotImplementedError()

    def _compute_rot_quat(self, dx, dy, dz):
        """Compute rotation quaternion from direction vector.

        Computes a quaternion representing the rotation needed to align
        with the direction (dx, dy, dz).

        Returns quaternion in (w, x, y, z) format.
        """
        if np.isclose(dx, 0) and np.isclose(dy, 0):
            rotq = self._last_rot
        else:
            heading = np.arctan2(dy, dx)
            # axangle2quat returns (w, x, y, z)
            rotq = axangle2quat([0, 0, 1], heading, is_normalized=True)

        if self._is_full_dof:
            pitch = -1 * np.arctan2(dz, np.sqrt(dx**2 + dy**2))
            rote = axangle2quat([0, 1, 0], pitch, is_normalized=True)
            # qmult(q1, q2) returns q1 * q2 in (w, x, y, z) format
            rotq = qmult(rotq, rote)

        # Certify that the next quaternion remains in the same half hemisphere
        d_prod = np.dot(self._last_rot, rotq)
        if d_prod < 0:
            rotq *= -1

        return rotq
