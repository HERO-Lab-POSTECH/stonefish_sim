# SPDX-FileCopyrightText: 2016-2019 The UUV Simulator Authors
#
# SPDX-License-Identifier: GPL-3.0-or-later
import numpy as np
from copy import deepcopy
import logging
import sys
import time

from .trajectory_point import TrajectoryPoint
from .waypoint import Waypoint
from .waypoint_set import WaypointSet
from transforms3d.quaternions import qmult, qinverse, qconjugate, axangle2quat

from ..path_generator import PathGenerator


class WPTrajectoryGenerator(object):
    """Class that generates a trajectory from the interpolated path generated
    from a set of waypoints. It uses the information given for the waypoint's
    maximum forward speed to estimate the velocity between waypoint and
    parametrize the interpolated curve.
    The velocity and acceleration profiles are the generated through finite
    discretization. These profiles are not optimized, this class is a
    simple solution for quick trajectory generation for waypoint navigation.

    > *Input arguments*

    * `full_dof` (*type:* `bool`, *default:* `False`): `True` to generate 6 DoF
    trajectories
    * `use_finite_diff` (*type:* `bool`, *default:* `True`): Use finite differentiation
    if `True`, otherwise use the motion regression algorithm
    * `interpolation_method` (*type:* `str`, *default:* `cubic`): Name of the interpolation
    method, options are `cubic`, `dubins`, `lipb` or `linear`
    * `stamped_pose_only` (*type:* `bool`, *default:* `False`): Generate only position
    and quaternion vectors, velocities and accelerations are set to zero
    """
    def __init__(self, full_dof=False, use_finite_diff=True,
                 interpolation_method='cubic',
                 stamped_pose_only=False):
        """Class constructor."""
        self._logger = logging.getLogger(__name__)
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(logging.Formatter(
            '%(asctime)s | %(levelname)s | %(module)s | %(message)s'))
        handler.setLevel(logging.WARNING)
        self._logger.addHandler(handler)
        self._logger.setLevel(logging.WARNING)

        self._path_generators = dict()
        for gen in PathGenerator.get_all_generators():
            self._path_generators[gen.get_label()] = gen
            self._path_generators[gen.get_label()].set_full_dof(full_dof)
        # Time step between interpolated samples
        self._dt = None
        # Last time stamp
        self._last_t = None
        # Last interpolated point
        self._last_pnt = None
        self._this_pnt = None

        # Velocity profiler integration
        self._integrated_s = 0.0  # Actual s based on velocity integration
        self._last_integration_time = None

        # Trajectory advancement mode
        # 'time': Time-based (original)
        # 'velocity_damped': Velocity integration with error-based damping (Option 3)
        # 'pure_pursuit': Pure pursuit controller (Option 4, future)
        self._advancement_mode = 'time'

        # Error-based damping parameters (for velocity_damped mode)
        self._error_threshold = 0.5  # meters (error at which damping = 0.5, increased for more tolerance)
        self._min_damping = 0.15  # minimum damping factor (maintain at least 15% speed, prevents complete stop)

        # Distance-based termination
        self._final_waypoint_pos = None  # Set when trajectory starts
        self._completion_threshold = 0.5  # meters (distance to final waypoint)

        # Flag to generate only stamped pose, no velocity profiles
        self._stamped_pose_only = stamped_pose_only

        self._t_step = 0.001

        # Interpolation method
        self._interp_method = interpolation_method

        # True if the path is generated for all degrees of freedom, otherwise
        # the path will be generated for (x, y, z, yaw) only
        self._is_full_dof = full_dof

        # Use finite differentiation if true, otherwise use motion regression
        # algorithm
        self._use_finite_diff = use_finite_diff
        # Time window used for the regression method
        self._regression_window = 0.5
        # If the regression method is used, adjust the time step
        if not self._use_finite_diff:
            self._t_step = self._regression_window / 30

        # Flags to indicate that the interpolation process has started and
        # ended
        self._has_started = False
        self._has_ended = False

        # The parametric variable to use as input for the interpolator
        self._cur_s = 0

        self._init_rot = axangle2quat([0, 0, 1], 0.0, is_normalized=True)

    def __del__(self):
        # Removing logging message handlers
        while self._logger.handlers:
            self._logger.handlers.pop()

    @property
    def started(self):
        """`bool`: Flag set to true if the interpolation has started."""
        return self._has_started

    @property
    def closest_waypoint(self):
        """Return the closest waypoint to the current position on the path."""
        return self._path_generators[self._interp_method].closest_waypoint

    @property
    def closest_waypoint_idx(self):
        """`int`: Index of the closest waypoint to the current position on the
        path.
        """
        return self._path_generators[self._interp_method].closest_waypoint_idx

    @property
    def interpolator(self):
        """`PathGenerator`: Current interpolation method instance"""
        return self._path_generators[self._interp_method]

    @property
    def interpolator_tags(self):
        """List of `str`: List of all interpolation method"""
        return [gen.get_label() for gen in PathGenerator.get_all_generators()]

    @property
    def use_finite_diff(self):
        """`bool`: Use finite differentiation for computation of
        trajectory points
        """
        return self._use_finite_diff

    @use_finite_diff.setter
    def use_finite_diff(self, flag):
        assert type(flag) == bool
        self._use_finite_diff = flag

    @property
    def stamped_pose_only(self):
        """`bool`: Flag to enable computation of stamped poses"""
        return self._stamped_pose_only

    @stamped_pose_only.setter
    def stamped_pose_only(self, flag):
        self._stamped_pose_only = flag

    def get_interpolation_method(self):
        return self._interp_method

    def get_visual_markers(self):
        return self.interpolator.get_visual_markers()

    def set_interpolation_method(self, method):
        if method in self._path_generators:
            self._interp_method = method
            return True
        else:
            return False

    def set_interpolator_parameters(self, method, params):
        if method not in self.interpolator_tags:
            self._logger.error('Invalid interpolation method: ' + str(method))
            return False
        return self._path_generators[method].set_parameters(params)

    def is_full_dof(self):
        """Return true if the trajectory is generated for all 6 degrees of
        freedom.
        """
        return self._is_full_dof

    def get_max_time(self):
        """Return maximum trajectory time."""
        return self.interpolator.max_time

    def set_duration(self, t):
        """Set a new maximum trajectory time."""
        if t > 0:
            self.interpolator.duration = t
            self.interpolator.s_step = self._t_step / self.interpolator.duration
            self._logger.info('New duration, max. relative time=%.2f s' % self.interpolator.duration)
            return True
        else:
            self._logger.info('Invalid max. time, time=%.2f s' % t)
            return False

    def set_advancement_mode(self, mode):
        """Set trajectory advancement mode.

        Args:
            mode: 'time', 'velocity_damped', or 'pure_pursuit'
        """
        valid_modes = ['time', 'velocity_damped', 'pure_pursuit']
        if mode in valid_modes:
            self._advancement_mode = mode
            self._logger.info(f'Trajectory advancement mode: {mode}')
            return True
        else:
            self._logger.error(f'Invalid advancement mode: {mode}. Valid: {valid_modes}')
            return False

    def set_damping_parameters(self, error_threshold=None, min_damping=None):
        """Set error-based damping parameters.

        Args:
            error_threshold: Error (m) at which damping = 0.5
            min_damping: Minimum damping factor (0-1)
        """
        if error_threshold is not None:
            if error_threshold > 0:
                self._error_threshold = error_threshold
                self._logger.info(f'Error threshold: {error_threshold:.2f}m')
            else:
                self._logger.error('Error threshold must be positive')
                return False

        if min_damping is not None:
            if 0 < min_damping <= 1.0:
                self._min_damping = min_damping
                self._logger.info(f'Min damping: {min_damping:.2f}')
            else:
                self._logger.error('Min damping must be in (0, 1]')
                return False

        return True

    def has_finished(self):
        """Return true if the trajectory has finished."""
        return self._has_ended

    def reset(self):
        """Reset all class attributes to allow a new trajectory to be
        computed.
        """
        self._dt = None
        self._last_t = None
        self._last_pnt = None
        self._this_pnt = None
        self._has_started = False
        self._has_ended = False
        self._cur_s = 0
        self._integrated_s = 0.0
        self._last_integration_time = None

    def init_waypoints(self, waypoint_set, init_rot=(1, 0, 0, 0)):
        """Initialize the waypoint set."""
        self.reset()
        self.interpolator.reset()
        return self.interpolator.init_waypoints(waypoint_set, init_rot)

    def add_waypoint(self, waypoint, add_to_beginning=False):
        """Add waypoint to the existing waypoint set. If no waypoint set has
        been initialized, create new waypoint set structure and add the given
        waypoint."""
        return self.interpolator.add_waypoint(waypoint, add_to_beginning)

    def get_waypoints(self):
        """Return waypoint set."""
        return self.interpolator.waypoints

    def update_dt(self, t):
        """Update the time stamp."""
        if self._last_t is None:
            self._last_t = t
            self._dt = 0.0
            if self.interpolator.start_time is None:
                self.interpolator.start_time = t
            return False
        self._dt = t - self._last_t
        self._last_t = t
        return (True if self._dt > 0 else False)

    def get_samples(self, step=0.005):
        """Return pose samples from the interpolated path."""
        assert step > 0, 'Step size must be positive'
        return self.interpolator.get_samples(0.0, step)

    def set_start_time(self, t):
        """Set a custom starting time to the interpolated trajectory."""
        assert t >= 0, 'Starting time must be positive'
        self.interpolator.start_time = t

    def _motion_regression_1d(self, pnts, t):
        """
        Computation of the velocity and acceleration for the target time t
        using a sequence of points with time stamps for one dimension. This
        is an implementation of the algorithm presented by [1].

        !!! note

            [1] Sittel, Florian, Joerg Mueller, and Wolfram Burgard. Computing
                velocities and accelerations from a pose time sequence in
                three-dimensional space. Technical Report 272, University of
                Freiburg, Department of Computer Science, 2013.
        """

        sx = 0.0
        stx = 0.0
        st2x = 0.0
        st = 0.0
        st2 = 0.0
        st3 = 0.0
        st4 = 0.0
        for pnt in pnts:
            ti = pnt[1] - t
            sx += pnt[0]
            stx += pnt[0] * ti
            st2x += pnt[0] * ti**2
            st += ti
            st2 += ti**2
            st3 += ti**3
            st4 += ti**4

        n = len(pnts)
        A = n * (st3 * st3 - st2 * st4) + \
            st * (st * st4 - st2 * st3) + \
            st2 * (st2 * st2 - st * st3)

        if A == 0.0:
            return 0.0, 0.0

        v = (1.0 / A) * (sx * (st * st4 - st2 * st3) +
                         stx * (st2 * st2 - n * st4) +
                         st2x * (n * st3 - st * st2))

        a = (2.0 / A) * (sx * (st2 * st2 - st * st3) +
                         stx * (n * st3 - st * st2) +
                         st2x * (st * st - n * st2))
        return v, a

    def _motion_regression_6d(self, pnts, qt, t):
        """
        Compute translational and rotational velocities and accelerations in
        the inertial frame at the target time t.

        !!! note

            [1] Sittel, Florian, Joerg Mueller, and Wolfram Burgard. Computing
                velocities and accelerations from a pose time sequence in
                three-dimensional space. Technical Report 272, University of
                Freiburg, Department of Computer Science, 2013.
        """

        lin_vel = np.zeros(3)
        lin_acc = np.zeros(3)

        q_d = np.zeros(4)
        q_dd = np.zeros(4)

        for i in range(3):
            v, a = self._motion_regression_1d(
                [(pnt['pos'][i], pnt['t']) for pnt in pnts], t)
            lin_vel[i] = v
            lin_acc[i] = a

        for i in range(4):
            v, a = self._motion_regression_1d(
                [(pnt['rot'][i], pnt['t']) for pnt in pnts], t)
            q_d[i] = v
            q_dd[i] = a

        # Keeping all velocities and accelerations in the inertial frame
        ang_vel = 2 * qmult(q_d, qconjugate(qt))
        ang_acc = 2 * qmult(q_dd, qconjugate(qt))

        return np.hstack((lin_vel, ang_vel[0:3])), np.hstack((lin_acc, ang_acc[0:3]))

    def generate_pnt(self, t, pos, rot):
        """Return trajectory sample for the current parameter s."""
        # Check if velocity profiler is active
        use_velocity_integration = (hasattr(self.interpolator, 'get_velocity_at_s') and
                                     hasattr(self.interpolator, '_use_velocity_profiler') and
                                     self.interpolator._use_velocity_profiler)

        if use_velocity_integration:
            # CRITICAL FIX: Integrate velocity to get actual s
            # This ensures position reference matches actual velocity!
            if self._last_integration_time is None:
                self._last_integration_time = t
                self._integrated_s = 0.0

            # Time step
            dt = t - self._last_integration_time
            self._last_integration_time = t

            # Get current velocity from profile
            current_velocity = self.interpolator.get_velocity_at_s(self._integrated_s)
            if current_velocity is None:
                current_velocity = 1.0  # Fallback

            # Calculate distance traveled: ds = v * dt / total_length
            total_length = getattr(self.interpolator, '_total_path_length', 1.0)
            if total_length > 0 and dt > 0:
                # Base ds calculation
                ds = (current_velocity * dt) / total_length

                # Apply error-based damping if in velocity_damped mode
                damping = 1.0

                # DEBUG: Log advancement mode check (once)
                if not hasattr(self, '_mode_logged'):
                    import sys
                    print(f'[MODE_CHECK] advancement_mode={self._advancement_mode}, last_pnt={self._last_pnt is not None}',
                          file=sys.stderr, flush=True)
                    self._mode_logged = True

                if self._advancement_mode == 'velocity_damped' and self._last_pnt is not None:
                    # CRITICAL FIX: Use cross-track error, NOT total position error!
                    # Total position error causes vicious cycle:
                    #   Robot behind → high error → strong damping → slower → falls further behind
                    #
                    # Cross-track error is correct:
                    #   Robot off-path → slow down to converge
                    #   Robot on-path but behind → maintain speed to catch up

                    robot_pos = np.array(pos[:3])
                    ref_pos = np.array(self._last_pnt.pos[:3])

                    # Calculate path tangent (direction of motion)
                    if hasattr(self.interpolator, 'generate_pos'):
                        # Get tangent from interpolator
                        s_curr = self._integrated_s
                        s_next = min(s_curr + 0.01, 1.0)
                        if s_curr < 0.99:
                            pos_curr = self.interpolator.generate_pos(s_curr)
                            pos_next = self.interpolator.generate_pos(s_next)
                            path_tangent = pos_next - pos_curr
                            tangent_norm = np.linalg.norm(path_tangent)
                            if tangent_norm > 1e-6:
                                path_tangent = path_tangent / tangent_norm
                            else:
                                path_tangent = np.array([1.0, 0.0, 0.0])  # Fallback
                        else:
                            path_tangent = np.array([1.0, 0.0, 0.0])  # End of path
                    else:
                        path_tangent = np.array([1.0, 0.0, 0.0])  # Fallback

                    # Calculate cross-track error (perpendicular to path)
                    error_vector = robot_pos - ref_pos
                    along_track_component = np.dot(error_vector, path_tangent) * path_tangent
                    cross_track_vector = error_vector - along_track_component
                    cross_track_error = np.linalg.norm(cross_track_vector)

                    # Smooth damping based on cross-track error only
                    # cross_track=0 → damping=1.0 (full speed, robot on path)
                    # cross_track=threshold → damping ≈ 0.52
                    # cross_track=∞ → damping=min_damping (maintains minimum 15% speed)
                    k = 1.5
                    damping = self._min_damping + (1.0 - self._min_damping) * np.exp(-k * cross_track_error / self._error_threshold)

                    # Apply damping
                    ds = ds * damping

                    # DEBUG: Log damping (very throttled)
                    if not hasattr(self, '_damping_log_count'):
                        self._damping_log_count = 0
                    if self._damping_log_count % 200 == 0:  # Log every 200 calls
                        import sys
                        print(f'[DAMPING] cross_track_error={cross_track_error:.3f}m, damping={damping:.3f}, ds={ds:.6f}',
                              file=sys.stderr, flush=True)
                    self._damping_log_count += 1
                else:
                    # DEBUG: Log why damping is NOT applied
                    if not hasattr(self, '_no_damping_logged'):
                        import sys
                        print(f'[NO_DAMPING] mode={self._advancement_mode}, last_pnt_exists={self._last_pnt is not None}',
                              file=sys.stderr, flush=True)
                        self._no_damping_logged = True

                self._integrated_s += ds

                # DEBUG: Log integration (throttled)
                if not hasattr(self, '_debug_log_count'):
                    self._debug_log_count = 0
                if self._debug_log_count % 100 == 0:  # Log every 100 calls
                    import sys
                    print(f'[DEBUG] t={t:.2f}s, s={self._integrated_s:.4f}, v={current_velocity:.3f}m/s, dt={dt:.4f}s, ds={ds:.6f}, L={total_length:.2f}m',
                          file=sys.stderr, flush=True)
                self._debug_log_count += 1

            # Clamp to [0, 1]
            cur_s = max(0.0, min(1.0, self._integrated_s))
        else:
            # Original time-based calculation
            cur_s = (t - self.interpolator.start_time) / (self.interpolator.max_time - self.interpolator.start_time)

        last_s = cur_s - self.interpolator.s_step
        # Generate position and rotation quaternion for the current path
        # generator method
        pnt = self.interpolator.generate_pnt(
            cur_s,
            cur_s * (self.interpolator.max_time - self.interpolator.start_time) + self.interpolator.start_time,
            pos,
            rot)

        if self.get_interpolation_method() != 'los':
            if self._use_finite_diff:
                # Set linear velocity
                pnt.vel = self._generate_vel(cur_s)
                # Compute linear and angular accelerations
                last_vel = self._generate_vel(last_s)
                pnt.acc = (pnt.vel - last_vel) / self._t_step
            else:
                pnts = list()
                for ti in np.arange(pnt.t - self._regression_window / 2, pnt.t + self._regression_window, self._t_step):
                    if ti < 0:
                        si = 0
                    elif ti > self.interpolator.max_time - self.interpolator.start_time:
                        si = 1
                    else:
                        si = (ti - self.interpolator.start_time) / self.interpolator.max_time
                    pnts.append(dict(pos=self.interpolator.generate_pos(si),
                                     rot=self.interpolator.generate_quat(si),
                                     t=ti))
                if not self._stamped_pose_only:
                    vel, acc = self._motion_regression_6d(pnts, pnt.rotq, pnt.t)
                    pnt.vel = vel
                    pnt.acc = acc
                else:
                    pnt.vel = np.zeros(6)
                    pnt.acc = np.zeros(6)
        else:
            pnt.vel = np.zeros(6)
            pnt.acc = np.zeros(6)
        return pnt

    def _generate_vel(self, s=None):
        if self._stamped_pose_only:
            return np.zeros(6)
        cur_s = (self._cur_s if s is None else s)
        last_s = cur_s - self.interpolator.s_step

        if last_s < 0 or cur_s > 1:
            return np.zeros(6)

        q_cur = self.interpolator.generate_quat(cur_s)
        q_last = self.interpolator.generate_quat(last_s)

        cur_pos = self.interpolator.generate_pos(cur_s)
        last_pos = self.interpolator.generate_pos(last_s)

        ########################################################
        # Computing angular velocities
        ########################################################
        # Quaternion difference to the last step in the inertial frame
        q_diff = qmult(q_cur, qinverse(q_last))
        # Angular velocity
        ang_vel = 2 * q_diff[0:3] / self._t_step

        # Compute linear velocities
        lin_vel = np.array([
            (cur_pos[0] - last_pos[0]) / self._t_step,
            (cur_pos[1] - last_pos[1]) / self._t_step,
            (cur_pos[2] - last_pos[2]) / self._t_step
        ])

        # Apply velocity profiler if available
        if hasattr(self.interpolator, 'get_velocity_at_s'):
            target_speed = self.interpolator.get_velocity_at_s(cur_s)
            if target_speed is not None:
                # Scale linear velocity to match target speed
                current_speed = np.linalg.norm(lin_vel)
                if current_speed > 1e-6:
                    # Log velocity profiler application (very throttled - only once)
                    if not hasattr(self, '_vp_logged'):
                        import sys
                        print(f'[VP] Velocity profiler active (s={cur_s:.2f}, target={target_speed:.2f} m/s)',
                              file=sys.stderr, flush=True)
                        self._vp_logged = True

                    lin_vel = lin_vel * (target_speed / current_speed)

        vel = [lin_vel[0], lin_vel[1], lin_vel[2],
               ang_vel[0], ang_vel[1], ang_vel[2]]
        return np.array(vel)

    def generate_reference(self, t, *args):
        t = max(t, self.interpolator.start_time)
        t = min(t, self.interpolator.max_time)
        pnt = self.generate_pnt(t, *args)
        pnt.t = t
        return pnt

    def interpolate(self, t, *args):
        if not self._has_started:
            tic = time.time()
            if not self.interpolator.init_interpolator():
                self._logger.error('Error initializing the waypoint interpolator')
                return None
            if self.interpolator.start_time is None:
                self.set_start_time(t + (time.time() - tic))
            self.interpolator.s_step = self._t_step / (self.interpolator.max_time - self.interpolator.start_time)
            self.update_dt(t)
            # Generate first point
            self._cur_s = 0
            self._integrated_s = 0.0
            self._last_integration_time = t
            self._has_started = True
            self._has_ended = False

            # Get final waypoint position for distance-based termination
            if hasattr(self.interpolator, 'waypoints'):
                last_wp = self.interpolator.waypoints.get_waypoint(
                    self.interpolator.waypoints.num_waypoints - 1
                )
                if last_wp is not None:
                    self._final_waypoint_pos = np.array([last_wp.x, last_wp.y, last_wp.z])
                    self._logger.info(f'Final waypoint: [{last_wp.x:.2f}, {last_wp.y:.2f}, {last_wp.z:.2f}]')
                    self._logger.info(f'Completion threshold: {self._completion_threshold:.2f}m')

        # Check termination conditions
        use_velocity_integration = (hasattr(self.interpolator, 'get_velocity_at_s') and
                                     hasattr(self.interpolator, '_use_velocity_profiler') and
                                     self.interpolator._use_velocity_profiler)

        # Path parameter-based termination: check if we've completed the path
        # This is robust to different interpolation methods and radius values
        trajectory_finished = False
        if use_velocity_integration:
            # Trajectory finished when path parameter s >= 0.99
            # This works for all interpolation methods (LIPB, cubic, linear)
            trajectory_finished = self._integrated_s >= 0.99

            # Debug log (throttled)
            if not hasattr(self, '_termination_log_count'):
                self._termination_log_count = 0
            if self._termination_log_count % 200 == 0:
                import sys
                print(f'[TERM] s={self._integrated_s:.4f}, threshold=0.99, finished={trajectory_finished}',
                      file=sys.stderr, flush=True)
            self._termination_log_count += 1
        else:
            # Original time-based termination
            trajectory_finished = self.interpolator.has_finished(t)

        if trajectory_finished or not self.interpolator.has_started(t):
            if trajectory_finished:
                self._has_ended = True
                self._cur_s = 1
                self._integrated_s = 1.0
                # Continue publishing final waypoint position
                self._this_pnt = self.generate_pnt(t, *args)
                self._this_pnt.t = t
            else:
                self._this_pnt = self.generate_pnt(0, *args)
                self._this_pnt.vel = np.zeros(6)
                self._this_pnt.acc = np.zeros(6)
                self._this_pnt.t = t
        else:
            self._has_started = True
            self._has_ended = False

            # Retrieving current position and heading
            # Use integrated_s if velocity profiler is active, otherwise use time-based
            use_velocity_integration = (hasattr(self.interpolator, 'get_velocity_at_s') and
                                         hasattr(self.interpolator, '_use_velocity_profiler') and
                                         self.interpolator._use_velocity_profiler)

            if use_velocity_integration:
                # Use velocity-integrated s (calculated in generate_pnt)
                self._this_pnt = self.generate_pnt(t, *args)
                self._cur_s = self._integrated_s  # Use the integrated value!
            else:
                # Original time-based calculation
                self._cur_s = (t - self.interpolator.start_time) / (self.interpolator.max_time - self.interpolator.start_time)
                self._this_pnt = self.generate_pnt(t, *args)

            self._this_pnt.t = t

            self._last_pnt = deepcopy(self._this_pnt)
        return self._this_pnt
