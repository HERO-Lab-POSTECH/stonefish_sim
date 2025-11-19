# Copyright (c) 2016-2019 The UUV Simulator Authors.
# All rights reserved.
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
#
# ROS2 Port: Copyright 2025

from copy import deepcopy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import WrenchStamped, Vector3, Quaternion, Point, Pose, Twist, Accel, Vector3Stamped, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Float64
from .vehicle import Vehicle
from transforms3d.quaternions import qmult, qinverse
from stonefish_control_msgs.msg import TrajectoryPoint
from stonefish_control_msgs.srv import ResetController
from rcl_interfaces.msg import SetParametersResult


class DPControllerBase(Node):
    """General abstract class for DP controllers for underwater vehicles.
    This is an abstract class, must be inherited by a controller module that
    overrides the update_controller method. If the controller is set to be
    model based (is_model_based=True), than the vehicle parameters are going
    to be read from the ROS parameter server.

    Args:
        node_name: Name of the ROS2 node
        is_model_based: If True, the controller uses a model of the vehicle,
                        False if it is non-model-based.
        list_odometry_callbacks: List of function handles or lambda functions
                                 that will be called after each odometry update.

    ROS2 Parameters:
        saturation: Absolute saturation of the control signal (default: 5000)

    ROS2 Publishers:
        thruster_output: Control set-point for the thruster manager node
                        (geometry_msgs/WrenchStamped)
        reference: Current reference trajectory point
                   (stonefish_control_msgs/TrajectoryPoint)
        error: Current trajectory error (stonefish_control_msgs/TrajectoryPoint)

    ROS2 Services:
        reset_controller: Reset all variables, including error and reference
                         signals (stonefish_control_msgs/ResetController)
    """

    _LABEL = ''

    def __init__(self, node_name, is_model_based=False,
                 list_odometry_callbacks=None):
        super().__init__(node_name)

        # Flag will be set to true when all parameters are initialized correctly
        self._is_init = False

        # Reading current namespace
        self._namespace = self.get_namespace()

        # Configuration for the vehicle dynamic model
        self._is_model_based = is_model_based

        # Declare parameters
        self.declare_parameter('saturation', 100.0)
        self._control_saturation = self.get_parameter('saturation').value
        if self._control_saturation <= 0:
            raise ValueError('Invalid control saturation forces')

        # Reference setpoint parameters (fallback when no reference topic)
        self.declare_parameter('ref_position', [0.0, 0.0, 0.0])
        self.declare_parameter('ref_orientation', [0.0, 0.0, 0.0])  # [roll, pitch, yaw] in radians

        # Cache parameter values (updated via callback, not read every odometry)
        # This prevents YAML default values from overriding optimizer-set values
        self._ref_position_param = self.get_parameter('ref_position').value
        self._ref_orientation_param = self.get_parameter('ref_orientation').value

        # Flag to track if manual reference was explicitly set (via YAML or runtime)
        # This allows distinguishing between:
        # - ref_position=[0,0,0] (station-keeping) vs
        # - ref_position=[0,0,0] set by optimizer (manual, go to origin)
        self._manual_ref_set = False

        # If YAML has non-zero reference, consider it manually set
        if any(p != 0.0 for p in self._ref_position_param):
            self._manual_ref_set = True

        self._use_manual_reference = False

        # QoS profile for publishers and subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher for thruster allocator
        self._thrust_pub = self.create_publisher(
            WrenchStamped, 'thruster_manager/input_stamped', qos_profile)

        # Publish error (for debugging and tuning tool)
        self._error_pub = self.create_publisher(
            TrajectoryPoint, 'error', qos_profile)

        # Subscribe to cmd_pose topic (from trajectory_follower, LOS guidance, or optimizer)
        # Uses TrajectoryPoint (pos + vel + acc) for velocity tracking
        # Relative name: resolved to /{namespace}/cmd_pose
        self._cmd_pose_sub = self.create_subscription(
            TrajectoryPoint,
            'cmd_pose',
            self._cmd_pose_callback,
            qos_profile)

        # Track when last reference was received
        self._last_reference_time = self.get_clock().now()
        self._reference_timeout = 1.0  # seconds
        self._use_topic_reference = False

        self._init_reference = False

        # Reference with relation to the INERTIAL frame
        # Initialize with identity quaternion [qx, qy, qz, qw]
        self._reference = dict(pos=np.zeros(3),
                               rot=np.array([0, 0, 0, 1]),
                               vel=np.zeros(6),
                               acc=np.zeros(6))

        # Errors with relation to the BODY frame
        self._errors = dict(pos=np.zeros(3),
                            rot=np.zeros(4),
                            vel=np.zeros(6))

        # Time step
        self._dt = 0
        self._prev_time = self.get_clock().now()

        # Services
        self._reset_service = self.create_service(
            ResetController,
            'reset_controller',
            self.reset_controller_callback)

        # Time stamp for the received trajectory
        self._stamp_trajectory_received = self.get_clock().now()

        # Instance of the vehicle model
        self._vehicle_model = None

        # If list of callbacks is empty, set the default
        if list_odometry_callbacks is not None and \
                isinstance(list_odometry_callbacks, list):
            self._odometry_callbacks = list_odometry_callbacks
        else:
            self._odometry_callbacks = [self.update_errors,
                                        self.update_controller]

        # Initialize vehicle, if model based
        self._create_vehicle_model()

        # Flag to indicate that odometry topic is receiving data
        self._init_odom = False

        # Subscribe to odometry topic (Stonefish publishes to 'odometry')
        self._odom_topic_sub = self.create_subscription(
            Odometry,
            'odometry',
            self._odometry_callback,
            qos_profile)

        # Stores last simulation time
        self._prev_t = -1.0

        # Add parameter callback for dynamic reconfiguration
        self.add_on_set_parameters_callback(self._on_parameter_event)

    @staticmethod
    def get_controller(name, *args):
        """Create instance of a specific DP controller."""
        for controller in DPControllerBase.__subclasses__():
            if name == controller.__name__:
                return controller(*args)
        return None

    @staticmethod
    def get_list_of_controllers():
        """Return list of DP controllers using this interface."""
        return [controller.__name__ for controller in
                DPControllerBase.__subclasses__()]

    @property
    def label(self):
        """`str`: Identifier name of the controller"""
        return self._LABEL

    @property
    def odom_is_init(self):
        """`bool`: `True` if the first odometry message was received"""
        return self._init_odom

    @property
    def error_pos_world(self):
        """`numpy.array`: Position error wrt world frame"""
        return np.dot(self._vehicle_model.rotBtoI, self._errors['pos'])

    @property
    def error_orientation_quat(self):
        """`numpy.array`: Orientation error"""
        return deepcopy(self._errors['rot'][0:3])

    @property
    def error_orientation_rpy(self):
        """`numpy.array`: Orientation error in Euler angles."""
        e1 = self._errors['rot'][0]
        e2 = self._errors['rot'][1]
        e3 = self._errors['rot'][2]
        eta = self._errors['rot'][3]
        rot = np.array([[1 - 2 * (e2**2 + e3**2),
                         2 * (e1 * e2 - e3 * eta),
                         2 * (e1 * e3 + e2 * eta)],
                        [2 * (e1 * e2 + e3 * eta),
                         1 - 2 * (e1**2 + e3**2),
                         2 * (e2 * e3 - e1 * eta)],
                        [2 * (e1 * e3 - e2 * eta),
                         2 * (e2 * e3 + e1 * eta),
                         1 - 2 * (e1**2 + e2**2)]])
        # Roll
        roll = np.arctan2(rot[2, 1], rot[2, 2])
        # Pitch, treating singularity cases
        den = np.sqrt(1 - rot[2, 1]**2)
        pitch = - np.arctan(rot[2, 1] / max(0.001, den))
        # Yaw
        yaw = np.arctan2(rot[1, 0], rot[0, 0])
        return np.array([roll, pitch, yaw])

    @property
    def error_pose_euler(self):
        """`numpy.array`: Pose error with orientation represented in Euler angles."""
        return np.hstack((self._errors['pos'], self.error_orientation_rpy))

    @property
    def error_vel_world(self):
        """`numpy.array`: Linear velocity error"""
        return np.dot(self._vehicle_model.rotBtoI, self._errors['vel'])

    def __str__(self):
        msg = 'Dynamic positioning controller\n'
        msg += 'Controller= ' + self._LABEL + '\n'
        msg += 'Is model based? ' + str(self._is_model_based) + '\n'
        msg += 'Vehicle namespace= ' + self._namespace
        return msg

    def _create_vehicle_model(self):
        """Create a new instance of a vehicle model. If controller is not model
        based, this model will have its parameters set to 0 and will be used
        to receive and transform the odometry data.
        """
        if self._vehicle_model is not None:
            del self._vehicle_model
        # Stonefish uses 'world' (NED) frame
        self._vehicle_model = Vehicle(
            node=self,
            inertial_frame_id='world_ned')

    def _update_reference(self, pos=None, quat=None, vel=None, acc=None):
        """Update reference trajectory point.

        This is a simplified version for manual setpoint updates.
        For full trajectory tracking, use the local planner.

        Args:
            pos: Position reference [x, y, z]
            quat: Quaternion reference [qx, qy, qz, qw]
            vel: Velocity reference [vx, vy, vz, wx, wy, wz]
            acc: Acceleration reference [ax, ay, az, alpha_x, alpha_y, alpha_z]
        """
        if pos is not None:
            self._reference['pos'] = np.array(pos)
        if quat is not None:
            self._reference['rot'] = np.array(quat)
        if vel is not None:
            self._reference['vel'] = np.array(vel)
        if acc is not None:
            self._reference['acc'] = np.array(acc)

        # Reference topic removed - not needed (cmd_pose serves the same purpose)
        return True

    def _update_time_step(self):
        """Update time step."""
        t = self.get_clock().now()
        dt_duration = t - self._prev_time
        self._dt = dt_duration.nanoseconds / 1e9  # Convert to seconds
        self._prev_time = t

    def _reset_controller(self):
        """Reset reference and and error vectors."""
        self._init_reference = False

        # Reference with relation to the INERTIAL frame
        # Initialize with identity quaternion [qx, qy, qz, qw]
        self._reference = dict(pos=np.zeros(3),
                               rot=np.array([0, 0, 0, 1]),
                               vel=np.zeros(6),
                               acc=np.zeros(6))

        # Errors with relation to the BODY frame
        self._errors = dict(pos=np.zeros(3),
                            rot=np.zeros(4),
                            vel=np.zeros(6))

    def reset_controller_callback(self, request, response):
        """Service handler function."""
        self._reset_controller()
        response.success = True
        return response

    def _cmd_pose_callback(self, msg):
        """Callback for cmd_pose topic (TrajectoryPoint from trajectory_follower or optimizer).

        Receives full 6DOF reference with velocity and acceleration.
        Position, Orientation, Acceleration: World frame (NED)
        Velocity (linear + angular): Body frame (FRD)

        Args:
            msg: TrajectoryPoint message with pos, vel, acc
        """
        # Update timestamp
        self._last_reference_time = self.get_clock().now()
        self._use_topic_reference = True

        # Extract position (World frame - NED)
        self._reference['pos'] = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        # Extract orientation quaternion [x, y, z, w]
        self._reference['rot'] = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])

        # Extract velocity (Body frame - FRD)
        # Following Fossen (2011) convention: velocity reference in body frame
        self._reference['vel'] = np.array([
            msg.velocity.linear.x,
            msg.velocity.linear.y,
            msg.velocity.linear.z,
            msg.velocity.angular.x,
            msg.velocity.angular.y,
            msg.velocity.angular.z
        ])

        # Extract acceleration (World frame - NED)
        self._reference['acc'] = np.array([
            msg.acceleration.linear.x,
            msg.acceleration.linear.y,
            msg.acceleration.linear.z,
            msg.acceleration.angular.x,
            msg.acceleration.angular.y,
            msg.acceleration.angular.z
        ])

        # Mark reference as initialized
        self._init_reference = True

    def _on_parameter_event(self, params):
        """Callback for parameter changes (dynamic reconfigure).

        Allows runtime adjustment of reference via ros2 param set.
        Example:
            ros2 param set /bluerov2/pid_controller ref_position "[2.0, 3.0, -1.5]"
            ros2 param set /bluerov2/pid_controller ref_orientation "[0.0, 0.0, 1.57]"
        """
        result = SetParametersResult(successful=True)

        for param in params:
            if param.name == 'ref_position':
                ref_pos = param.value
                if len(ref_pos) == 3:
                    # Update cached parameter value
                    self._ref_position_param = ref_pos
                    # Mark as manually set (optimizer or user explicitly changed it)
                    self._manual_ref_set = True
                    self.get_logger().info(f'Reference position updated to: {ref_pos}')
                else:
                    result.successful = False
                    result.reason = 'ref_position must have 3 values [x, y, z]'

            elif param.name == 'ref_orientation':
                ref_orient = param.value
                if len(ref_orient) == 3:
                    # Update cached parameter value
                    self._ref_orientation_param = ref_orient
                    # Mark as manually set
                    self._manual_ref_set = True
                    self.get_logger().info(f'Reference orientation updated to: {ref_orient} rad [roll, pitch, yaw]')
                else:
                    result.successful = False
                    result.reason = 'ref_orientation must have 3 values [roll, pitch, yaw]'

        return result

    def update_controller(self):
        """This function must be implemented by derived classes
        with the implementation of the control algorithm.
        """
        # Does nothing, must be overloaded
        raise NotImplementedError()

    def update_errors(self):
        """Update error vectors."""
        if not self.odom_is_init:
            self.get_logger().warning('Odometry topic has not been updated yet',
                                      throttle_duration_sec=5.0)
            return

        # Update reference (publishes reference message)
        self._update_reference()

        # Calculate error in the BODY frame
        self._update_time_step()

        # Rotation matrix from INERTIAL to BODY frame
        rotItoB = self._vehicle_model.rotItoB
        rotBtoI = self._vehicle_model.rotBtoI

        if self._dt > 0:
            # Update position error with respect to the the BODY frame
            pos = self._vehicle_model.pos
            vel = self._vehicle_model.vel
            quat = self._vehicle_model.quat
            self._errors['pos'] = np.dot(
                rotItoB, self._reference['pos'] - pos)

            # Update orientation error
            # Convert [x,y,z,w] to [w,x,y,z] for transforms3d
            quat_wxyz = np.array([quat[3], quat[0], quat[1], quat[2]])
            ref_wxyz = np.array([self._reference['rot'][3], self._reference['rot'][0],
                                 self._reference['rot'][1], self._reference['rot'][2]])

            err_wxyz = qmult(qinverse(quat_wxyz), ref_wxyz)

            # Convert back to [x,y,z,w]
            self._errors['rot'] = np.array([err_wxyz[1], err_wxyz[2], err_wxyz[3], err_wxyz[0]])

            # DEBUG: Log orientation data periodically
            if hasattr(self, '_debug_counter'):
                self._debug_counter += 1
            else:
                self._debug_counter = 0

            # Velocity error with respect to the BODY frame
            # Reference velocity is already in body frame (Fossen 2011 convention)
            self._errors['vel'] = self._reference['vel'] - vel

        if self._error_pub.get_subscription_count() > 0:
            stamp = self.get_clock().now()
            msg = TrajectoryPoint()
            msg.header.stamp = stamp.to_msg()
            msg.header.frame_id = self._vehicle_model.inertial_frame_id

            # Publish pose error
            pos_error_world = np.dot(rotBtoI, self._errors['pos'])
            msg.pose = Pose()
            msg.pose.position = Point(
                x=float(pos_error_world[0]),
                y=float(pos_error_world[1]),
                z=float(pos_error_world[2]))
            msg.pose.orientation = Quaternion(
                x=float(self._errors['rot'][0]),
                y=float(self._errors['rot'][1]),
                z=float(self._errors['rot'][2]),
                w=float(self._errors['rot'][3]))

            # Publish velocity errors in INERTIAL frame
            vel_error_world = np.dot(rotBtoI, self._errors['vel'][0:3])
            ang_vel_error_world = np.dot(rotBtoI, self._errors['vel'][3:6])
            msg.velocity = Twist()
            msg.velocity.linear = Vector3(
                x=float(vel_error_world[0]),
                y=float(vel_error_world[1]),
                z=float(vel_error_world[2]))
            msg.velocity.angular = Vector3(
                x=float(ang_vel_error_world[0]),
                y=float(ang_vel_error_world[1]),
                z=float(ang_vel_error_world[2]))

            # Acceleration is not computed in basic controller
            msg.acceleration = Accel()

            self._error_pub.publish(msg)

    def publish_control_wrench(self, force):
        """Publish the thruster manager control set-point.

        Args:
            force: 6 DoF control set-point wrench vector
        """
        if not self.odom_is_init:
            return

        # Apply saturation
        force = np.array(force)
        for i in range(6):
            if force[i] < -self._control_saturation:
                force[i] = -self._control_saturation
            elif force[i] > self._control_saturation:
                force[i] = self._control_saturation

        force_msg = WrenchStamped()
        force_msg.header.stamp = self.get_clock().now().to_msg()
        force_msg.header.frame_id = f'{self._namespace}/{self._vehicle_model.body_frame_id}'
        force_msg.wrench.force.x = float(force[0])
        force_msg.wrench.force.y = float(force[1])
        force_msg.wrench.force.z = float(force[2])

        force_msg.wrench.torque.x = float(force[3])
        force_msg.wrench.torque.y = float(force[4])
        force_msg.wrench.torque.z = float(force[5])

        self._thrust_pub.publish(force_msg)

    def _odometry_callback(self, msg):
        """Odometry topic subscriber callback function.

        Args:
            msg: Input odometry message (nav_msgs/Odometry)
        """
        self._vehicle_model.update_odometry(msg)

        if not self._init_odom:
            self._init_odom = True

        # Check if reference topic is active (received within timeout)
        time_since_ref = (self.get_clock().now() - self._last_reference_time).nanoseconds / 1e9
        topic_is_active = time_since_ref < self._reference_timeout

        if not topic_is_active:
            # No active topic, use parameter fallback or station-keeping
            ref_pos = self._ref_position_param
            ref_orient = self._ref_orientation_param

            # Check if parameter reference is set (non-zero)
            param_is_set = (any(p != 0.0 for p in ref_pos) or
                           any(o != 0.0 for o in ref_orient))

            if param_is_set or self._manual_ref_set:
                # Use parameter reference (6DOF)
                from transforms3d.euler import euler2quat
                self._reference['pos'] = np.array(ref_pos)
                # Convert roll/pitch/yaw to quaternion
                quat_wxyz = euler2quat(ref_orient[0], ref_orient[1], ref_orient[2])
                self._reference['rot'] = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])  # [x,y,z,w]
                self._init_reference = True
            else:
                # Station-keeping mode: track current pose
                if not self._init_reference:
                    self._reference['pos'] = self._vehicle_model.pos
                    self._reference['rot'] = self._vehicle_model.quat
                    self._init_reference = True

        # Reference topic callback already sets pos/rot if active

        self._reference['vel'] = np.zeros(6)
        self._reference['acc'] = np.zeros(6)

        if len(self._odometry_callbacks):
            for func in self._odometry_callbacks:
                func()
