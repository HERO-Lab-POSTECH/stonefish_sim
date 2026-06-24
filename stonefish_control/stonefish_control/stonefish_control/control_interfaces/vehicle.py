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

from __future__ import print_function
from dataclasses import dataclass
import numpy as np
from nav_msgs.msg import Odometry
from copy import deepcopy
from transforms3d.euler import quat2euler, euler2quat
from transforms3d.quaternions import qmult, qconjugate, quat2mat
from ._log import get_logger

try:
    import casadi
    casadi_exists = True
except ImportError:
    casadi_exists = False


@dataclass
class VehicleParams:
    """Vehicle physical parameters loaded from ROS2 node parameters.

    Holds the values that ``Vehicle.__init__`` previously read inline from the
    node's parameter server. Populated by :class:`VehicleParamsLoader`.
    """
    mass: float
    inertial: dict
    cog: list
    cob: list
    body_frame: str
    volume: float
    density: float
    height: float
    length: float
    width: float


class VehicleParamsLoader:
    """Loads :class:`VehicleParams` from a ROS2 node's parameter server.

    Extracted verbatim from the former ``Vehicle.__init__`` parameter block to
    preserve the exact declare/get/has call order, default values, and the
    raise timing of the eight validation ``ValueError``\\ s (behavior-preserving
    dependency inversion — characterized by ``test_characterization_vehicle_params``).
    """

    @staticmethod
    def load(node):
        """Declare, read, and validate vehicle parameters on ``node``.

        Args:
            node: ROS2 node instance for parameter access.

        Returns:
            VehicleParams: validated physical parameters.

        Raises:
            ValueError: on invalid mass/cog/cob/volume/density/height/length/width,
                at the same point in the call sequence as the original ``__init__``.
        """
        # Declare and get parameters
        node.declare_parameter('mass', 0.0)
        mass = node.get_parameter('mass').value
        if mass <= 0:
            raise ValueError('Mass has to be positive')

        # Inertial parameters
        inertial = dict(ixx=0, iyy=0, izz=0, ixy=0, ixz=0, iyz=0)
        node.declare_parameter('inertial.ixx', 0.0)
        node.declare_parameter('inertial.iyy', 0.0)
        node.declare_parameter('inertial.izz', 0.0)
        node.declare_parameter('inertial.ixy', 0.0)
        node.declare_parameter('inertial.ixz', 0.0)
        node.declare_parameter('inertial.iyz', 0.0)

        for key in inertial:
            param_name = f'inertial.{key}'
            if node.has_parameter(param_name):
                inertial[key] = node.get_parameter(param_name).value

        # Center of gravity
        node.declare_parameter('cog', [0.0, 0.0, 0.0])
        cog = node.get_parameter('cog').value
        if len(cog) != 3:
            raise ValueError('Invalid center of gravity vector')

        # Center of buoyancy
        node.declare_parameter('cob', [0.0, 0.0, 0.0])
        cob = node.get_parameter('cob').value
        if len(cob) != 3:
            raise ValueError('Invalid center of buoyancy vector')

        # Body frame name
        node.declare_parameter('base_link', 'base_link')
        body_frame = node.get_parameter('base_link').value

        # Volume
        node.declare_parameter('volume', 0.0)
        volume = node.get_parameter('volume').value
        if volume <= 0:
            raise ValueError('Invalid volume')

        # Fluid density
        node.declare_parameter('density', 1028.0)
        density = node.get_parameter('density').value
        if density <= 0:
            raise ValueError('Invalid fluid density')

        # Bounding box
        node.declare_parameter('height', 0.0)
        node.declare_parameter('length', 0.0)
        node.declare_parameter('width', 0.0)

        height = node.get_parameter('height').value
        if height <= 0:
            raise ValueError('Invalid height')

        length = node.get_parameter('length').value
        if length <= 0:
            raise ValueError('Invalid length')

        width = node.get_parameter('width').value
        if width <= 0:
            raise ValueError('Invalid width')

        return VehicleParams(
            mass=mass, inertial=inertial, cog=cog, cob=cob,
            body_frame=body_frame, volume=volume, density=density,
            height=height, length=length, width=width,
        )


def cross_product_operator(x):
    """Return a cross product operator for the given vector."""
    S = np.array([[0, -x[2], x[1]],
                  [x[2], 0, -x[0]],
                  [-x[1], x[0], 0]])
    return S


class Vehicle(object):
    """Vehicle interface to be used by model-based controllers. It receives the
    parameters necessary to compute the vehicle's motion according to Fossen's.
    """

    _INSTANCE = None

    def __init__(self, node, inertial_frame_id='world_ned'):
        """Class constructor.

        Args:
            node: ROS2 node instance for parameter access
            inertial_frame_id: Inertial frame name (Stonefish uses 'world' for NED)
        """
        self._node = node
        self._logger = node.get_logger()

        # Reading current namespace
        self._namespace = node.get_namespace()

        # Stonefish uses NED (world) and FRD (base_link) frames natively
        # No ENU-NED conversion needed (unlike Gazebo/UUV Simulator)
        self._inertial_frame_id = 'world_ned'
        self._body_frame_id = 'base_link'

        # Declare, read, and validate physical parameters from the node.
        # Extracted to VehicleParamsLoader (behavior-preserving dependency inversion):
        # the loader performs the same declare/get/has calls and raises the same
        # validation errors at the same point in the sequence.
        params = VehicleParamsLoader.load(self._node)
        self._mass = params.mass
        self._inertial = params.inertial
        self._cog = params.cog
        self._cob = params.cob
        self._body_frame = params.body_frame
        self._volume = params.volume
        self._density = params.density
        self._height = params.height
        self._length = params.length
        self._width = params.width

        # Calculating the rigid-body mass matrix
        self._M = np.zeros(shape=(6, 6), dtype=float)
        self._M[0:3, 0:3] = self._mass * np.eye(3)
        self._M[0:3, 3:6] = - self._mass * \
            cross_product_operator(self._cog)
        self._M[3:6, 0:3] = self._mass * \
            cross_product_operator(self._cog)
        self._M[3:6, 3:6] = self._calc_inertial_tensor()

        # Loading the added-mass matrix (optional for non-model-based controllers)
        self._Ma = np.zeros((6, 6))
        # Note: ROS2 doesn't support nested lists in parameters
        # For model-based controllers, use a separate config loading mechanism

        # Sum rigid-body and added-mass matrices
        self._Mtotal = np.zeros(shape=(6, 6))
        self._calc_mass_matrix()

        # Acceleration of gravity
        self._gravity = 9.81

        # Initialize the Coriolis and centripetal matrix
        self._C = np.zeros((6, 6))

        # Vector of restoring forces
        self._g = np.zeros(6)

        # Loading the damping coefficients (optional for non-model-based controllers)
        # For basic PID controllers, these are not used
        self._linear_damping = np.zeros(shape=(6, 6))
        self._quad_damping = np.zeros(shape=(6,))
        self._linear_damping_forward_speed = np.zeros(shape=(6, 6))

        # Note: ROS2 doesn't support nested lists in parameters well
        # For model-based controllers that need these parameters,
        # use a separate YAML loading mechanism

        # Initialize damping matrix
        self._D = np.zeros((6, 6))

        # Vehicle states
        self._pose = dict(pos=np.zeros(3),
                          rot=euler2quat(0, 0, 0))
        # Velocity in the body frame
        self._vel = np.zeros(6)
        # Acceleration in the body frame
        self._acc = np.zeros(6)
        # Generalized forces
        self._gen_forces = np.zeros(6)

    @staticmethod
    def q_to_matrix(q):
        """Convert quaternion into orthogonal rotation matrix.

        Args:
            q: Quaternion vector as (qx, qy, qz, qw)

        Returns:
            numpy.array: Rotation matrix
        """
        e1 = q[0]
        e2 = q[1]
        e3 = q[2]
        eta = q[3]
        R = np.array([[1 - 2 * (e2**2 + e3**2),
                       2 * (e1 * e2 - e3 * eta),
                       2 * (e1 * e3 + e2 * eta)],
                      [2 * (e1 * e2 + e3 * eta),
                       1 - 2 * (e1**2 + e3**2),
                       2 * (e2 * e3 - e1 * eta)],
                      [2 * (e1 * e3 - e2 * eta),
                       2 * (e2 * e3 + e1 * eta),
                       1 - 2 * (e1**2 + e2**2)]])
        return R

    @property
    def namespace(self):
        """`str`: Return robot namespace."""
        return self._namespace

    @property
    def body_frame_id(self):
        """`str`: Body frame ID"""
        return self._body_frame_id

    @property
    def inertial_frame_id(self):
        """`str`: Inertial frame ID"""
        return self._inertial_frame_id

    @property
    def mass(self):
        """`float`: Mass in kilograms"""
        return self._mass

    @property
    def volume(self):
        """`float`: Volume of the vehicle in m^3"""
        return self._volume

    @property
    def gravity(self):
        """`float`: Magnitude of acceleration of gravity m / s^2"""
        return self._gravity

    @property
    def density(self):
        """`float`: Fluid density as kg / m^3"""
        return self._density

    @property
    def height(self):
        """`float`: Height of the vehicle in meters"""
        return self._height

    @property
    def width(self):
        """`float`: Width of the vehicle in meters"""
        return self._width

    @property
    def length(self):
        """`float`: Length of the vehicle in meters"""
        return self._length

    @property
    def pos(self):
        """`numpy.array`: Position of the vehicle in meters."""
        return deepcopy(self._pose['pos'])

    @pos.setter
    def pos(self, position):
        pos = np.array(position)
        if pos.size != 3:
            self._logger.error('Invalid position vector')
        else:
            self._pose['pos'] = pos

    @property
    def depth(self):
        """`numpy.array`: Depth of the vehicle in meters."""
        return deepcopy(np.abs(self._pose['pos'][2]))

    @property
    def heading(self):
        """`float`: Heading of the vehicle in radians."""
        return deepcopy(self.euler[2])

    @property
    def quat(self):
        """`numpy.array`: Orientation quaternion as `(qx, qy, qz, qw)`."""
        return deepcopy(self._pose['rot'])

    @quat.setter
    def quat(self, q):
        q_rot = np.array(q)
        if q_rot.size != 4:
            self._logger.error('Invalid quaternion')
        else:
            self._pose['rot'] = q_rot

    @property
    def quat_dot(self):
        """`numpy.array`: Time derivative of the quaternion vector."""
        return np.dot(self.TBtoIquat, self.vel[3:6])

    @property
    def vel(self):
        """`numpy.array`: Linear and angular velocity vector."""
        return deepcopy(self._vel)

    @vel.setter
    def vel(self, velocity):
        """Set the velocity vector in the BODY frame."""
        v = np.array(velocity)
        if v.size != 6:
            self._logger.error('Invalid velocity vector')
        else:
            self._vel = v

    @property
    def acc(self):
        """`numpy.array`: Linear and angular acceleration vector."""
        return deepcopy(self._acc)

    @property
    def euler(self):
        """`list`: Orientation in Euler angles in radians
        as described in Fossen, 2011.
        """
        # Rotation matrix from BODY to INERTIAL
        rot = self.rotBtoI
        # Roll
        roll = np.arctan2(rot[2, 1], rot[2, 2])
        # Pitch, treating singularity cases
        den = np.sqrt(1 - rot[2, 1]**2)
        pitch = - np.arctan(rot[2, 1] / max(0.001, den))
        # Yaw
        yaw = np.arctan2(rot[1, 0], rot[0, 0])
        return roll, pitch, yaw

    @property
    def euler_dot(self):
        """`numpy.array`: Time derivative of the Euler
        angles in radians.
        """
        return np.dot(self.TItoBeuler, self.vel[3:6])

    @property
    def restoring_forces(self):
        """`numpy.array`: Restoring force vector in N."""
        self._update_restoring()
        return deepcopy(self._g)

    @property
    def Mtotal(self):
        """`numpy.array`: Combined system inertia
        and added-mass matrices.
        """
        return deepcopy(self._Mtotal)

    @property
    def Ctotal(self):
        """`numpy.array`: Combined Coriolis matrix"""
        return deepcopy(self._C)

    @property
    def Dtotal(self):
        """`numpy.array`: Linear and non-linear damping matrix"""
        return deepcopy(self._D)

    @property
    def pose_euler(self):
        """`numpy.array`: Pose as a vector, orientation in Euler angles."""
        roll, pitch, yaw = self.euler
        pose = np.zeros(6)
        pose[0:3] = self.pos
        pose[3] = roll
        pose[4] = pitch
        pose[5] = yaw
        return pose

    @property
    def pose_quat(self):
        """`numpy.array`: Pose as a vector, orientation as quaternion."""
        pose = np.zeros(7)
        pose[0:3] = self.pos
        pose[3:7] = self.quat
        return pose

    @property
    def rotItoB(self):
        """`numpy.array`: Rotation matrix from INERTIAL to BODY frame"""
        return self.rotBtoI.T

    @property
    def rotBtoI(self):
        """`numpy.array`: Rotation from BODY to INERTIAL
        frame using the zyx convention to retrieve Euler
        angles from the quaternion vector (Fossen, 2011).
        """
        # Using the (x, y, z, w) format to describe quaternions
        return self.q_to_matrix(self._pose['rot'])

    @property
    def TItoBeuler(self):
        r, p, y = self.euler
        T = np.array([[1, 0, -np.sin(p)],
                      [0, np.cos(r), np.cos(p) * np.sin(r)],
                      [0, -np.sin(r), np.cos(p) * np.cos(r)]])
        return T

    @property
    def TBtoIeuler(self):
        r, p, y = self.euler
        cp = np.cos(p)
        cp = np.sign(cp) * min(0.001, np.abs(cp))
        T = 1 / cp * np.array(
            [[0, np.sin(r) * np.sin(p), np.cos(r) * np.sin(p)],
             [0, np.cos(r) * np.cos(p), -np.cos(p) * np.sin(r)],
             [0, np.sin(r), np.cos(r)]])
        return T

    @property
    def TBtoIquat(self):
        """
        Return matrix for transformation of BODY-fixed angular velocities in the
        BODY frame in relation to the INERTIAL frame into quaternion rate.
        """
        e1 = self._pose['rot'][0]
        e2 = self._pose['rot'][1]
        e3 = self._pose['rot'][2]
        eta = self._pose['rot'][3]
        T = 0.5 * np.array(
            [[-e1, -e2, -e3],
             [eta, -e3, e2],
             [e3, eta, -e1],
             [-e2, e1, eta]]
        )
        return T

    def to_SNAME(self, x):
        """Convert to SNAME convention (FRD body frame).

        Stonefish already uses FRD (base_link), so no conversion needed.
        This is a no-op for Stonefish (kept for compatibility).
        """
        return x

    def from_SNAME(self, x):
        """Convert from SNAME convention (FRD body frame).

        Stonefish already uses FRD (base_link), so no conversion needed.
        This is a no-op for Stonefish (kept for compatibility).
        """
        return x

    def print_info(self):
        """Print the vehicle's parameters."""
        print('Namespace: {}'.format(self._namespace))
        print('Mass: {0:.3f} kg'.format(self._mass))
        print('System inertia matrix:\n{}'.format(self._M))
        print('Added-mass:\n{}'.format(self._Ma))
        print('M:\n{}'.format(self._Mtotal))
        print('Linear damping: {}'.format(self._linear_damping))
        print('Quad. damping: {}'.format(self._quad_damping))
        print('Center of gravity: {}'.format(self._cog))
        print('Center of buoyancy: {}'.format(self._cob))
        print('Inertial:\n{}'.format(self._calc_inertial_tensor()))

    def _calc_mass_matrix(self):
        self._Mtotal = self._M + self._Ma

    def _update_coriolis(self, vel=None):
        if vel is not None:
            if vel.shape != (6,):
                raise ValueError('Velocity vector has the wrong dimension')
            nu = vel
        else:
            nu = self.to_SNAME(self._vel)

        self._C = np.zeros((6, 6))

        S_12 = - cross_product_operator(
            np.dot(self._Mtotal[0:3, 0:3], nu[0:3]) +
            np.dot(self._Mtotal[0:3, 3:6], nu[3:6]))
        S_22 = - cross_product_operator(
            np.dot(self._Mtotal[3:6, 0:3], nu[0:3]) +
            np.dot(self._Mtotal[3:6, 3:6], nu[3:6]))

        self._C[0:3, 3:6] = S_12
        self._C[3:6, 0:3] = S_12
        self._C[3:6, 3:6] = S_22

    def _update_damping(self, vel=None):
        if vel is not None:
            if vel.shape != (6,):
                raise ValueError('Velocity vector has the wrong dimension')
            # Assume the input velocity is already given in the SNAME convention
            nu = vel
        else:
            nu = self.to_SNAME(self._vel)

        self._D = -1 * self._linear_damping - nu[0] * self._linear_damping_forward_speed
        for i in range(6):
            self._D[i, i] += -1 * self._quad_damping[i] * np.abs(nu[i])

    def _calc_inertial_tensor(self):
        return np.array(
            [[self._inertial['ixx'], self._inertial['ixy'],
              self._inertial['ixz']],
             [self._inertial['ixy'], self._inertial['iyy'],
              self._inertial['iyz']],
             [self._inertial['ixz'], self._inertial['iyz'],
              self._inertial['izz']]])

    def _update_restoring(self, q=None, use_sname=False):
        """
        Update the restoring forces for the current orientation.
        """
        if use_sname:
            Fg = np.array([0, 0, -self._mass * self._gravity])
            Fb = np.array([0, 0, self._volume * self._gravity * self._density])
        else:
            Fg = np.array([0, 0, self._mass * self._gravity])
            Fb = np.array([0, 0, -self._volume * self._gravity * self._density])
        if q is not None:
            rotItoB = self.q_to_matrix(q).T
        else:
            rotItoB = self.rotItoB
        self._g = np.zeros(6)

        self._g[0:3] = -1 * np.dot(rotItoB, Fg + Fb)
        self._g[3:6] = -1 * np.dot(rotItoB,
                                   np.cross(self._cog, Fg) + np.cross(self._cob, Fb))

    def set_added_mass(self, Ma):
        """Set added-mass matrix coefficients."""
        if Ma.shape != (6, 6):
            self._logger.error('Added mass matrix must have dimensions 6x6')
            return False
        self._Ma = np.array(Ma, copy=True)
        self._calc_mass_matrix()
        return True

    def set_damping_coef(self, linear_damping, quad_damping):
        """Set linear and quadratic damping coefficients."""
        if linear_damping.size != 6 or quad_damping.size != 6:
            self._logger.error('Invalid dimensions for damping coefficient vectors')
            return False
        self._linear_damping = np.array(linear_damping, copy=True)
        self._quad_damping = np.array(quad_damping, copy=True)
        return True

    def compute_force(self, acc=None, vel=None, with_restoring=True, use_sname=True):
        """Return the sum of forces acting on the vehicle.

        Given acceleration and velocity vectors, this function returns the
        sum of forces given the rigid-body and hydrodynamic models for the
        marine vessel.
        """
        if acc is not None:
            if acc.shape != (6,):
                raise ValueError('Acceleration vector must have 6 elements')
            # It is assumed the input acceleration is given in the SNAME convention
            nu_dot = acc
        else:
            # Convert the acceleration vector to the SNAME convention (FRD body frame)
            # Note: Stonefish uses FRD (base_link_ned), so to_SNAME() returns as-is
            nu_dot = self.to_SNAME(self._acc)

        if vel is not None:
            if vel.shape != (6,):
                raise ValueError('Velocity vector must have 6 elements')
            # It is assumed the input velocity is given in the SNAME convention
            nu = vel
        else:
            nu = self.to_SNAME(self._vel)

        self._update_damping(nu)
        self._update_coriolis(nu)
        self._update_restoring(use_sname=True)

        if with_restoring:
            g = deepcopy(self._g)
        else:
            g = np.zeros(6)

        f = np.dot(self._Mtotal, nu_dot) + np.dot(self._C, nu) + \
            np.dot(self._D, nu) + g

        if not use_sname:
            f = self.from_SNAME(f)

        return f

    def compute_acc(self, gen_forces=None, use_sname=True):
        """Calculate inverse dynamics to obtain the acceleration vector."""
        self._gen_forces = np.zeros(shape=(6,))
        if gen_forces is not None:
            # It is assumed the generalized forces are given in the SNAME convention
            self._gen_forces = gen_forces
        # Check if the mass and inertial parameters were set
        if self._Mtotal.sum() == 0:
            self._acc = np.zeros(6)
        else:
            nu = self.to_SNAME(self._vel)

            self._update_damping()
            self._update_coriolis()
            self._update_restoring(use_sname=True)
            # Compute the vehicle's acceleration
            self._acc = np.linalg.solve(self._Mtotal, self._gen_forces -
                                        np.dot(self._C, nu) -
                                        np.dot(self._D, nu) -
                                        self._g)
        if not use_sname:
            self._acc = self.from_SNAME(self._acc)

        return self._acc

    def get_jacobian(self):
        """
        Return the Jacobian for the current orientation using transformations
        from BODY to INERTIAL frame.
        """
        jac = np.zeros(shape=(6, 6))
        # Build the Jacobian matrix
        jac[0:3, 0:3] = self.rotBtoI
        jac[3:6, 3:6] = self.TBtoIeuler
        return jac

    def update_odometry(self, msg):
        """Odometry topic subscriber callback function."""
        # The frames of reference delivered by the odometry seems to be as
        # follows
        # position -> world frame
        # orientation -> world frame
        # linear velocity -> world frame
        # angular velocity -> world frame

        # Stonefish uses "world" frame with NED (North-East-Down) convention
        # X=North, Y=East, Z=Down (underwater is positive Z)
        if self._inertial_frame_id != msg.header.frame_id:
            raise ValueError(f'Inertial frame ID mismatch: '
                           f'vehicle={self._inertial_frame_id}, odom={msg.header.frame_id}')

        # Update the velocity vector
        # Update the pose in the inertial frame
        self._pose['pos'] = np.array([msg.pose.pose.position.x,
                                      msg.pose.pose.position.y,
                                      msg.pose.pose.position.z])

        # Using the (x, y, z, w) format for quaternions
        self._pose['rot'] = np.array([msg.pose.pose.orientation.x,
                                      msg.pose.pose.orientation.y,
                                      msg.pose.pose.orientation.z,
                                      msg.pose.pose.orientation.w])
        # Linear velocity on the INERTIAL frame
        lin_vel = np.array([msg.twist.twist.linear.x,
                            msg.twist.twist.linear.y,
                            msg.twist.twist.linear.z])
        # Transform linear velocity to the BODY frame
        lin_vel = np.dot(self.rotItoB, lin_vel)
        # Angular velocity in the INERTIAL frame
        ang_vel = np.array([msg.twist.twist.angular.x,
                            msg.twist.twist.angular.y,
                            msg.twist.twist.angular.z])
        # Transform angular velocity to BODY frame
        ang_vel = np.dot(self.rotItoB, ang_vel)
        # Store velocity vector
        self._vel = np.hstack((lin_vel, ang_vel))
