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
import numpy as np
from stonefish_control_msgs.msg import TrajectoryPoint as TrajectoryPointMsg
import geometry_msgs.msg as geometry_msgs
from transforms3d.euler import quat2euler, euler2quat
from transforms3d.quaternions import mat2quat


class TrajectoryPoint(object):
    """Trajectory point data structure.

    > *Input arguments*

    * `t` (*type:* `float`, *value:* `0`): Timestamp
    * `pos` (*type:* list of `float` or `numpy.array`, *default:* `[0, 0, 0]`):
    3D position vector in meters
    * `quat` (*type:* list of `float` or `numpy.array`, *default:* `[0, 0, 0, 1]`):
    Quaternion in the form of `(w, x, y, z)` for transforms3d.
    * `lin_vel` (*type:* list of `float` or `numpy.array`, *default:* `[0, 0, 0]`):
    3D linear velocity vector in m/s
    * `ang_vel` (*type:* list of `float` or `numpy.array`, *default:* `[0, 0, 0]`):
    3D angular velocity vector as rad/s
    * `lin_acc` (*type:* list of `float` or `numpy.array`, *default:* `[0, 0, 0]`):
    3D linear acceleration vector as m/s$^2$
    * `ang_acc` (*type:* list of `float` or `numpy.array`, *default:* `[0, 0, 0]`):
    3D angular acceleration vector as rad/s$^2$
    """
    def __init__(self, t=0.0, pos=[0, 0, 0], quat=[1, 0, 0, 0],
                 lin_vel=[0, 0, 0], ang_vel=[0, 0, 0], lin_acc=[0, 0, 0],
                 ang_acc=[0, 0, 0]):
        self._pos = np.array(pos)
        # Store as (w, x, y, z) for transforms3d
        if len(quat) == 4:
            self._rot = np.array(quat)
        else:
            self._rot = np.array([1, 0, 0, 0])
        self._vel = np.hstack((lin_vel, ang_vel))
        self._acc = np.hstack((lin_acc, ang_acc))
        self._t = t

    def __str__(self):
        msg = 'Time [s] = {}\n'.format(self._t)
        msg += 'Position [m] = ({}, {}, {})\n'.format(self._pos[0], self._pos[1], self._pos[2])
        eu = [a * 180 / np.pi for a in self.rot]
        msg += 'Rotation [degrees] = ({}, {}, {})\n'.format(eu[0], eu[1], eu[2])
        msg += 'Lin. velocity [m/s] = ({}, {}, {})\n'.format(self._vel[0], self._vel[1], self._vel[2])
        msg += 'Ang. velocity [rad/s] = ({}, {}, {})\n'.format(self._vel[3], self._vel[4], self._vel[5])
        return msg

    def __eq__(self, pnt):
        return self._t == pnt._t and np.array_equal(self._pos, pnt._pos) and \
            np.array_equal(self._rot, pnt._rot) and \
            np.array_equal(self._vel, pnt._vel) and \
            np.array_equal(self._acc, pnt._acc)

    @property
    def p(self):
        """`numpy.array`: Position vector"""
        return self._pos

    @property
    def q(self):
        """`numpy.array`: Quaternion vector as `(w, x, y, z)`"""
        return self._rot

    @property
    def v(self):
        """`numpy.array`: Linear velocity vector"""
        return self._vel[0:3]

    @property
    def w(self):
        """`numpy.array`: Angular velocity vector"""
        return self._vel[3::]

    @property
    def a(self):
        """`numpy.array`: Linear acceleration vector"""
        return self._acc[0:3]

    @property
    def alpha(self):
        """`numpy.array`: Angular acceleartion vector"""
        return self._acc[3::]

    @property
    def x(self):
        """`float`: X coordinate of position vector"""
        return self._pos[0]

    @x.setter
    def x(self, x):
        self._pos[0] = x

    @property
    def y(self):
        """`float`: Y coordinate of position vector"""
        return self._pos[1]

    @y.setter
    def y(self, y):
        self._pos[1] = y

    @property
    def z(self):
        """`float`: Z coordinate of position vector"""
        return self._pos[2]

    @z.setter
    def z(self, z):
        self._pos[2] = z

    @property
    def t(self):
        """`float`: Time stamp"""
        return self._t

    @t.setter
    def t(self, new_t):
        self._t = new_t

    @property
    def pos(self):
        """`numpy.array`: Position vector"""
        return self._pos

    @pos.setter
    def pos(self, new_pos):
        self._pos = np.array(new_pos)

    @property
    def rot(self):
        """`numpy.array`: `roll`, `pitch` and `yaw` angles"""
        # quat2euler expects (w, x, y, z) - our internal format is already correct
        rpy = quat2euler(self._rot, axes='sxyz')
        return np.array([rpy[0], rpy[1], rpy[2]])

    @rot.setter
    def rot(self, new_rot):
        # euler2quat returns (w, x, y, z)
        self._rot = np.array(euler2quat(new_rot[0], new_rot[1], new_rot[2], axes='sxyz'))

    @property
    def rot_matrix(self):
        """`numpy.array`: Rotation matrix"""
        from transforms3d.quaternions import quat2mat
        return quat2mat(self._rot)

    @property
    def rotq(self):
        """`numpy.array`: Quaternion vector as `(w, x, y, z)`"""
        return self._rot

    @rotq.setter
    def rotq(self, quat):
        self._rot = np.array(quat)

    @property
    def vel(self):
        """`numpy.array`: 6D velocity vector (linear + angular)"""
        return self._vel

    @vel.setter
    def vel(self, new_vel):
        self._vel = np.array(new_vel)

    @property
    def acc(self):
        """`numpy.array`: 6D acceleration vector (linear + angular)"""
        return self._acc

    @acc.setter
    def acc(self, new_acc):
        self._acc = np.array(new_acc)

    def to_message(self):
        """Convert current data to a trajectory point message.

        > *Returns*

        Trajectory point message as `stonefish_control_msgs/TrajectoryPoint`
        """
        p_msg = TrajectoryPointMsg()
        # Convert timestamp
        p_msg.header.stamp.sec = int(self.t)
        p_msg.header.stamp.nanosec = int((self.t - int(self.t)) * 1e9)

        p_msg.pose.position = geometry_msgs.Point(x=float(self.p[0]), y=float(self.p[1]), z=float(self.p[2]))
        # ROS quaternion is (x, y, z, w), our internal is (w, x, y, z)
        p_msg.pose.orientation = geometry_msgs.Quaternion(
            x=float(self.q[1]), y=float(self.q[2]), z=float(self.q[3]), w=float(self.q[0]))

        p_msg.velocity.linear = geometry_msgs.Vector3(x=float(self.v[0]), y=float(self.v[1]), z=float(self.v[2]))
        p_msg.velocity.angular = geometry_msgs.Vector3(
            x=float(self.w[0]), y=float(self.w[1]), z=float(self.w[2]))

        p_msg.acceleration.linear = geometry_msgs.Vector3(x=float(self.a[0]), y=float(self.a[1]), z=float(self.a[2]))
        p_msg.acceleration.angular = geometry_msgs.Vector3(
            x=float(self.alpha[0]), y=float(self.alpha[1]), z=float(self.alpha[2]))

        return p_msg

    def from_message(self, msg):
        """Parse a trajectory point message of type `stonefish_control_msgs/TrajectoryPoint`
        into the `stonefish_trajectory_manager/TrajectoryPoint`.

        > *Input arguments*

        * `msg` (*type:* `stonefish_control_msgs/TrajectoryPoint`): Input trajectory message
        """
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        q = msg.pose.orientation
        v = msg.velocity.linear
        w = msg.velocity.angular
        a = msg.acceleration.linear
        al = msg.acceleration.angular

        self._t = t
        self._pos = np.array([p.x, p.y, p.z])
        # Convert ROS quaternion (x, y, z, w) to internal (w, x, y, z)
        self._rot = np.array([q.w, q.x, q.y, q.z])
        self._vel = np.array([v.x, v.y, v.z, w.x, w.y, w.z])
        self._acc = np.array([a.x, a.y, a.z, al.x, al.y, al.z])
        return True

    def from_dict(self, data):
        """Initialize the trajectory point attributes from a `dict`.

        > *Input arguments*

        * `data` (*type:* `dict`): Trajectory point as a `dict`
        """
        self._t = data['time']
        self._pos = np.array(data['pos'])
        rot_data = np.array(data['rot'])
        if rot_data.size == 3:
            # Assume RPY, convert to quaternion (w, x, y, z)
            self._rot = np.array(euler2quat(rot_data[0], rot_data[1], rot_data[2], axes='sxyz'))
        else:
            # Assume quaternion
            self._rot = rot_data
        self._vel = np.array(data['vel'])
        self._acc = np.array(data['acc'])

    def to_dict(self):
        """Convert trajectory point to `dict`.

        > *Returns*

        Trajectory points data as a `dict`
        """
        data = dict(time=self._t,
                    pos=self._pos.tolist(),
                    rot=self._rot.tolist(),
                    vel=self._vel.tolist(),
                    acc=self._acc.tolist())
        return data

    def to_4dof_message(self):
        """Convert to ROS2 TrajectoryPoint message with 4DOF (Roll=0, Pitch=0).

        For ROV control where roll and pitch should remain level.

        Returns:
            stonefish_control_msgs.msg.TrajectoryPoint with roll=0, pitch=0
        """
        from transforms3d.euler import quat2euler, euler2quat

        # Get current orientation
        r, p, y = quat2euler(self.rotq)

        # Force roll=0, pitch=0, keep yaw only (4DOF)
        quat_4dof_wxyz = euler2quat(0.0, 0.0, y)

        # Create ROS2 message
        msg = TrajectoryPointMsg()

        # Timestamp
        msg.header.stamp.sec = int(self.t)
        msg.header.stamp.nanosec = int((self.t - int(self.t)) * 1e9)
        msg.header.frame_id = 'world_ned'

        # Position
        msg.pose.position = geometry_msgs.Point(
            x=float(self.p[0]), 
            y=float(self.p[1]), 
            z=float(self.p[2])
        )

        # Orientation (4DOF: roll=0, pitch=0, yaw only)
        msg.pose.orientation = geometry_msgs.Quaternion(
            x=float(quat_4dof_wxyz[1]),
            y=float(quat_4dof_wxyz[2]),
            z=float(quat_4dof_wxyz[3]),
            w=float(quat_4dof_wxyz[0])
        )

        # Velocity (4DOF: wx=0, wy=0, wz only)
        msg.velocity.linear = geometry_msgs.Vector3(
            x=float(self.v[0]), 
            y=float(self.v[1]), 
            z=float(self.v[2])
        )
        msg.velocity.angular = geometry_msgs.Vector3(
            x=0.0,  # Roll rate = 0
            y=0.0,  # Pitch rate = 0
            z=float(self.w[2])  # Yaw rate only
        )

        # Acceleration (4DOF: αx=0, αy=0, αz only)
        msg.acceleration.linear = geometry_msgs.Vector3(
            x=float(self.a[0]),
            y=float(self.a[1]),
            z=float(self.a[2])
        )
        msg.acceleration.angular = geometry_msgs.Vector3(
            x=0.0,  # Roll accel = 0
            y=0.0,  # Pitch accel = 0
            z=float(self.alpha[2])  # Yaw accel only
        )

        return msg

