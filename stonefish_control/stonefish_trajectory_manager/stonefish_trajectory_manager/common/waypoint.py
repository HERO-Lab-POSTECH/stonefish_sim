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
from stonefish_control_msgs.msg import Waypoint as WaypointMessage


class Waypoint(object):
    """Waypoint data structure

    > *Attributes*

    * `FINAL_WAYPOINT_COLOR` (*type:* list of `float`, *value:* `[1.0, 0.5737, 0.0]`): RGB color for marker of the final waypoint in RViz
    * `OK_WAYPOINT` (*type:* list of `float`, *value:* `[0.1216, 0.4157, 0.8863]`): RGB color for marker of a successful waypoint in RViz
    * `FAILED_WAYPOINT` (*type:* list of `float`, *value:* `[1.0, 0.0, 0.0]`): RGB color for marker of a failed waypoint in RViz

    > *Input arguments*

    * `x` (*type:* `float`, *default:* `0`): X coordinate in meters
    * `y` (*type:* `float`, *default:* `0`): Y coordinate in meters
    * `z` (*type:* `float`, *default:* `0`): Z coordinate in meters
    * `max_forward_speed` (*type:* `float`, *default:* `0`): **DEPRECATED** - Use robot_max_speed parameter instead
    * `heading_offset` (*type:* `float`, *default:* `0`): Heading offset to be added to the computed heading reference in radians
    * `use_fixed_heading` (*type:* `float`, *default:* `False`): Use the heading offset as a fixed heading reference in radians
    * `inertial_frame_id` (*type:* `str`, *default:* `'world'`): Name of the inertial reference frame. Stonefish uses `world_ned` (NED convention)
    * `radius_acceptance` (*type:* `float`, *default:* `0`): Radius around the waypoint where the vehicle can be considered to have reached the waypoint

    !!! warning "Deprecated"
        `max_forward_speed` is deprecated. Use the global `robot_max_speed` ROS parameter instead.
        This field is kept for backward compatibility but is no longer used by LOS guidance.

    """
    FINAL_WAYPOINT_COLOR = [1.0, 131.0 / 255, 0.0]
    OK_WAYPOINT = [31. / 255, 106. / 255, 226. / 255]
    FAILED_WAYPOINT = [1.0, 0.0, 0.0]

    def __init__(self, x=0, y=0, z=0, max_forward_speed=0, heading_offset=0,
        use_fixed_heading=False, inertial_frame_id='world_ned', radius_acceptance=0.0,
        roll=0.0, pitch=0.0, yaw=0.0):
        # Stonefish uses 'world' (NED) frame
        self._x = x
        self._y = y
        self._z = z
        self._inertial_frame_id = inertial_frame_id
        self._max_forward_speed = max_forward_speed
        self._heading_offset = heading_offset
        self._violates_constraint = False
        self._use_fixed_heading = use_fixed_heading
        self._radius_acceptance = radius_acceptance
        self._heading = 0.0  # Initialize heading attribute

        # 6DOF support: Store roll, pitch, yaw
        self._roll = roll
        self._pitch = pitch
        self._yaw = yaw

    def __eq__(self, other):
        return self._x == other.x and self._y == other.y and self._z == other.z

    def __ne__(self, other):
        return self._x != other.x or self._y != other.y or self._z != other.z

    def __str__(self):
        msg = '(x, y, z)= (%.2f, %.2f, %.2f) m\n' % (self._x, self._y, self._z)
        msg += 'Max. forward speed = %.2f\n' % self._max_forward_speed
        if self._use_fixed_heading:
            msg += 'Heading offset = %.2f degrees\n' % (self._heading_offset * 180 / np.pi)
        return msg

    @property
    def inertial_frame_id(self):
        """`str`: Name of the inertial reference frame"""
        return self._inertial_frame_id

    @inertial_frame_id.setter
    def inertial_frame_id(self, frame_id):
        # Stonefish uses 'world' (NED) frame
        self._inertial_frame_id = frame_id

    @property
    def x(self):
        """`float`: X coordinate of the waypoint in meters"""
        return self._x

    @property
    def y(self):
        """`float`: Y coordinate of the waypoint in meters"""
        return self._y

    @property
    def z(self):
        """`float`: Z coordinate of the waypoint in meters"""
        return self._z

    @property
    def pos(self):
        """`numpy.ndarray`: Position 3D vector"""
        return np.array([self._x, self._y, self._z])

    @pos.setter
    def pos(self, new_pos):
        if isinstance(new_pos, list):
            assert len(new_pos) == 3, 'New position must have three elements'
        elif isinstance(new_pos, np.ndarray):
            assert new_pos.shape == (3,), 'New position must have three elements'
        else:
            raise Exception('Invalid position vector size')
        self._x = new_pos[0]
        self._y = new_pos[1]
        self._z = new_pos[2]

    @property
    def violates_constraint(self):
        """`bool`: Flag on constraint violation for this waypoint"""
        return self._violates_constraint

    @violates_constraint.setter
    def violates_constraint(self, flag):
        self._violates_constraint = flag

    @property
    def max_forward_speed(self):
        """`float`: Maximum reference forward speed"""
        return self._max_forward_speed

    @max_forward_speed.setter
    def max_forward_speed(self, vel):
        self._max_forward_speed = vel

    @property
    def heading_offset(self):
        """`float`: Heading offset in radians"""
        return self._heading_offset

    @property
    def heading(self):
        """`float`: Heading reference stored for this waypoint in radians"""
        return self._heading

    @heading.setter
    def heading(self, angle):
        self._heading = angle

    @property
    def radius_of_acceptance(self):
        """`float`: Radius of acceptance in meters"""
        return self._radius_acceptance

    @radius_of_acceptance.setter
    def radius_of_acceptance(self, radius):
        assert radius >= 0, 'Radius must be greater or equal to zero'
        self._radius_acceptance = radius

    @property
    def using_heading_offset(self):
        """`float`: Flag to use the heading offset"""
        return self._use_fixed_heading

    @property
    def roll(self):
        """`float`: Roll angle in radians"""
        return self._roll

    @roll.setter
    def roll(self, angle):
        self._roll = angle

    @property
    def pitch(self):
        """`float`: Pitch angle in radians"""
        return self._pitch

    @pitch.setter
    def pitch(self, angle):
        self._pitch = angle

    @property
    def yaw(self):
        """`float`: Yaw angle in radians"""
        return self._yaw

    @yaw.setter
    def yaw(self, angle):
        self._yaw = angle

    @property
    def orientation(self):
        """`numpy.ndarray`: Orientation as [roll, pitch, yaw] in radians"""
        return np.array([self._roll, self._pitch, self._yaw])

    @orientation.setter
    def orientation(self, rpy):
        if isinstance(rpy, (list, tuple, np.ndarray)):
            assert len(rpy) == 3, 'Orientation must have 3 elements [roll, pitch, yaw]'
            self._roll = float(rpy[0])
            self._pitch = float(rpy[1])
            self._yaw = float(rpy[2])
        else:
            raise ValueError('Invalid orientation type')

    def get_color(self):
        """Return the waypoint marker's color

        > *Returns*

        RGB color as a `list`
        """
        return (self.FAILED_WAYPOINT if self._violates_constraint else self.OK_WAYPOINT)

    def get_final_color(self):
        """Return the RGB color for the final waypoint

        > *Returns*

        RGB color as a `list`
        """
        return self.FINAL_WAYPOINT_COLOR

    def from_message(self, msg):
        """Set waypoint parameters from `stonefish_control_msgs/Waypoint`
        message

        > *Input arguments*

        * `msg` (*type:* `stonefish_control_msgs/Waypoint`): Waypoint message
        """
        self._inertial_frame_id = msg.header.frame_id
        if len(self._inertial_frame_id) == 0:
            self._inertial_frame_id = 'world_ned'
        self._x = msg.point.x
        self._y = msg.point.y
        self._z = msg.point.z
        self._max_forward_speed = msg.max_forward_speed
        self._use_fixed_heading = msg.use_fixed_heading
        self._heading_offset = msg.heading_offset
        self._radius_acceptance = msg.radius_of_acceptance

    def to_message(self):
        """Convert waypoint to `stonefish_control_msgs/Waypoint` message

        > *Returns*

        `stonefish_control_msgs/Waypoint` message
        """
        wp = WaypointMessage()
        wp.point.x = self._x
        wp.point.y = self._y
        wp.point.z = self._z
        wp.max_forward_speed = self._max_forward_speed
        wp.use_fixed_heading = self._use_fixed_heading
        wp.heading_offset = self._heading_offset
        wp.header.frame_id = self._inertial_frame_id
        wp.radius_of_acceptance = self._radius_acceptance
        return wp

    def dist(self, pos):
        """Compute distance of waypoint to a point

        > *Input arguments*

        * `pos` (*type:* list of `float`): 3D position vector

        > *Returns*

        Distance to point in meters
        """
        return np.sqrt((self._x - pos[0])**2 +
                       (self._y - pos[1])**2 +
                       (self._z - pos[2])**2)

    def calculate_heading(self, target):
        """Compute heading to target waypoint

        > *Input arguments*

        * `target` (*type:* `stonefish_trajectory_manager/Waypoint`): Target waypoint

        > *Returns*

        Heading angle in radians
        """
        dy = target.y - self.y
        dx = target.x - self.x
        return np.arctan2(dy, dx)

    def to_quaternion(self):
        """Convert RPY orientation to quaternion [x, y, z, w].

        Returns:
            Quaternion as numpy array [x, y, z, w]
        """
        from transforms3d.euler import euler2quat
        quat_wxyz = euler2quat(self._roll, self._pitch, self._yaw)
        return np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])

    def distance_to(self, other):
        """Calculate Euclidean distance to another waypoint.

        Args:
            other: Another Waypoint object

        Returns:
            Distance in meters
        """
        return self.dist(other.pos)
