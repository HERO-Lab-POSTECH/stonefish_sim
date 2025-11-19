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
from __future__ import print_function
import numpy as np
import os
import yaml
from .waypoint import Waypoint
from stonefish_control_msgs.msg import WaypointSet as WaypointSetMessage
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time as TimeMsg


class WaypointSet(object):
    """Set of waypoints.

    > *Attributes*

    * `FINAL_WAYPOINT_COLOR` (*type:* list of `float`, *value:* `[1.0, 0.5737, 0.0]`): RGB color for marker of the final waypoint in RViz
    * `OK_WAYPOINT` (*type:* list of `float`, *value:* `[0.1216, 0.4157, 0.8863]`): RGB color for marker of a successful waypoint in RViz
    * `FAILED_WAYPOINT` (*type:* list of `float`, *value:* `[1.0, 0.0, 0.0]`): RGB color for marker of a failed waypoint in RViz

    > *Input arguments*

    * `scale` (*type:* `float`, *default:* `0.1`): Scale of the spherical marker for waypoints
    * `inertial_frame_id` (*type:* `str`, *default:* `'world'`): Name of the inertial reference frame. Stonefish uses `world_ned` (NED convention)
    * `max_surge_speed` (*type:* `float`, *default:* `None`): Max. surge speed in m/s associated with each waypoint
    * `clock` (*type:* `rclpy.clock.Clock`, *default:* `None`): ROS2 clock for timestamps (optional, for ROS-independent usage)

    """
    FINAL_WAYPOINT_COLOR = [1.0, 0.5737, 0.0]
    OK_WAYPOINT = [31. / 255, 106. / 255, 226. / 255]
    FAILED_WAYPOINT = [1.0, 0.0, 0.0]

    def __init__(self, scale=0.1, inertial_frame_id='world_ned', max_surge_speed=None, clock=None):
        # Stonefish uses 'world' (NED) frame
        self._waypoints = list()
        self._violates_constraint = False
        self._scale = scale
        self._inertial_frame_id = inertial_frame_id
        self._max_surge_speed = max_surge_speed
        self._clock = clock  # Optional clock for ROS2 timestamps

    def __deepcopy__(self, memo):
        """Custom deepcopy to handle non-picklable clock object."""
        from copy import deepcopy
        # Create new instance without clock
        new_obj = WaypointSet(
            scale=self._scale,
            inertial_frame_id=self._inertial_frame_id,
            max_surge_speed=self._max_surge_speed,
            clock=None  # Don't copy clock
        )
        # Deep copy waypoints
        new_obj._waypoints = deepcopy(self._waypoints, memo)
        new_obj._violates_constraint = self._violates_constraint
        return new_obj

    def __str__(self):
        if self.num_waypoints:
            msg = '================================\n'
            msg += 'List of waypoints\n'
            msg += '================================\n'
            for i in range(self.num_waypoints):
                msg += self.get_waypoint(i).__str__()
                msg += '---\n'
            msg += 'Number of waypoints = %d\n' % self.num_waypoints
            msg += 'Number of valid waypoints = %d\n' % self.num_waypoints
            msg += 'Inertial frame ID = %s\n' % self._inertial_frame_id
            return msg
        else:
            return 'Waypoint set is empty'

    @property
    def num_waypoints(self):
        """`int`: Number of waypoints"""
        return len(self._waypoints)

    @property
    def x(self):
        """`list`: List of the X-coordinates of all waypoints"""
        return [wp.x for wp in self._waypoints]

    @property
    def y(self):
        """`list`: List of the Y-coordinates of all waypoints"""
        return [wp.y for wp in self._waypoints]

    @property
    def z(self):
        """`list`: List of the Z-coordinates of all waypoints"""
        return [wp.z for wp in self._waypoints]

    @property
    def is_empty(self):
        """`bool`: True if the list of waypoints is empty"""
        return len(self._waypoints) == 0

    @property
    def inertial_frame_id(self):
        """`str`: Name of inertial reference frame"""
        return self._inertial_frame_id

    @inertial_frame_id.setter
    def inertial_frame_id(self, frame_id):
        # Stonefish uses 'world' (NED) frame
        self._inertial_frame_id = frame_id

    def clear_waypoints(self):
        """Clear the list of waypoints"""
        self._waypoints = list()

    def set_constraint_status(self, index, flag):
        """Set the flag violates_constraint to a waypoint

        > *Input arguments*

        * `index` (*type:* `int`): Index of the waypoints
        * `flag` (*type:* `bool`): True, if waypoint violates a constraint

        > *Returns*

        `True` if successful, and `False` if the waypoint `index` is outsite of the list's range.
        """
        if index < 0 or index >= len(self._waypoints):
            return False
        self._waypoints[index].violates_constraint = flag
        return True

    def get_waypoint(self, index):
        """Return a waypoint

        > *Input arguments*

        * `index` (*type:* `int`): Index of the waypoint

        > *Returns*

        Return a waypoint as `stonefish_trajectory_manager.Waypoint` or `None` if `index` is outside of range.
        """
        if index < 0 or index >= len(self._waypoints):
            return None
        return self._waypoints[index]

    def add_waypoint(self, waypoint, add_to_beginning=False):
        """Add a waypoint to the set

        > *Input arguments*

        * `waypoint` (*type:* `stonefish_trajectory_manager.Waypoint`): Waypoint object
        * `add_to_beginning` (*type:* `bool`, *default:* `False`): If `True`, add the waypoint to the beginning of the list.

        > *Returns*

        `True` if waypoint was added to the set. `False` if a repeated waypoint is already found in the set.
        """
        if len(self._waypoints):
            if self._waypoints[-1] != waypoint:
                if not add_to_beginning:
                    self._waypoints.append(waypoint)
                else:
                    self._waypoints = [waypoint] + self._waypoints
            else:
                print('Cannot add repeated waypoint')
                return False
        else:
            if not add_to_beginning:
                self._waypoints.append(waypoint)
            else:
                self._waypoints = [waypoint] + self._waypoints
        return True

    def add_waypoint_from_msg(self, msg):
        """Add waypoint from ROS `stonefish_control_msgs/Waypoint` message

        > *Input arguments*

        * `msg` (*type:* `stonefish_control_msgs/Waypoint`): Waypoint message

        > *Returns*

        `True`, if waypoint could be added to set, `False`, otherwise.
        """
        waypoint = Waypoint()
        waypoint.from_message(msg)
        return self.add_waypoint(waypoint)

    def get_start_waypoint(self):
        """Return the starting waypoint

        > *Returns*

        A `stonefish_trajectory_manager.Waypoint` object or None, if the list of waypoints is empty.
        """
        if len(self._waypoints):
            return self._waypoints[0]
        else:
            return None

    def get_last_waypoint(self):
        """Return the final waypoint

        > *Returns*

        A `stonefish_trajectory_manager.Waypoint` object or None, if the list of waypoints is empty.
        """
        if len(self._waypoints):
            return self._waypoints[-1]
        return None

    def remove_waypoint(self, waypoint):
        """Remove waypoint from set.

        > *Input arguments*

        * `waypoint` (*type:* `stonefish_trajectory_manager.Waypoint`): Waypoint object
        """
        new_waypoints = list()
        for point in self._waypoints:
            if point == waypoint:
                continue
            new_waypoints.append(point)
        self._waypoints = new_waypoints

    def read_from_file(self, filename):
        """Read waypoint set from a YAML file.

        > *Input arguments*

        * `filename` (*type:* `str`): Filename of the waypoint set

        > *Returns*

        `True` if waypoint set file could be parsed, `False`, otherwise.
        """
        if not os.path.isfile(filename):
            print('Invalid waypoint filename, filename={}'.format(filename))
            return False
        try:
            self.clear_waypoints()
            with open(filename, 'r') as wp_file:
                wps = yaml.safe_load(wp_file)
                if isinstance(wps, list):
                    # Legacy format: list of waypoints
                    for wp_data in wps:
                        # Support both 'point' and 'position'
                        if 'point' in wp_data:
                            pos = wp_data['point']
                        elif 'position' in wp_data:
                            pos = wp_data['position']
                        else:
                            raise ValueError('Waypoint must have "point" or "position" key')

                        # Heading handling
                        if 'heading_deg' in wp_data:
                            heading = np.deg2rad(wp_data['heading_deg'])
                        else:
                            heading = wp_data.get('heading', 0.0)

                        wp = Waypoint(
                            x=pos[0],
                            y=pos[1],
                            z=pos[2],
                            max_forward_speed=wp_data.get('max_forward_speed', 1.0),
                            heading_offset=heading,
                            use_fixed_heading=wp_data.get('use_fixed_heading', False),
                            inertial_frame_id='world_ned',
                            roll=0.0,
                            pitch=0.0,
                            yaw=heading)
                        self.add_waypoint(wp)
                    self._inertial_frame_id = 'world_ned'
                    # Auto-calculate headings
                    self._auto_calculate_headings()
                else:
                    assert 'inertial_frame_id' in wps, 'Waypoint input has no inertial_frame_id key'
                    assert 'waypoints' in wps
                    # Stonefish uses 'world' (NED) frame
                    self._inertial_frame_id = wps['inertial_frame_id']
                    for wp_data in wps['waypoints']:
                        # Support both old format (point) and new format (position)
                        if 'point' in wp_data:
                            # Old format
                            pos = wp_data['point']
                        elif 'position' in wp_data:
                            # New format (preferred)
                            pos = wp_data['position']
                        else:
                            raise ValueError('Waypoint must have "point" or "position" key')

                        # Heading handling (UUV Simulator style)
                        use_fixed_heading = wp_data.get('use_fixed_heading', False)

                        # Support heading in degrees (convenience)
                        if 'heading_deg' in wp_data:
                            heading = np.deg2rad(wp_data['heading_deg'])
                            use_fixed_heading = True
                        elif 'heading' in wp_data:
                            heading = wp_data['heading']
                            use_fixed_heading = True
                        else:
                            heading = 0.0
                            # Will be auto-calculated if use_fixed_heading=False

                        # 4DOF control: Roll and Pitch always 0
                        roll = 0.0
                        pitch = 0.0
                        yaw = heading

                        wp = Waypoint(
                            x=pos[0],
                            y=pos[1],
                            z=pos[2],
                            max_forward_speed=wp_data.get('max_forward_speed', 1.0),
                            heading_offset=heading,
                            use_fixed_heading=use_fixed_heading,
                            inertial_frame_id=wps['inertial_frame_id'],
                            roll=roll,
                            pitch=pitch,
                            yaw=yaw)
                        self.add_waypoint(wp)

                    # Auto-calculate heading for waypoints with use_fixed_heading=False
                    self._auto_calculate_headings()
        except Exception as e:
            print('Error while loading the file, message={}'.format(e))
            return False
        return True

    def export_to_file(self, path, filename):
        """Export waypoint set to YAML file.

        > *Input arguments*

        * `path` (*type:* `str`): Path to the folder containing the file
        * `filename` (*type:* `str`): Name of the YAML file.

        > *Returns*

        `True` is waypoints could be exported to file. `False`, otherwise.
        """
        try:
            output = dict(inertial_frame_id=self._inertial_frame_id,
                          waypoints=list())
            for wp in self._waypoints:
                wp_elem = dict(point=[float(wp.x), float(wp.y), float(wp.z)],
                               max_forward_speed=float(wp._max_forward_speed),
                               heading=float(wp._heading_offset if wp._heading_offset is not None else 0.0),
                               use_fixed_heading=bool(wp._use_fixed_heading))
                output['waypoints'].append(wp_elem)
            with open(os.path.join(path, filename), 'w') as wp_file:
                yaml.dump(output, wp_file, default_flow_style=False)
            return True
        except Exception as e:
            print('Error occured while exporting waypoint file, message={}'.format(e))
            return False

    def _get_time_msg(self):
        """Get current time as ROS2 Time message.

        Returns TimeMsg (builtin_interfaces/Time) with current time.
        If clock is not set, returns time with sec=0, nanosec=0.
        """
        if self._clock is not None:
            now = self._clock.now()
            time_msg = TimeMsg()
            time_msg.sec = now.seconds_nanoseconds()[0]
            time_msg.nanosec = now.seconds_nanoseconds()[1]
            return time_msg
        else:
            # Return zero time if no clock available
            return TimeMsg(sec=0, nanosec=0)

    def to_message(self):
        """Convert waypoints set to message `stonefish_control_msgs/WaypointSet`

        > *Returns*

        `stonefish_control_msgs/WaypointSet` message object
        """
        msg = WaypointSetMessage()
        msg.header.stamp = self._get_time_msg()
        msg.header.frame_id = self._inertial_frame_id
        msg.waypoints = list()
        for wp in self._waypoints:
            wp_msg = wp.to_message()
            wp_msg.header.frame_id = self._inertial_frame_id
            msg.waypoints.append(wp_msg)
        return msg

    def from_message(self, msg):
        """Convert `stonefish_control_msgs/WaypointSet` message into `stonefish_trajectory_manager.WaypointSet`

        > *Input arguments*

        * `msg` (*type:* `stonefish_control_msgs/WaypointSet` object): Waypoint set message
        """
        self.clear_waypoints()
        self.inertial_frame_id = msg.header.frame_id
        for pnt in msg.waypoints:
            self.add_waypoint_from_msg(pnt)

    @staticmethod
    def from_message_static(msg, clock=None):
        """Static factory method to create WaypointSet from message

        > *Input arguments*

        * `msg` (*type:* `stonefish_control_msgs/WaypointSet`): Waypoint set message
        * `clock` (*type:* `rclpy.clock.Clock`, *default:* `None`): ROS2 clock

        > *Returns*

        `WaypointSet` object
        """
        wp_set = WaypointSet(clock=clock)
        wp_set.from_message(msg)
        return wp_set

    def dist_to_waypoint(self, pos, index=0):
        """Compute the distance of a waypoint in the set to point

        > *Input arguments*

        * `pos` (*type:* list of `float`): 3D point as a list of coordinates
        * `index` (*type:* `int`, *default:* `0`): Waypoint index in set

        > *Returns*

        Distance between `pos` and the waypoint in `index`. `None` if waypoint set is empty.
        """
        wp = self.get_waypoint(index)
        if wp is not None:
            return wp.dist(pos)
        return None

    def set_radius_of_acceptance(self, index, radius):
        """Set the radius of acceptance around each waypoint
        inside which a vehicle is considered to have reached
        a waypoint.

        > *Input arguments*

        * `index` (*type:* `int`): Index of the waypoint
        * `radius` (*type:* `float`): Radius of the sphere representing the volume of acceptance
        """
        if index >= 0 and index < len(self._waypoints):
            self._waypoints[index].radius_of_acceptance = radius

    def get_radius_of_acceptance(self, index):
        """Return the radius of acceptance for a waypoint

        > *Input arguments*

        * `index` (*type:* `int`): Index of the waypoint

        > *Returns*

        Radius of acceptance for the waypoint in position
        given by `index` as a `float`. `None` if waypoint
        set is empty.
        """
        if index >= 0 and index < len(self._waypoints):
            return self._waypoints[index].radius_of_acceptance
        else:
            return None

    def to_path_marker(self, clear=False):
        """Return a `nav_msgs/Path` message with all waypoints in the set

        > *Input arguments*

        * `clear` (*type:* `bool`, *default:* `False`): Return an empty `nav_msgs/Path` message.

        > *Returns*

        `nav_msgs/Path` message
        """
        path = Path()
        path.header.stamp = self._get_time_msg()
        path.header.frame_id = self._inertial_frame_id
        if self.num_waypoints > 1 and not clear:
            for i in range(self.num_waypoints):
                wp = self.get_waypoint(i)
                pose = PoseStamped()
                # Create time message for each pose
                time_msg = TimeMsg()
                time_msg.sec = i
                time_msg.nanosec = 0
                pose.header.stamp = time_msg
                pose.header.frame_id = self._inertial_frame_id
                pose.pose.position.x = wp.x
                pose.pose.position.y = wp.y
                pose.pose.position.z = wp.z
                path.poses.append(pose)
        return path

    def to_marker_list(self, clear=False):
        """Return waypoint set as a markers list message of type `visualization_msgs/MarkerArray`

        > *Input arguments*

        * `clear` (*type:* `bool`, *default:* `False`): Return an empty marker array message

        > *Returns*

        `visualization_msgs/MarkerArray` message
        """
        list_waypoints = MarkerArray()
        t = self._get_time_msg()
        if self.num_waypoints == 0 or clear:
            marker = Marker()
            marker.header.stamp = t
            marker.header.frame_id = self._inertial_frame_id
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = 3  # DELETE
            list_waypoints.markers.append(marker)
        else:
            for i in range(self.num_waypoints):
                wp = self.get_waypoint(i)
                marker = Marker()
                marker.header.stamp = t
                marker.header.frame_id = self._inertial_frame_id
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = wp.x
                marker.pose.position.y = wp.y
                marker.pose.position.z = wp.z
                marker.scale.x = self._scale
                marker.scale.y = self._scale
                marker.scale.z = self._scale
                marker.color.a = 1.0
                if wp == self.get_last_waypoint():
                    color = wp.get_final_color()
                else:
                    color = wp.get_color()
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                list_waypoints.markers.append(marker)
        return list_waypoints

    def generate_circle(self, radius, center, num_points, max_forward_speed,
        theta_offset=0.0, heading_offset=0.0, append=False):
        """Generate a set of waypoints describing a circle

        > *Input arguments*

        * `radius` (*type:* `float`): Radius of the circle in meters
        * `center` (*type:* `stonefish_trajectory_manager.Waypoint`): Center waypoint
        * `num_points` (*type:* `int`): Number of waypoints to be generated
        * `max_forward_speed` (*type:* `float`): Max. forward speed set to each waypoint in m/s
        * `theta_offset` (*type:* `float`, *default:* `0`): Angle offset to start generating the waypoints in radians
        * `heading_offset` (*type:* `float`, *default:* `0`): Heading offset set to the reference heading of the vehicle in radians
        * `append` (*type:* `bool`, *default:* `False`): If `True`, append the generated waypoints to the existent waypoints in the set

        > *Returns*

        `True` if the circle was successfully generated, `False`, otherwise
        """
        if radius <= 0:
            print('Invalid radius, value={}'.format(radius))
            return False

        if num_points <= 0:
            print('Invalid number of samples, value={}'.format(num_points))
            return False

        if max_forward_speed <= 0:
            print('Invalid absolute maximum velocity, value={}'.format(max_forward_speed))
            return False

        if not append:
            # Clear current list
            self.clear_waypoints()

        step_theta = 2 * np.pi / num_points
        for i in range(num_points):
            angle = i * step_theta + theta_offset
            x = np.cos(angle) * radius + center.x
            y = np.sin(angle) * radius + center.y
            z = center.z
            wp = Waypoint(x, y, z, max_forward_speed,
                          heading_offset)
            self.add_waypoint(wp)
        return True

    def generate_helix(self, radius, center, num_points, max_forward_speed, delta_z,
                       num_turns, theta_offset=0.0, heading_offset=0.0,
                       append=False):
        """Generate a set of waypoints describing a helix

        > *Input arguments*

        * `radius` (*type:* `float`): Radius of the circle in meters
        * `center` (*type:* `stonefish_trajectory_manager.Waypoint`): Center waypoint
        * `num_points` (*type:* `int`): Number of waypoints to be generated
        * `max_forward_speed` (*type:* `float`): Max. forward speed set to each waypoint in m/s
        * `delta_z` (*type:* `float`): Step in the Z direction for each lap of the helix in meters
        * `num_turns` (*type:* `int`): Number of turns in the helix
        * `theta_offset` (*type:* `float`, *default:* `0`): Angle offset to start generating the waypoints in radians
        * `heading_offset` (*type:* `float`, *default:* `0`): Heading offset set to the reference heading of the vehicle in radians
        * `append` (*type:* `bool`, *default:* `False`): If `True`, append the generated waypoints to the existent waypoints in the set

        > *Returns*

        `True` if the circle was successfully generated, `False`, otherwise
        """
        if radius <= 0:
            print('Invalid radius, value={}'.format(radius))
            return False

        if num_points <= 0:
            print('Invalid number of samples, value={}'.format(num_points))
            return False

        if num_turns <= 0:
            print('Invalid number of turns, value={}'.format(num_turns))
            return False

        if max_forward_speed <= 0:
            print('Invalid absolute maximum velocity, value={}'.format(max_forward_speed))
            return False

        if not append:
            # Clear current list
            self.clear_waypoints()

        total_angle = 2 * np.pi * num_turns
        step_angle = total_angle / num_points
        step_z = float(delta_z) / num_points
        for i in range(num_points):
            angle = theta_offset + i * step_angle
            x = radius * np.cos(angle) + center.x
            y = radius * np.sin(angle) + center.y
            z = step_z * i + center.z

            wp = Waypoint(x, y, z, max_forward_speed,
                          heading_offset)
            self.add_waypoint(wp)
        return True

    def _auto_calculate_headings(self):
        """Auto-calculate heading for waypoints with use_fixed_heading=False.

        For trajectory control, heading should point towards the next waypoint
        when not explicitly specified. This makes YAML files much simpler.

        Note:
        - Auto-heading means the robot follows the path tangent direction
        - heading_offset should be 0 (no additional rotation from tangent)
        - The interpolator's _compute_rot_quat() will calculate tangent from path
        - Only calculates yaw (heading). Roll and pitch remain 0 for 4DOF control.
        """
        for i in range(len(self._waypoints) - 1):
            wp_current = self._waypoints[i]
            wp_next = self._waypoints[i + 1]

            # Only auto-calculate if use_fixed_heading is False
            if not wp_current.using_heading_offset:
                # Auto-heading: Let interpolator compute tangent direction
                # heading_offset = 0 means "follow the path tangent"
                wp_current._heading_offset = 0.0

                # Calculate and store expected heading for reference (debugging)
                heading = wp_current.calculate_heading(wp_next)
                wp_current._heading = heading
                wp_current._yaw = heading

        # Last waypoint: use same heading as previous
        if len(self._waypoints) >= 2:
            if not self._waypoints[-1].using_heading_offset:
                # Keep same heading_offset as second-to-last waypoint
                self._waypoints[-1]._heading_offset = self._waypoints[-2]._heading_offset
                self._waypoints[-1]._heading = self._waypoints[-2]._heading
                self._waypoints[-1]._yaw = self._waypoints[-2]._yaw

