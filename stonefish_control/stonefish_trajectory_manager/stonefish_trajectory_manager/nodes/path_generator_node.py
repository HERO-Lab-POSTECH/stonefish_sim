#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Path Generator Node

Generates paths from waypoints and publishes them for path following nodes.

This node:
1. Loads waypoints from a YAML file
2. Generates path using specified interpolation method (linear, LIPB, cubic_spline, etc.)
3. Publishes path as nav_msgs/Path topic (for path following nodes)
4. Publishes visualization markers to RViz
5. (Future) Supports dynamic waypoint updates via service/topic

Supported Interpolation Methods:
- linear: Piecewise linear segments
- lipb: Log-Interpolated Polynomial Bezier (smooth corners)
- cubic: Cubic spline interpolation
- cs: Cubic Hermite spline
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

from ..common import WaypointSet, WPTrajectoryGenerator
from .utils import load_waypoints, create_trajectory_generator
from stonefish_control_msgs.srv import ResetTrajectory


class PathGeneratorNode(Node):
    """Path Generator Node for generating and publishing paths from waypoints."""

    def __init__(self):
        super().__init__('path_generator_node')

        # Declare parameters
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('interpolation_method', 'cubic')
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('sample_step', 0.01)  # Parametric step for sampling
        self.declare_parameter('vehicle_name', 'bluerov2')  # For odometry
        self.declare_parameter('initial_waypoint_distance_threshold', 0.5)  # meters

        # Get parameters
        self.waypoint_file = self.get_parameter('waypoint_file').value
        self.interp_method = self.get_parameter('interpolation_method').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.sample_step = self.get_parameter('sample_step').value
        self.vehicle_name = self.get_parameter('vehicle_name').value
        self.distance_threshold = self.get_parameter('initial_waypoint_distance_threshold').value

        # Validate parameters
        if not self.waypoint_file:
            self.get_logger().error('No waypoint file specified! Use -p waypoint_file:=<path>')
            raise ValueError('waypoint_file parameter is required')

        # State
        self._robot_initial_position = None
        self._path_generated = False
        self._trajectory_points = None

        # TF Buffer and Listener for getting base_link position
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Publishers
        # Main topic for path following
        self._path_pub = self.create_publisher(
            Path, '~/path', 10
        )
        # Visualization topic for RViz (waypoints only, path is visualized via /path topic)
        self._waypoint_markers_pub = self.create_publisher(
            MarkerArray, '~/waypoint_markers', 10
        )

        # Subscriber: odometry for robot initial position
        self._odom_sub = self.create_subscription(
            Odometry,
            f'/{self.vehicle_name}/odometry',
            self._odometry_callback,
            10
        )

        # Service: Reset path generation
        self._reset_service = self.create_service(
            ResetTrajectory,
            '~/reset_path',
            self._reset_path_callback
        )

        # Load waypoints (don't generate path yet - wait for robot position)
        self._waypoints = WaypointSet(clock=self.get_clock())

        if not self._waypoints.read_from_file(self.waypoint_file):
            self.get_logger().error(f'[Path Generator] Failed to load waypoints from {self.waypoint_file}')
            raise RuntimeError('Failed to load waypoint file')

        self.get_logger().info(f'[Path Generator] Loaded {self._waypoints.num_waypoints} waypoints from {self.waypoint_file}')

        # Create timer to publish path (once generated)
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_callback)

        self.get_logger().info('[Path Generator] Initialized - waiting for robot odometry...')

    def _odometry_callback(self, msg):
        """Receive robot initial position and generate path.

        Args:
            msg: nav_msgs/Odometry message
        """
        if self._path_generated:
            return  # Already generated, ignore further odometry

        # Extract robot position directly from odometry
        robot_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        # Always use first valid odometry (no validation needed)
        # The user should call reset service if they want to regenerate path

        self._robot_initial_position = robot_pos

        # Generate path with robot position
        self._generate_path_with_initial_position()

        # DO NOT unsubscribe - we need odometry for reset functionality
        # The _path_generated flag prevents redundant path generation

    def _reset_path_callback(self, request, response):
        """Service callback to reset path generation.

        This allows path to be regenerated from current robot position.
        Useful for optimization where robot position changes between evaluations.

        Args:
            request: ResetTrajectory request
            response: ResetTrajectory response

        Returns:
            ResetTrajectory response
        """
        try:
            self.get_logger().info('[Path Generator] Reset requested - reloading waypoints from file')

            # Reset flag to allow path regeneration
            self._path_generated = False
            self._robot_initial_position = None
            self._trajectory_points = None

            # IMPORTANT: Reload waypoints from file to clear any previously added WP0
            # Without this, WP0 waypoints would accumulate on each reset!
            self._waypoints = WaypointSet(clock=self.get_clock())
            if not self._waypoints.read_from_file(self.waypoint_file):
                self.get_logger().error(f'[Path Generator] Failed to reload waypoints from {self.waypoint_file}')
                response.success = False
                response.message = 'Failed to reload waypoints from file'
                return response

            self.get_logger().info(f'[Path Generator] Reloaded {self._waypoints.num_waypoints} waypoints from file')

            response.success = True
            response.message = 'Path generator reset. Waypoints reloaded. Path will be regenerated on next odometry update.'

        except Exception as e:
            response.success = False
            response.message = f'Failed to reset path generator: {str(e)}'
            self.get_logger().error(f'[Path Generator] Reset failed: {e}')

        return response

    def _generate_path_with_initial_position(self):
        """Generate path, adding robot position as WP0 if needed."""
        if self._path_generated:
            return

        # Get first waypoint from YAML
        first_wp = self._waypoints.get_waypoint(0)
        first_wp_pos = np.array([first_wp.x, first_wp.y, first_wp.z])

        # Calculate distance
        distance = np.linalg.norm(self._robot_initial_position - first_wp_pos)

        # Add robot position as WP0 if far from first waypoint
        if distance > self.distance_threshold:
            from ..common import Waypoint
            wp0 = Waypoint(
                x=float(self._robot_initial_position[0]),
                y=float(self._robot_initial_position[1]),
                z=float(self._robot_initial_position[2]),
                use_fixed_heading=False,
                inertial_frame_id='world_ned'
            )

            # Insert at beginning
            self._waypoints.add_waypoint(wp0, add_to_beginning=True)

            self.get_logger().info(f'[Path Generator] Added WP0 at robot position (distance to first WP: {distance:.2f}m > {self.distance_threshold:.2f}m)')
        else:
            self.get_logger().info(f'[Path Generator] Robot close to first WP ({distance:.2f}m), no WP0 needed')

        # Generate path
        self._traj_gen = WPTrajectoryGenerator(
            interpolation_method=self.interp_method
        )

        if not self._traj_gen.init_waypoints(self._waypoints):
            self.get_logger().error('[Path Generator] Failed to initialize trajectory generator')
            raise RuntimeError('Failed to initialize trajectory generator')

        if not self._traj_gen.interpolator.init_interpolator():
            self.get_logger().error('[Path Generator] Failed to initialize interpolator')
            raise RuntimeError('Failed to initialize interpolator')

        max_time = self._traj_gen.get_max_time()

        # Get trajectory samples
        self._trajectory_points = self._traj_gen.get_samples(step=self.sample_step)

        self._path_generated = True

        # Calculate actual spatial resolution achieved
        if len(self._trajectory_points) > 1:
            total_dist = 0.0
            for i in range(1, len(self._trajectory_points)):
                total_dist += np.linalg.norm(
                    np.array(self._trajectory_points[i].p) - np.array(self._trajectory_points[i-1].p)
                )
            avg_resolution = total_dist / (len(self._trajectory_points) - 1) if len(self._trajectory_points) > 1 else 0.0

            self.get_logger().info(
                f'[Path Generator] Ready - {self.interp_method} interpolation | '
                f'{len(self._trajectory_points)} points | '
                f'{total_dist:.1f}m total | '
                f'{avg_resolution:.3f}m resolution'
            )
        else:
            self.get_logger().info(
                f'[Path Generator] Ready - {self.interp_method} interpolation | '
                f'{len(self._trajectory_points)} points'
            )

    def publish_callback(self):
        """Publish path and visualization markers."""
        if not self._path_generated or self._trajectory_points is None:
            return  # Path not generated yet, skip publishing

        # Publish waypoint markers for RViz
        waypoint_markers = self._waypoints.to_marker_list()
        self._waypoint_markers_pub.publish(waypoint_markers)

        # Publish as nav_msgs/Path (for path following AND RViz visualization)
        path_msg = Path()
        path_msg.header.frame_id = 'world_ned'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for traj_pnt in self._trajectory_points:
            pose = PoseStamped()
            pose.header.frame_id = 'world_ned'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(traj_pnt.p[0])
            pose.pose.position.y = float(traj_pnt.p[1])
            pose.pose.position.z = float(traj_pnt.p[2])
            # Quaternion (w, x, y, z) internal -> (x, y, z, w) ROS
            pose.pose.orientation.w = float(traj_pnt.q[0])
            pose.pose.orientation.x = float(traj_pnt.q[1])
            pose.pose.orientation.y = float(traj_pnt.q[2])
            pose.pose.orientation.z = float(traj_pnt.q[3])
            path_msg.poses.append(pose)

        self._path_pub.publish(path_msg)


def main(args=None):
    """Main entry point for path generator node."""
    rclpy.init(args=args)
    node = None

    try:
        node = PathGeneratorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Graceful shutdown
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        # Ensure node is destroyed before shutdown
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
