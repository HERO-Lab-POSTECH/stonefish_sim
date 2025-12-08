#!/usr/bin/env python3
# Copyright 2025
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

"""
4DOF Path Following Node (ILOS Guidance)

ROS2 node for ILOS (Integral Line-of-Sight) guidance for 4DOF path following.
Subscribes to path from path_generator_node.

Architecture:
    Path Generator → /path → Path Following 4DOF → cmd_pose → 4DOF PID Controller

References:
- Lekkas & Fossen (2014). "Integral LOS Path Following for Curved Paths"
- Fossen (2011). "Handbook of Marine Craft Hydrodynamics and Motion Control"
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, TransformStamped, PointStamped
from stonefish_control_msgs.msg import TrajectoryPoint
from stonefish_control_msgs.srv import ResetTrajectory
from std_msgs.msg import String
import numpy as np
from transforms3d.euler import euler2quat, quat2euler
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

from ..path_following import ILOSGuidance


class PathFollowing4DOFNode(Node):
    """ROS2 node for 4DOF path following using ILOS guidance."""

    def __init__(self):
        super().__init__('path_following_4dof_node')

        # Declare parameters
        self.declare_parameter('vehicle_name', 'bluerov2')
        self.declare_parameter('update_rate', 50.0)  # Hz (match PID rate)

        # ILOS parameters
        self.declare_parameter('lookahead_distance', 3.0)  # meters
        self.declare_parameter('integral_gain', 0.05)      # ILOS integral gain
        self.declare_parameter('integral_limit', 5.0)      # Anti-windup limit
        self.declare_parameter('lateral_gain', 0.5)        # Lateral correction gain
        self.declare_parameter('depth_gain', 0.8)          # Depth error correction gain

        # PD control parameters
        self.declare_parameter('lateral_kd', 0.3)     # Lateral velocity derivative gain
        self.declare_parameter('depth_kd', 0.5)       # Depth velocity derivative gain

        # Velocity saturation limits
        self.declare_parameter('max_lateral_velocity', 0.5)  # m/s (BlueROV2 sway limit)
        self.declare_parameter('max_heave_velocity', 0.4)    # m/s (BlueROV2 heave limit)

        # Velocity profiling parameters
        self.declare_parameter('cruise_speed', 0.5)     # m/s (straight line)
        self.declare_parameter('min_speed', 0.2)        # m/s (tight curves)
        self.declare_parameter('curvature_gain', 2.0)   # Speed reduction sensitivity

        # Initial heading alignment
        self.declare_parameter('heading_align_threshold', 10.0)  # degrees

        # Get parameters
        self.vehicle_name = self.get_parameter('vehicle_name').value
        update_rate = self.get_parameter('update_rate').value

        lookahead_distance = self.get_parameter('lookahead_distance').value
        integral_gain = self.get_parameter('integral_gain').value
        integral_limit = self.get_parameter('integral_limit').value
        lateral_gain = self.get_parameter('lateral_gain').value
        depth_gain = self.get_parameter('depth_gain').value
        lateral_kd = self.get_parameter('lateral_kd').value
        depth_kd = self.get_parameter('depth_kd').value
        max_lateral_velocity = self.get_parameter('max_lateral_velocity').value
        max_heave_velocity = self.get_parameter('max_heave_velocity').value
        cruise_speed = self.get_parameter('cruise_speed').value
        min_speed = self.get_parameter('min_speed').value
        curvature_gain = self.get_parameter('curvature_gain').value
        heading_align_threshold_deg = self.get_parameter('heading_align_threshold').value
        heading_align_threshold = np.deg2rad(heading_align_threshold_deg)

        # State
        self._path_received = False
        self._odom_received = False
        self._last_update_time = None
        self._path_complete_logged = False
        self._recording_active = False
        self._prev_mode = None  # For mode transition logging

        # Actual trajectory recording
        self._actual_trajectory = Path()
        self._actual_trajectory.header.frame_id = 'world_ned'

        # TF Buffer and Listener for getting base_link position
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Initialize ILOS guidance
        self.guidance = ILOSGuidance(
            lookahead_distance=lookahead_distance,
            integral_gain=integral_gain,
            integral_limit=integral_limit,
            cruise_speed=cruise_speed,
            min_speed=min_speed,
            curvature_gain=curvature_gain,
            lateral_gain=lateral_gain,
            depth_gain=depth_gain,
            heading_align_threshold=heading_align_threshold,
            lateral_kd=lateral_kd,
            depth_kd=depth_kd,
            max_lateral_velocity=max_lateral_velocity,
            max_heave_velocity=max_heave_velocity
        )

        # Subscriber: path from path_generator_node
        self.path_sub = self.create_subscription(
            Path,
            '/path_generator_node/path',
            self._path_callback,
            10
        )

        # Subscriber: odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.vehicle_name}/odometry',
            self._odometry_callback,
            10
        )

        # Publisher: cmd_pose
        self.guidance_pub = self.create_publisher(
            TrajectoryPoint,
            f'/{self.vehicle_name}/cmd_pose',
            10
        )

        # Publisher: control_mode
        self.mode_pub = self.create_publisher(
            String,
            f'/{self.vehicle_name}/control_mode',
            10
        )

        # Publisher: actual_trajectory (for visualization/analysis)
        self.actual_trajectory_pub = self.create_publisher(
            Path,
            f'/{self.vehicle_name}/actual_trajectory',
            10
        )

        # Publisher: lookahead_point (for visualization)
        self.lookahead_point_pub = self.create_publisher(
            PointStamped,
            f'/{self.vehicle_name}/lookahead_point',
            10
        )

        # Control mode state (hybrid: velocity + position)
        self.current_control_mode = 'hybrid'

        # Service: Reset trajectory
        self._reset_service = self.create_service(
            ResetTrajectory,
            f'/{self.vehicle_name}/reset_trajectory',
            self._reset_trajectory_callback
        )

        # Timer for guidance update
        timer_period = 1.0 / update_rate
        self.timer = self.create_timer(timer_period, self._guidance_update_callback)

        # Timer for trajectory publishing (1 Hz)
        self._trajectory_pub_timer = self.create_timer(1.0, self._publish_trajectory)

        self.get_logger().info(
            f'[ILOS 4DOF] Initialized - lookahead: {lookahead_distance:.1f}m | '
            f'integral_gain: {integral_gain:.3f} | lateral: Kp={lateral_gain:.1f} Kd={lateral_kd:.1f} | '
            f'depth: Kp={depth_gain:.1f} Kd={depth_kd:.1f} | '
            f'velocity_limits: lateral={max_lateral_velocity:.1f} heave={max_heave_velocity:.1f} m/s | '
            f'cruise_speed: {cruise_speed:.1f}m/s | heading_align: {heading_align_threshold_deg:.1f}° | '
            f'rate: {update_rate:.0f}Hz'
        )

        # Publish initial control mode
        mode_msg = String()
        mode_msg.data = self.current_control_mode
        self.mode_pub.publish(mode_msg)

    def _path_callback(self, msg):
        """Receive path from path_generator_node.

        Args:
            msg: nav_msgs/Path message
        """
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path')
            return

        # Extract path poses (dense array from path generator)
        path_poses = []
        for pose_stamped in msg.poses:
            pos = pose_stamped.pose.position
            path_poses.append([pos.x, pos.y, pos.z])

        # Set path in guidance
        self.guidance.set_path(path_poses)

        # Log path reception with update indicator
        if self._path_received:
            self.get_logger().info(
                f'[ILOS 4DOF] Path updated - {len(path_poses)} points '
                f'({self.guidance._total_path_length:.1f}m total)'
            )
        else:
            self.get_logger().info(
                f'[ILOS 4DOF] Path received - {len(path_poses)} points '
                f'({self.guidance._total_path_length:.1f}m total)'
            )
            self._path_received = True

        # Start trajectory recording
        self._recording_active = True
        self._actual_trajectory.poses.clear()
        self.get_logger().info('[ILOS 4DOF] Trajectory recording started')

    def _odometry_callback(self, msg):
        """Update vehicle state from odometry.

        Args:
            msg: nav_msgs/Odometry message
        """
        if self.guidance is None:
            return

        # Extract position directly from odometry (world_ned frame)
        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        # Extract orientation quaternion
        q = msg.pose.pose.orientation
        orientation_quat = np.array([q.w, q.x, q.y, q.z])

        # Extract linear velocity (NED world frame from odometry)
        velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])

        # Update guidance
        self.guidance.update_vehicle_state(position, orientation_quat, velocity)

        # Record trajectory if active
        if self._recording_active:
            from geometry_msgs.msg import PoseStamped
            pos = msg.pose.pose.position
            if not (np.isnan(pos.x) or np.isnan(pos.y) or np.isnan(pos.z)):
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = msg.header.stamp
                pose_stamped.header.frame_id = 'world_ned'
                pose_stamped.pose = msg.pose.pose
                self._actual_trajectory.poses.append(pose_stamped)

        if not self._odom_received:
            self._odom_received = True
            self.get_logger().info('[ILOS 4DOF] Odometry received - guidance active')

    def _reset_trajectory_callback(self, request, response):
        """Service callback to reset trajectory.

        Args:
            request: ResetTrajectory request
            response: ResetTrajectory response

        Returns:
            ResetTrajectory response
        """
        try:
            # Reset guidance
            if self.guidance is not None:
                self.guidance.reset()

            # Reset state
            self._path_received = False  # Allow new path to be received
            self._odom_received = False
            self._last_update_time = None
            self._path_complete_logged = False

            # Reset trajectory recording
            self._actual_trajectory.poses.clear()
            self._recording_active = False  # Will be restarted when new path arrives
            self.get_logger().info('[ILOS 4DOF] Trajectory recording reset')

            response.success = True
            response.message = 'Trajectory reset - ready for new path'
            self.get_logger().info('[ILOS 4DOF] Full reset complete - ready for new path')

        except Exception as e:
            response.success = False
            response.message = f'Failed to reset: {str(e)}'
            self.get_logger().error(f'[ILOS 4DOF] Reset failed: {e}')

        return response

    def _guidance_update_callback(self):
        """Update guidance law and publish command."""
        if not self._path_received or not self._odom_received:
            return

        if self.guidance is None:
            return

        # Compute time step
        current_time = self.get_clock().now()
        if self._last_update_time is None:
            dt = 1.0 / 50.0  # Initial guess
        else:
            dt = (current_time - self._last_update_time).nanoseconds / 1e9

        self._last_update_time = current_time

        # Update guidance law
        success = self.guidance.update(dt)
        if not success:
            self.get_logger().warn('Guidance update failed', throttle_duration_sec=5.0)
            return

        # Get guidance command (hybrid architecture: position + heading + velocities)
        desired_position, desired_heading, desired_velocities = self.guidance.compute_guidance(dt)

        # Check mode transition
        current_mode = self.guidance.get_mode()
        if self._prev_mode is None:
            self._prev_mode = current_mode

        if current_mode != self._prev_mode:
            mode_name = current_mode.value
            self.get_logger().info(f'[PATH FOLLOWING] Mode transition: {self._prev_mode.value} → {mode_name}')
            self._prev_mode = current_mode

        # Publish lookahead point for visualization
        lookahead_msg = PointStamped()
        lookahead_msg.header.stamp = current_time.to_msg()
        lookahead_msg.header.frame_id = 'world_ned'
        lookahead_msg.point.x = float(desired_position[0])
        lookahead_msg.point.y = float(desired_position[1])
        lookahead_msg.point.z = float(desired_position[2])
        self.lookahead_point_pub.publish(lookahead_msg)

        # Create TrajectoryPoint message
        msg = TrajectoryPoint()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'world_ned'

        # Check path completion
        path_finished = self.guidance.is_path_finished()

        # Switch control mode: hybrid during path following, position when finished
        new_mode = 'position' if path_finished else 'hybrid'
        if new_mode != self.current_control_mode:
            self.current_control_mode = new_mode
            mode_msg = String()
            mode_msg.data = new_mode
            self.mode_pub.publish(mode_msg)
            self.get_logger().info(f'[PATH FOLLOWING] Control mode: {new_mode}')

        if path_finished:
            # Path complete: station-keeping at end point
            # Position: End point (for station-keeping)
            end_point = self.guidance._path_poses[-1]
            msg.pose.position.x = float(end_point[0])
            msg.pose.position.y = float(end_point[1])
            msg.pose.position.z = float(end_point[2])

            # Heading: final heading
            quat = euler2quat(0.0, 0.0, desired_heading, 'sxyz')
            msg.pose.orientation.w = quat[0]
            msg.pose.orientation.x = quat[1]
            msg.pose.orientation.y = quat[2]
            msg.pose.orientation.z = quat[3]

            # Zero velocity (stop)
            msg.velocity = Twist()

            self.guidance_pub.publish(msg)

            # Log completion once
            if not self._path_complete_logged:
                max_cte = self.guidance.get_max_cross_track_error()
                goal_pos = self.guidance._path_poses[-1]
                final_dist = np.linalg.norm(self.guidance._vehicle_pos - goal_pos)

                self.get_logger().info(
                    f'[ILOS 4DOF] Path complete - '
                    f'Final dist: {final_dist:.3f}m | Max CTE: {max_cte:.3f}m'
                )
                self._path_complete_logged = True

                # Stop trajectory recording
                if self._recording_active:
                    self._recording_active = False
                    self.get_logger().info(
                        f'[ILOS 4DOF] Recording stopped ({len(self._actual_trajectory.poses)} points)'
                    )

            return  # Don't update further

        # =========== HYBRID ARCHITECTURE: Velocity + Position Control ===========
        # Position: Lookahead point (for outer loop stability)
        msg.pose.position.x = float(desired_position[0])
        msg.pose.position.y = float(desired_position[1])
        msg.pose.position.z = float(desired_position[2])

        # Desired heading (roll=0, pitch=0, yaw=desired_heading)
        # 4DOF: Roll and pitch are passive (not controlled)
        quat = euler2quat(0.0, 0.0, desired_heading, 'sxyz')  # [w, x, y, z]
        msg.pose.orientation.w = quat[0]
        msg.pose.orientation.x = quat[1]
        msg.pose.orientation.y = quat[2]
        msg.pose.orientation.z = quat[3]

        # Desired velocities (FRD body frame)
        msg.velocity = Twist()
        msg.velocity.linear.x = float(desired_velocities[0])   # u (surge)
        msg.velocity.linear.y = float(desired_velocities[1])   # v (sway, lateral correction)
        msg.velocity.linear.z = float(desired_velocities[2])   # w (heave)
        msg.velocity.angular.x = 0.0  # p (roll rate, passive)
        msg.velocity.angular.y = 0.0  # q (pitch rate, passive)
        msg.velocity.angular.z = float(desired_velocities[3])  # r (yaw rate)

        # Publish cmd_pose
        self.guidance_pub.publish(msg)

        # Log progress (throttled) with mode indication
        cmd = self.guidance.get_guidance_command()
        goal_pos = self.guidance._path_poses[-1]
        distance_to_goal = np.linalg.norm(self.guidance._vehicle_pos - goal_pos)

        # Calculate actual lookahead distance
        lookahead_dist = np.linalg.norm(desired_position - self.guidance._vehicle_pos)

        # Mode-based logging
        mode_str = current_mode.value.upper()
        self.get_logger().info(
            f"[{mode_str}] Progress: {cmd['path_progress']*100:.0f}% | "
            f"Lookahead: {lookahead_dist:.2f}m | CTE: {cmd['cross_track_error']:.2f}m | "
            f"Speed: {cmd['desired_speed']:.2f}m/s",
            throttle_duration_sec=2.0
        )

    def _publish_trajectory(self):
        """Publish actual trajectory periodically."""
        if not self._recording_active or len(self._actual_trajectory.poses) == 0:
            return

        # Update timestamp
        self._actual_trajectory.header.stamp = self.get_clock().now().to_msg()

        # Publish trajectory
        self.actual_trajectory_pub.publish(self._actual_trajectory)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = None

    try:
        node = PathFollowing4DOFNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Graceful shutdown
    except Exception as e:
        import traceback
        print(f'Error: {e}')
        traceback.print_exc()
    finally:
        # Clear actual trajectory on shutdown
        if node is not None:
            # Publish empty trajectory to clear RViz visualization
            empty_traj = Path()
            empty_traj.header.stamp = node.get_clock().now().to_msg()
            empty_traj.header.frame_id = 'world_ned'
            node.actual_trajectory_pub.publish(empty_traj)

            # Stop recording
            node._recording_active = False
            node.get_logger().info('[ILOS 4DOF] Shutdown - trajectory cleared')

            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
