#!/usr/bin/env python3
# Copyright (c) 2025 Stonefish Control Contributors
# Licensed under the Apache License, Version 2.0

"""
ROS2 Node for 4DOF PID Controller

Subscribes:
    - cmd_pose (stonefish_control_msgs/TrajectoryPoint): Desired pose and velocity
    - odometry (nav_msgs/Odometry): Current vehicle state

Publishes:
    - thruster_manager/input_stamped (geometry_msgs/WrenchStamped): Control forces

Parameters:
    Loaded from pid_4dof_params.yaml and dynamics_params.yaml

Reference:
    - UUV Simulator: rov_ua_pid_controller.py
    - Fossen (2011) Section 8.2.1
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
import numpy as np

# ROS2 messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from stonefish_control_msgs.msg import TrajectoryPoint
from scipy.spatial.transform import Rotation

# Import PID controller and dynamics loader
from stonefish_control.controllers.position_controller import PositionController
from stonefish_control.control_interfaces import DynamicsLoader


class PID4DOFNode(Node):
    """
    ROS2 Node for 4DOF Underactuated PID Controller

    Control Architecture:
        cmd_pose → PID4DOF → thruster_manager → thrusters
        odometry ↗

    Frame Convention:
        - cmd_pose: World NED
        - odometry: World NED (position), Body FRD (velocity)
        - output wrench: Body FRD
    """

    def __init__(self):
        super().__init__('pid_4dof_controller')

        # ============================================================
        # Load Vehicle Dynamics (SINGLE SOURCE OF TRUTH)
        # ============================================================
        # DynamicsLoader loads essential parameters from ROS parameters
        # Parameters must be loaded in launch file from dynamics_params.yaml
        try:
            self.dynamics = DynamicsLoader(self)
        except Exception as e:
            self.get_logger().error(f'Failed to load vehicle dynamics: {e}')
            self.get_logger().error('Make sure dynamics_params.yaml is loaded in launch file')
            raise

        # ============================================================
        # Load PID Parameters
        # ============================================================
        self._setup_pid_parameters()
        self._load_pid_parameters()

        # ============================================================
        # Initialize Controller
        # ============================================================
        self.controller = PositionController(
            Kp=self.Kp,
            Kd=self.Kd,
            Ki=self.Ki,
            Kb=self.Kb,
            mass=self.dynamics.mass,
            inertia_zz=self.dynamics.inertia_zz,
            max_force=self.max_force,
            max_torque=self.max_torque,
            integral_limit=self.integral_limit
        )

        # ============================================================
        # State Variables
        # ============================================================
        self.odom_received = False
        self.cmd_received = False

        self.current_pose = np.zeros(6)  # [x, y, z, roll, pitch, yaw]
        self.current_vel = np.zeros(6)   # [u, v, w, p, q, r]
        self.desired_pose = np.zeros(4)  # [x, y, z, yaw]
        self.desired_vel = None          # Optional feedforward velocity

        self.last_control_time = None

        # ============================================================
        # QoS Profile (Best Effort for real-time control)
        # ============================================================
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ============================================================
        # Subscribers
        # ============================================================
        self.sub_odom = self.create_subscription(
            Odometry,
            'odometry',
            self.odometry_callback,
            qos
        )

        self.sub_cmd = self.create_subscription(
            TrajectoryPoint,
            'cmd_pose',
            self.cmd_pose_callback,
            10
        )

        # ============================================================
        # Publishers
        # ============================================================
        self.pub_wrench = self.create_publisher(
            WrenchStamped,
            'thruster_manager/input_stamped',
            10
        )

        # ============================================================
        # Timer (Control Loop)
        # ============================================================
        self.control_dt = 1.0 / self.control_rate
        self.timer = self.create_timer(self.control_dt, self.control_loop)

        # ============================================================
        # Logging
        # ============================================================
        self.get_logger().info('=' * 60)
        self.get_logger().info('4DOF PID Controller Initialized')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Kp: {self.Kp}')
        self.get_logger().info(f'Kd: {self.Kd}')
        self.get_logger().info(f'Ki: {self.Ki}')
        self.get_logger().info(f'Kb: {self.Kb}')
        self.get_logger().info(f'Control rate: {self.control_rate} Hz')
        self.get_logger().info(f'Feedforward: {self.enable_feedforward}')
        self.get_logger().info('=' * 60)

    def _setup_pid_parameters(self):
        """Declare PID-specific ROS2 parameters"""
        # PID Gains
        self.declare_parameter('Kp', [300.0, 300.0, 400.0, 200.0])
        self.declare_parameter('Kd', [150.0, 150.0, 200.0, 100.0])
        self.declare_parameter('Ki', [10.0, 10.0, 20.0, 5.0])
        self.declare_parameter('Kb', [0.8, 0.8, 0.8, 0.8])

        # Saturation
        self.declare_parameter('max_force', 200.0)
        self.declare_parameter('max_torque', 50.0)
        self.declare_parameter('integral_limit', [10.0, 10.0, 5.0, 5.0])

        # Control
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('enable_feedforward', True)
        self.declare_parameter('feedforward_gain', 0.1)

    def _load_pid_parameters(self):
        """Load PID-specific parameters from ROS2"""
        # PID Gains
        self.Kp = np.array(self.get_parameter('Kp').value)
        self.Kd = np.array(self.get_parameter('Kd').value)
        self.Ki = np.array(self.get_parameter('Ki').value)
        self.Kb = np.array(self.get_parameter('Kb').value)

        # Saturation
        self.max_force = self.get_parameter('max_force').value
        self.max_torque = self.get_parameter('max_torque').value
        self.integral_limit = np.array(self.get_parameter('integral_limit').value)

        # Control
        self.control_rate = self.get_parameter('control_rate').value
        self.enable_feedforward = self.get_parameter('enable_feedforward').value
        self.feedforward_gain = self.get_parameter('feedforward_gain').value

    def odometry_callback(self, msg: Odometry):
        """Process odometry message"""
        # Position (World NED)
        self.current_pose[0] = msg.pose.pose.position.x
        self.current_pose[1] = msg.pose.pose.position.y
        self.current_pose[2] = msg.pose.pose.position.z

        # Orientation (World NED)
        quat = msg.pose.pose.orientation
        r = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        self.current_pose[3] = roll
        self.current_pose[4] = pitch
        self.current_pose[5] = yaw

        # Velocity (Body FRD)
        self.current_vel[0] = msg.twist.twist.linear.x  # u (surge)
        self.current_vel[1] = msg.twist.twist.linear.y  # v (sway)
        self.current_vel[2] = msg.twist.twist.linear.z  # w (heave)
        self.current_vel[3] = msg.twist.twist.angular.x # p (roll rate)
        self.current_vel[4] = msg.twist.twist.angular.y # q (pitch rate)
        self.current_vel[5] = msg.twist.twist.angular.z # r (yaw rate)

        self.odom_received = True

    def cmd_pose_callback(self, msg: TrajectoryPoint):
        """Process command pose message"""
        # Desired position (World NED)
        self.desired_pose[0] = msg.pose.position.x
        self.desired_pose[1] = msg.pose.position.y
        self.desired_pose[2] = msg.pose.position.z

        # Desired yaw (World NED)
        quat = msg.pose.orientation
        r = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        _, _, yaw = r.as_euler('xyz', degrees=False)
        self.desired_pose[3] = yaw

        # Feedforward velocity (Body FRD)
        if self.enable_feedforward:
            # cmd_pose velocity is in body frame
            self.desired_vel = np.array([
                msg.velocity.linear.x,   # u
                msg.velocity.linear.y,   # v
                msg.velocity.linear.z,   # w
                msg.velocity.angular.z   # r (yaw rate)
            ]) * self.feedforward_gain
        else:
            self.desired_vel = None

        self.cmd_received = True

    def control_loop(self):
        """Main control loop (called at control_rate Hz)"""
        # Wait for messages
        if not self.odom_received or not self.cmd_received:
            return

        # Compute dt
        current_time = self.get_clock().now()
        if self.last_control_time is None:
            dt = self.control_dt
        else:
            dt = (current_time - self.last_control_time).nanoseconds / 1e9

        self.last_control_time = current_time

        # Compute control
        tau_6dof, debug_info = self.controller.compute_control(
            pose_des=self.desired_pose,
            pose_curr=self.current_pose,
            vel_curr=self.current_vel,
            dt=dt,
            vel_ff=self.desired_vel
        )

        # Publish wrench
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = current_time.to_msg()
        wrench_msg.header.frame_id = 'base_link'

        wrench_msg.wrench.force.x = tau_6dof[0]
        wrench_msg.wrench.force.y = tau_6dof[1]
        wrench_msg.wrench.force.z = tau_6dof[2]
        wrench_msg.wrench.torque.x = tau_6dof[3]
        wrench_msg.wrench.torque.y = tau_6dof[4]
        wrench_msg.wrench.torque.z = tau_6dof[5]

        self.pub_wrench.publish(wrench_msg)

        # Logging (throttled)
        self.log_debug_info(debug_info, tau_6dof)

    def log_debug_info(self, debug_info: dict, tau_6dof: np.ndarray):
        """Log debug information (throttled)"""
        # Log every 2 seconds
        self.get_logger().info(
            f'[PID 4DOF] '
            f'pose_cur=[{self.current_pose[0]:.2f}, {self.current_pose[1]:.2f}, {self.current_pose[2]:.2f}] | '
            f'pose_des=[{self.desired_pose[0]:.2f}, {self.desired_pose[1]:.2f}, {self.desired_pose[2]:.2f}] | '
            f'e=[{debug_info["e_4dof"][0]:.3f}, {debug_info["e_4dof"][1]:.3f}, '
            f'{debug_info["e_4dof"][2]:.3f}, {np.rad2deg(debug_info["e_4dof"][3]):.1f}°]',
            throttle_duration_sec=2.0
        )
        self.get_logger().info(
            f'[WRENCH] '
            f'F=[{tau_6dof[0]:.1f}, {tau_6dof[1]:.1f}, {tau_6dof[2]:.1f}] | '
            f'M=[{tau_6dof[3]:.1f}, {tau_6dof[4]:.1f}, {tau_6dof[5]:.1f}] | '
            f'Sat: {debug_info["saturated"]}',
            throttle_duration_sec=2.0
        )


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    node = None

    try:
        node = PID4DOFNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Graceful shutdown
    except Exception as e:
        import traceback
        print(f'Error in PID 4DOF Node: {e}')
        traceback.print_exc()
    finally:
        # Ensure node is destroyed before shutdown
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
