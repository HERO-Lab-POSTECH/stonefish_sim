#!/usr/bin/env python3
# Copyright (c) 2025 Stonefish Control Contributors
# Licensed under the Apache License, Version 2.0

"""
Unified 6DOF Controller ROS2 Node

ROS2 wrapper for Unified6DOFController. Handles:
- Odometry subscription (vehicle state)
- Command subscription (trajectory reference)
- Mode subscription (control mode switching)
- Wrench publication (thruster manager input)

Reference:
- Fossen (2011) "Handbook of Marine Craft Hydrodynamics and Motion Control"
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String
from std_srvs.srv import Empty
from stonefish_control_msgs.msg import TrajectoryPoint

import numpy as np
from transforms3d.euler import quat2euler

from .unified_controller import Unified6DOFController, ControlMode
from ..control_interfaces.data_types import (
    ControlGains,
    ControlLimits,
    VehicleParams,
    TrajectoryReference,
    VehicleState,
    OuterLoopGains,
    InnerLoopGains,
    OuterLoopLimits,
    InnerLoopLimits
)


class UnifiedControllerNode(Node):
    """ROS2 Node for Unified 6DOF Controller.

    Subscribers:
        /{vehicle}/odometry (Odometry): Vehicle state
        /{vehicle}/cmd_pose (TrajectoryPoint): Trajectory reference
        /{vehicle}/control_mode (String): Control mode

    Publishers:
        /{vehicle}/thruster_manager/input_stamped (WrenchStamped): Control output

    Services:
        /{vehicle}/controller/reset (Empty): Reset controller state
    """

    def __init__(self):
        super().__init__('unified_controller')

        # Declare and load parameters
        self._declare_parameters()
        self._load_parameters()

        # Initialize controller
        self._init_controller()

        # State tracking
        self._state_received = False
        self._reference_received = False
        self._current_state: VehicleState = None
        self._current_reference: TrajectoryReference = None
        self._last_update_time: Time = None

        # Setup ROS2 interfaces
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_services()
        self._setup_timer()

        self.get_logger().info(
            f'[UnifiedController] Initialized - mode: {self._initial_mode} | '
            f'rate: {self._control_rate:.0f}Hz | '
            f'vehicle: {self._vehicle_name}'
        )

    def _declare_parameters(self):
        """Declare all ROS2 parameters."""
        # General
        self.declare_parameter('vehicle_name', 'bluerov2')
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('initial_mode', 'velocity')

        # Outer loop (position → velocity)
        self.declare_parameter('outer_loop.Kp', [0.5, 0.5, 0.8, 0.5])
        self.declare_parameter('outer_loop.max_velocity', [0.5, 0.5, 0.4, 0.5])

        # Inner loop (velocity → force/torque)
        self.declare_parameter('inner_loop.Kp', [200.0, 200.0, 250.0, 150.0])
        self.declare_parameter('inner_loop.Ki', [50.0, 50.0, 60.0, 10.0])
        self.declare_parameter('inner_loop.Kb', [0.8, 0.8, 0.8, 0.8])
        self.declare_parameter('inner_loop.max_force', 200.0)
        self.declare_parameter('inner_loop.max_torque', 50.0)

        # Vehicle parameters
        self.declare_parameter('mass', 20.0)
        self.declare_parameter('inertial.izz', 0.13)

        # Feedforward
        self.declare_parameter('feedforward.enabled', True)
        self.declare_parameter('feedforward.gain', 0.8)

    def _load_parameters(self):
        """Load parameters from ROS2 parameter server."""
        # General
        self._vehicle_name = self.get_parameter('vehicle_name').value
        self._control_rate = self.get_parameter('control_rate').value
        self._initial_mode = self.get_parameter('initial_mode').value

        # Outer loop gains
        outer_Kp = np.array(self.get_parameter('outer_loop.Kp').value)
        outer_max_vel = np.array(self.get_parameter('outer_loop.max_velocity').value)

        # Inner loop gains
        inner_Kp = np.array(self.get_parameter('inner_loop.Kp').value)
        inner_Ki = np.array(self.get_parameter('inner_loop.Ki').value)
        inner_Kb = np.array(self.get_parameter('inner_loop.Kb').value)
        max_force = self.get_parameter('inner_loop.max_force').value
        max_torque = self.get_parameter('inner_loop.max_torque').value

        # Vehicle parameters
        mass = self.get_parameter('mass').value
        izz = self.get_parameter('inertial.izz').value

        # Feedforward
        ff_enabled = self.get_parameter('feedforward.enabled').value
        ff_gain = self.get_parameter('feedforward.gain').value if ff_enabled else 0.0

        # Build configuration objects
        self._gains = ControlGains(
            outer=OuterLoopGains(Kp=outer_Kp),
            inner=InnerLoopGains(Kp=inner_Kp, Ki=inner_Ki, Kb=inner_Kb)
        )

        self._limits = ControlLimits(
            outer=OuterLoopLimits(max_velocity=outer_max_vel),
            inner=InnerLoopLimits(max_force=max_force, max_torque=max_torque)
        )

        self._vehicle_params = VehicleParams(mass=mass, inertia_zz=izz)
        self._feedforward_gain = ff_gain

        # Log loaded parameters
        self.get_logger().info(
            f'[UnifiedController] Gains loaded - '
            f'outer_Kp: {outer_Kp.tolist()} | '
            f'inner_Kp: {inner_Kp.tolist()} | '
            f'inner_Ki: {inner_Ki.tolist()}'
        )

    def _init_controller(self):
        """Initialize the unified controller."""
        self._controller = Unified6DOFController(
            gains=self._gains,
            limits=self._limits,
            vehicle_params=self._vehicle_params,
            control_mode=self._initial_mode,
            feedforward_gain=self._feedforward_gain
        )

    def _setup_subscribers(self):
        """Setup ROS2 subscribers."""
        # QoS profiles
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Odometry subscriber
        self._odom_sub = self.create_subscription(
            Odometry,
            f'/{self._vehicle_name}/odometry',
            self._odom_callback,
            state_qos
        )

        # Command subscriber (from path following)
        self._cmd_sub = self.create_subscription(
            TrajectoryPoint,
            f'/{self._vehicle_name}/cmd_pose',
            self._cmd_callback,
            reliable_qos
        )

        # Control mode subscriber
        self._mode_sub = self.create_subscription(
            String,
            f'/{self._vehicle_name}/control_mode',
            self._mode_callback,
            reliable_qos
        )

    def _setup_publishers(self):
        """Setup ROS2 publishers."""
        # Wrench publisher (to thruster allocator)
        self._wrench_pub = self.create_publisher(
            WrenchStamped,
            f'/{self._vehicle_name}/thruster_manager/input_stamped',
            10
        )

    def _setup_services(self):
        """Setup ROS2 services."""
        # Reset service
        self._reset_srv = self.create_service(
            Empty,
            f'/{self._vehicle_name}/controller/reset',
            self._reset_callback
        )

    def _setup_timer(self):
        """Setup control loop timer."""
        timer_period = 1.0 / self._control_rate
        self._timer = self.create_timer(timer_period, self._control_callback)

    def _odom_callback(self, msg: Odometry):
        """Handle odometry message.

        Args:
            msg: Odometry message with pose and velocity
        """
        # Extract pose [x, y, z, roll, pitch, yaw]
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation

        # Convert quaternion to euler (sxyz convention for NED)
        roll, pitch, yaw = quat2euler([quat.w, quat.x, quat.y, quat.z], 'sxyz')

        pose = np.array([pos.x, pos.y, pos.z, roll, pitch, yaw])

        # Extract velocity [u, v, w, p, q, r] - body frame
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular
        velocity = np.array([lin.x, lin.y, lin.z, ang.x, ang.y, ang.z])

        # Update state
        self._current_state = VehicleState(pose=pose, velocity=velocity)

        if not self._state_received:
            self._state_received = True
            self.get_logger().info('[UnifiedController] Odometry received')

    def _cmd_callback(self, msg: TrajectoryPoint):
        """Handle trajectory command message.

        Args:
            msg: TrajectoryPoint with desired pose and velocity
        """
        # Extract position [x, y, z, yaw]
        pos = msg.pose.position
        quat = msg.pose.orientation
        _, _, yaw = quat2euler([quat.w, quat.x, quat.y, quat.z], 'sxyz')

        position = np.array([pos.x, pos.y, pos.z, yaw])

        # Extract velocity [u, v, w, r]
        lin = msg.velocity.linear
        ang = msg.velocity.angular
        velocity = np.array([lin.x, lin.y, lin.z, ang.z])

        # Extract acceleration if available (for feedforward)
        acceleration = None
        if hasattr(msg, 'acceleration'):
            acc_lin = msg.acceleration.linear
            acc_ang = msg.acceleration.angular
            acceleration = np.array([acc_lin.x, acc_lin.y, acc_lin.z, acc_ang.z])

        # Update reference
        self._current_reference = TrajectoryReference(
            position=position,
            velocity=velocity,
            acceleration=acceleration
        )

        if not self._reference_received:
            self._reference_received = True
            self.get_logger().info('[UnifiedController] Command received')

    def _mode_callback(self, msg: String):
        """Handle control mode message.

        Args:
            msg: String message with mode name
        """
        mode = msg.data.lower()

        # Map 'hybrid' to 'velocity' if controller doesn't support hybrid
        # (or keep as-is if supported)
        valid_modes = ['position', 'velocity', 'hybrid']

        if mode not in valid_modes:
            self.get_logger().warn(f'Unknown control mode: {mode}, using velocity')
            mode = 'velocity'

        try:
            old_mode = self._controller.mode.value
            self._controller.set_mode(mode)
            if old_mode != mode:
                self.get_logger().info(f'[UnifiedController] Mode: {old_mode} → {mode}')
        except ValueError as e:
            self.get_logger().error(f'Failed to set mode: {e}')

    def _control_callback(self):
        """Control loop timer callback."""
        if not self._state_received or not self._reference_received:
            return

        if self._current_state is None or self._current_reference is None:
            return

        # Calculate dt
        current_time = self.get_clock().now()
        if self._last_update_time is None:
            dt = 1.0 / self._control_rate
        else:
            dt = (current_time - self._last_update_time).nanoseconds / 1e9
        self._last_update_time = current_time

        # Clamp dt to reasonable range
        dt = np.clip(dt, 0.001, 0.1)

        # Compute control
        output = self._controller.compute(
            reference=self._current_reference,
            state=self._current_state,
            dt=dt
        )

        # Publish wrench
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = current_time.to_msg()
        wrench_msg.header.frame_id = f'{self._vehicle_name}/base_link'

        wrench_msg.wrench.force.x = float(output.tau_6dof[0])
        wrench_msg.wrench.force.y = float(output.tau_6dof[1])
        wrench_msg.wrench.force.z = float(output.tau_6dof[2])
        wrench_msg.wrench.torque.x = float(output.tau_6dof[3])
        wrench_msg.wrench.torque.y = float(output.tau_6dof[4])
        wrench_msg.wrench.torque.z = float(output.tau_6dof[5])

        self._wrench_pub.publish(wrench_msg)

        # Throttled logging
        self._log_status(output, dt)

    def _log_status(self, output, dt):
        """Log controller status (throttled).

        Args:
            output: Control output
            dt: Time step
        """
        info = output.info

        # Log every 2 seconds
        self.get_logger().info(
            f"[{info['mode'].upper()}] "
            f"vel_cmd: [{info['vel_cmd'][0]:.2f}, {info['vel_cmd'][1]:.2f}, "
            f"{info['vel_cmd'][2]:.2f}, {info['vel_cmd'][3]:.2f}] | "
            f"tau: [{output.tau_6dof[0]:.1f}, {output.tau_6dof[1]:.1f}, "
            f"{output.tau_6dof[2]:.1f}, {output.tau_6dof[5]:.1f}] | "
            f"sat: {info['inner_loop']['saturated']}",
            throttle_duration_sec=2.0
        )

    def _reset_callback(self, request, response):
        """Handle reset service request.

        Args:
            request: Empty request
            response: Empty response

        Returns:
            Empty response
        """
        self._controller.reset()
        self._state_received = False
        self._reference_received = False
        self._last_update_time = None

        self.get_logger().info('[UnifiedController] Reset complete')
        return response


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = None

    try:
        node = UnifiedControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        import traceback
        print(f'Error: {e}')
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Already shutdown


if __name__ == '__main__':
    main()
