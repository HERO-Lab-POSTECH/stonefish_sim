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
Thruster Allocator Node for Stonefish Simulator.

Converts 6DOF Wrench commands to individual thruster forces using TAM.
Publishes thruster forces as Float64MultiArray for Stonefish.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
from pathlib import Path

from geometry_msgs.msg import Wrench, WrenchStamped
from std_msgs.msg import Float64MultiArray

from .thruster_manager import ThrusterManager


class ThrusterAllocatorNode(Node):
    """
    Thruster allocator node for Stonefish simulator.

    Subscribes to:
        - ~/input (geometry_msgs/Wrench): 6DOF wrench command
        - ~/input_stamped (geometry_msgs/WrenchStamped): Stamped wrench command

    Publishes:
        - ~/thruster_forces (std_msgs/Float64MultiArray): Individual thruster forces
    """

    def __init__(self):
        """Initialize thruster allocator node."""
        super().__init__('thruster_allocator')

        # Declare parameters
        self.declare_parameter('tam_file', '')
        self.declare_parameter('vehicle_name', 'bluerov2')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('max_thrust', 200.0)  # PWM normalization scale (NOT physical limit)

        # Get parameters
        tam_file = self.get_parameter('tam_file').value
        self.vehicle_name = self.get_parameter('vehicle_name').value
        self.base_link = self.get_parameter('base_link').value

        # Output topic: dynamically generated from vehicle_name
        # Matches Stonefish scenario: <ros_subscriber thrusters="/$(vehicle_name)/setpoint/pwm"/>
        self.output_topic = f'/{self.vehicle_name}/setpoint/pwm'
        update_rate = self.get_parameter('update_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.max_thrust = self.get_parameter('max_thrust').value

        # Initialize TAM if file is provided
        if tam_file:
            tam_path = Path(tam_file)
        else:
            # Default TAM path based on vehicle name
            # Assumes package stonefish_description is installed
            from ament_index_python.packages import get_package_share_directory
            try:
                stonefish_desc_path = get_package_share_directory('stonefish_description')
                tam_path = Path(stonefish_desc_path) / 'data' / 'robots' / self.vehicle_name / 'config' / 'TAM.yaml'
            except Exception as e:
                self.get_logger().error(f'Could not find default TAM file: {e}')
                raise

        self.get_logger().info(f'Loading TAM from: {tam_path}')

        try:
            self.tam_manager = ThrusterManager(tam_file_path=str(tam_path))
            self.get_logger().info(
                f'TAM loaded successfully: {self.tam_manager.n_thrusters} thrusters'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to load TAM: {e}')
            raise

        # Initialize state
        self.last_wrench_time = self.get_clock().now()
        self.thrust_forces = np.zeros(self.tam_manager.n_thrusters)  # Force in Newton
        self.thrust_pwm = np.zeros(self.tam_manager.n_thrusters)     # PWM setpoint (-1 to 1)

        # Create QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Create subscribers
        # Using only WrenchStamped to avoid type conflicts on same topic
        # Relative name: resolved to /{namespace}/thruster_manager/input_stamped
        self.wrench_stamped_sub = self.create_subscription(
            WrenchStamped,
            'thruster_manager/input_stamped',
            self.wrench_stamped_callback,
            qos_profile
        )

        # Create publisher
        self.thruster_pub = self.create_publisher(
            Float64MultiArray,
            self.output_topic,
            qos_profile
        )

        # Create timer for timeout checking
        timer_period = 1.0 / update_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'Thruster allocator initialized for {self.vehicle_name}'
        )
        self.get_logger().info(f'Output topic: {self.output_topic}')
        self.get_logger().info(f'Update rate: {update_rate} Hz')
        self.get_logger().info(f'Timeout: {self.timeout} s')

    def wrench_callback(self, msg):
        """
        Callback for Wrench messages.

        Args:
            msg (Wrench): 6DOF wrench command.
        """
        wrench = np.array([
            msg.force.x,
            msg.force.y,
            msg.force.z,
            msg.torque.x,
            msg.torque.y,
            msg.torque.z
        ])

        self.process_wrench(wrench)

    def wrench_stamped_callback(self, msg):
        """
        Callback for WrenchStamped messages.

        Args:
            msg (WrenchStamped): Stamped 6DOF wrench command.
        """
        wrench = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z
        ])

        self.process_wrench(wrench)

    def process_wrench(self, wrench):
        """
        Process wrench and compute thruster forces.

        Args:
            wrench (np.ndarray): 6DOF wrench [Fx, Fy, Fz, Tx, Ty, Tz].
        """
        # Compute thrust forces using TAM pseudo-inverse
        self.thrust_forces = self.tam_manager.compute_thrust_forces(wrench)

        # Apply saturation (Force in Newton)
        self.thrust_forces = np.clip(
            self.thrust_forces,
            -self.max_thrust,
            self.max_thrust
        )

        # Normalize to PWM range (-1.0 to 1.0) for Stonefish
        self.thrust_pwm = self.thrust_forces / self.max_thrust

        # Publish PWM setpoints
        self.publish_thrust_forces()

        # Update last command time
        self.last_wrench_time = self.get_clock().now()

    def publish_thrust_forces(self):
        """Publish thruster PWM setpoints to Stonefish."""
        msg = Float64MultiArray()
        msg.data = self.thrust_pwm.tolist()
        self.thruster_pub.publish(msg)

    def timer_callback(self):
        """Timer callback for timeout checking."""
        if self.timeout <= 0:
            return

        # Check if last command was too long ago
        time_since_last = (self.get_clock().now() - self.last_wrench_time).nanoseconds / 1e9

        if time_since_last > self.timeout:
            # Zero thrust forces on timeout
            if np.any(self.thrust_forces != 0):
                self.get_logger().warn(
                    f'No wrench command for {time_since_last:.2f}s - zeroing thrusters'
                )
                self.thrust_forces = np.zeros(self.tam_manager.n_thrusters)
                self.thrust_pwm = np.zeros(self.tam_manager.n_thrusters)
                self.publish_thrust_forces()


def main(args=None):
    """Main entry point for thruster allocator node."""
    rclpy.init(args=args)

    try:
        node = ThrusterAllocatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
