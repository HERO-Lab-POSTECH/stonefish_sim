#!/usr/bin/env python3
# Copyright 2025

"""
PID Tuning Tool Node

Main ROS2 node for PID tuning data collection and analysis.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from pathlib import Path
import os
import tempfile

from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from stonefish_control_msgs.msg import TrajectoryPoint

from .logger import DataLogger
from .analyzer import DataAnalyzer
from .plotter import DataPlotter


class PIDTuningToolNode(Node):
    """ROS2 node for PID tuning data collection and analysis.

    Subscribes to controller topics and logs data for specified duration.
    Automatically analyzes and plots results when complete.
    """

    def __init__(self):
        """Initialize the tuning tool node."""
        super().__init__('pid_tuning_tool')

        # Parameters
        default_output = os.path.join(tempfile.gettempdir(), 'pid_tuning_data.csv')
        self.declare_parameter('output_file', default_output)
        self.declare_parameter('log_duration', 10.0)
        self.declare_parameter('auto_plot', True)
        self.declare_parameter('auto_analyze', True)

        self.output_file = self.get_parameter('output_file').value
        self.log_duration = self.get_parameter('log_duration').value
        self.auto_plot = self.get_parameter('auto_plot').value
        self.auto_analyze = self.get_parameter('auto_analyze').value

        # Initialize data logger
        self.logger = DataLogger(self.output_file)

        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subscribers
        self.ref_sub = self.create_subscription(
            TrajectoryPoint, 'reference',
            self.logger.reference_callback, qos_profile)

        self.error_sub = self.create_subscription(
            TrajectoryPoint, 'error',
            self._error_callback_wrapper, qos_profile)

        self.odom_sub = self.create_subscription(
            Odometry, 'odometry',
            self.logger.odometry_callback, qos_profile)

        self.wrench_sub = self.create_subscription(
            WrenchStamped, 'thruster_manager/input_stamped',
            self.logger.wrench_callback, qos_profile)

        # Timing
        self.start_time = None
        self.logging_active = False
        self.timer = self.create_timer(0.1, self._timer_callback)

        # Log startup
        self.get_logger().info('=' * 60)
        self.get_logger().info('PID Tuning Tool')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Output file: {self.output_file}')
        self.get_logger().info(f'Duration: {self.log_duration}s')
        self.get_logger().info(f'Auto-plot: {self.auto_plot}')
        self.get_logger().info(f'Auto-analyze: {self.auto_analyze}')
        self.get_logger().info('Waiting for data...')

    def _error_callback_wrapper(self, msg: TrajectoryPoint):
        """Wrapper for error callback that also logs data."""
        self.logger.error_callback(msg)
        self._log_data()

    def _log_data(self):
        """Log current data point."""
        if not self.logger.data_ready:
            return

        # Start logging on first data
        if self.start_time is None:
            self.start_time = self.get_clock().now()
            self.logging_active = True
            self.get_logger().info('Started logging data...')

        # Calculate elapsed time
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # Log data
        self.logger.log(elapsed)

    def _timer_callback(self):
        """Check if logging duration is complete."""
        if not self.logging_active:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if elapsed >= self.log_duration:
            self.get_logger().info(f'Logging complete: {elapsed:.2f}s')
            self.logger.close()

            # Post-processing
            if self.auto_analyze or self.auto_plot:
                self._post_process()

            # Shutdown
            rclpy.shutdown()

    def _post_process(self):
        """Analyze and plot data."""
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('POST-PROCESSING')
        self.get_logger().info('=' * 60)

        try:
            # Analyze
            if self.auto_analyze:
                self.get_logger().info('Analyzing data...')
                analyzer = DataAnalyzer(self.output_file)
                analyzer.analyze()
                analyzer.print_report()

            # Plot
            if self.auto_plot:
                self.get_logger().info('Generating plots...')
                plotter = DataPlotter(self.output_file)
                plot_file = plotter.plot()
                self.get_logger().info(f'✅ Plot saved: {plot_file}')

        except Exception as e:
            self.get_logger().error(f'❌ Post-processing failed: {e}')
            import traceback
            traceback.print_exc()

    def cleanup(self):
        """Cleanup resources."""
        self.logger.close()


def main(args=None):
    """Main entry point for PID tuning tool.

    Usage:
        ros2 run stonefish_control pid_tuning_tool --ros-args \\
            -p output_file:=/path/to/output.csv \\
            -p log_duration:=10.0 \\
            -p auto_plot:=true
    """
    rclpy.init(args=args)

    node = None
    try:
        node = PIDTuningToolNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        if node:
            node.get_logger().info('⚠️  Interrupted by user')

    except Exception as e:
        print(f'❌ Error: {e}')
        import traceback
        traceback.print_exc()

    finally:
        if node:
            node.cleanup()

        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
