#!/usr/bin/env python3
"""
PID Optimizer Node

Main executable for running PID optimization with different methods.
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
import time
import threading
from pathlib import Path

from .base import load_yaml_config
from .gwo import GWOOptimizer
from .smac3 import SMAC3Optimizer


class PIDOptimizerNode:
    """Main ROS2 node for PID optimization

    Loads configuration and runs the selected optimization method.
    """

    def __init__(self):
        """Initialize the optimizer node"""
        # Initialize ROS2
        rclpy.init()

        # Create temporary node for getting parameters
        temp_node = rclpy.create_node('pid_optimizer_loader')

        # Declare and get config file parameter
        temp_node.declare_parameter('config_file', '')
        config_file = temp_node.get_parameter('config_file').value

        if not config_file:
            temp_node.get_logger().error('❌ No config_file parameter provided!')
            temp_node.get_logger().error('   Usage: ros2 run stonefish_control pid_optimizer --ros-args -p config_file:=/path/to/config.yaml')
            temp_node.destroy_node()
            rclpy.shutdown()
            raise ValueError('config_file parameter required')

        # Load configuration
        config_path = Path(config_file)
        if not config_path.exists():
            temp_node.get_logger().error(f'❌ Config file not found: {config_file}')
            temp_node.destroy_node()
            rclpy.shutdown()
            raise FileNotFoundError(f'Config file not found: {config_file}')

        temp_node.get_logger().info(f'📁 Loading config: {config_file}')
        config = load_yaml_config(str(config_path))

        # Cleanup temporary node
        temp_node.destroy_node()

        # Get optimization method
        method = config['optimizer']['method'].lower()

        # Create optimizer based on method
        if method == 'gwo':
            self.optimizer = GWOOptimizer(config)
        elif method == 'smac3':
            self.optimizer = SMAC3Optimizer(config)
        else:
            rclpy.shutdown()
            raise ValueError(f"Unknown optimization method: {method}. Must be 'gwo' or 'smac3'")

        self.optimizer.get_logger().info(f'✅ Initialized {method.upper()} optimizer')

    def run(self):
        """Execute optimization"""
        try:
            # Run optimization (startup logs are in optimize() method)
            best_gains, best_cost = self.optimizer.optimize()

            # Log completion (only if we have valid results)
            if best_gains is not None:
                self.optimizer.get_logger().info('=' * 60)
                self.optimizer.get_logger().info('✅ Optimization Complete!')
                self.optimizer.get_logger().info(f'📊 Best Cost: {best_cost:.4f}')
                self.optimizer.get_logger().info(f'📁 Results: {self.optimizer.results_dir}')
                self.optimizer.get_logger().info('=' * 60)

            return best_gains, best_cost

        except KeyboardInterrupt:
            self.optimizer.get_logger().info('⚠️  Optimization interrupted by user')
            # Set interrupt flag
            self.optimizer.set_interrupted()
            raise

        except Exception as e:
            self.optimizer.get_logger().error(f'❌ Optimization failed: {e}')
            import traceback
            traceback.print_exc()
            raise


def main(args=None):
    """Main entry point

    This function is called when running:
        ros2 run stonefish_control pid_optimizer --ros-args -p config_file:=/path/to/config.yaml

    Args:
        args: Command-line arguments (typically None)
    """
    optimizer_node = None
    opt_thread = None

    try:
        # Create optimizer node
        optimizer_node = PIDOptimizerNode()

        # Create executor for ROS2 callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(optimizer_node.optimizer)

        # Run optimization in separate thread
        opt_thread = threading.Thread(target=optimizer_node.run, daemon=False)
        opt_thread.start()

        # Spin ROS2 node (handles callbacks)
        try:
            executor.spin()
        except KeyboardInterrupt:
            optimizer_node.optimizer.get_logger().info('⚠️  Received interrupt signal...')
            # Set interrupt flag to stop optimization gracefully
            optimizer_node.optimizer.set_interrupted()

        # Wait for optimization thread to complete
        optimizer_node.optimizer.get_logger().info('⏳ Waiting for optimization to finish...')
        opt_thread.join(timeout=10.0)

        if opt_thread.is_alive():
            optimizer_node.optimizer.get_logger().warning('⚠️  Optimization thread did not finish in time')

    except KeyboardInterrupt:
        # Second Ctrl+C - force quit
        print('\n⚠️  Force quit requested')
        if optimizer_node:
            optimizer_node.optimizer.set_interrupted()

    except Exception as e:
        print(f'❌ Fatal error: {e}')
        import traceback
        traceback.print_exc()

    finally:
        # Cleanup optimizer resources (WandB, subprocess)
        if optimizer_node is not None:
            try:
                optimizer_node.optimizer.cleanup()
            except Exception as e:
                print(f'Warning: Cleanup failed: {e}')

        # Cleanup ROS2
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
