#!/usr/bin/env python3
# Copyright 2025

"""
PID Data Logger

Collects data from running PID controller for analysis.
"""

import csv
from pathlib import Path
from transforms3d.euler import quat2euler

from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from stonefish_control_msgs.msg import TrajectoryPoint


class DataLogger:
    """Data logger for PID tuning.

    Subscribes to ROS topics and logs data to CSV file.
    Collects: reference, odometry, error, and wrench data.
    """

    def __init__(self, output_file: str):
        """Initialize data logger.

        Args:
            output_file: Path to output CSV file
        """
        self.output_file = Path(output_file)
        self.output_file.parent.mkdir(parents=True, exist_ok=True)

        # Data buffers
        self.reference_data = {'x': 0.0, 'y': 0.0, 'z': 0.0,
                               'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.error_data = {'x': 0.0, 'y': 0.0, 'z': 0.0,
                           'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.odom_data = {'x': 0.0, 'y': 0.0, 'z': 0.0,
                          'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.wrench_data = {'fx': 0.0, 'fy': 0.0, 'fz': 0.0,
                            'tx': 0.0, 'ty': 0.0, 'tz': 0.0}

        self.data_ready = False

        # Open CSV file
        self.csv_file = open(self.output_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write header
        self.csv_writer.writerow([
            'timestamp',
            'ref_x', 'ref_y', 'ref_z', 'ref_roll', 'ref_pitch', 'ref_yaw',
            'curr_x', 'curr_y', 'curr_z', 'curr_roll', 'curr_pitch', 'curr_yaw',
            'err_x', 'err_y', 'err_z', 'err_roll', 'err_pitch', 'err_yaw',
            'wrench_fx', 'wrench_fy', 'wrench_fz', 'wrench_tx', 'wrench_ty', 'wrench_tz'
        ])

    def reference_callback(self, msg: TrajectoryPoint):
        """Handle reference message."""
        pos = msg.pose.position
        quat = msg.pose.orientation
        roll, pitch, yaw = quat2euler([quat.w, quat.x, quat.y, quat.z])
        self.reference_data = {'x': pos.x, 'y': pos.y, 'z': pos.z,
                               'roll': roll, 'pitch': pitch, 'yaw': yaw}
        self.data_ready = True

    def error_callback(self, msg: TrajectoryPoint):
        """Handle error message."""
        pos = msg.pose.position
        quat = msg.pose.orientation
        roll, pitch, yaw = quat2euler([quat.w, quat.x, quat.y, quat.z])
        self.error_data = {'x': pos.x, 'y': pos.y, 'z': pos.z,
                           'roll': roll, 'pitch': pitch, 'yaw': yaw}
        self.data_ready = True

    def odometry_callback(self, msg: Odometry):
        """Handle odometry message."""
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        roll, pitch, yaw = quat2euler([quat.w, quat.x, quat.y, quat.z])
        self.odom_data = {'x': pos.x, 'y': pos.y, 'z': pos.z,
                          'roll': roll, 'pitch': pitch, 'yaw': yaw}
        self.data_ready = True

    def wrench_callback(self, msg: WrenchStamped):
        """Handle wrench message."""
        self.wrench_data = {
            'fx': msg.wrench.force.x, 'fy': msg.wrench.force.y, 'fz': msg.wrench.force.z,
            'tx': msg.wrench.torque.x, 'ty': msg.wrench.torque.y, 'tz': msg.wrench.torque.z
        }
        self.data_ready = True

    def log(self, timestamp: float):
        """Log current data to CSV.

        Args:
            timestamp: Current timestamp in seconds
        """
        if not self.data_ready:
            return

        row = [
            timestamp,
            self.reference_data['x'], self.reference_data['y'], self.reference_data['z'],
            self.reference_data['roll'], self.reference_data['pitch'], self.reference_data['yaw'],
            self.odom_data['x'], self.odom_data['y'], self.odom_data['z'],
            self.odom_data['roll'], self.odom_data['pitch'], self.odom_data['yaw'],
            self.error_data['x'], self.error_data['y'], self.error_data['z'],
            self.error_data['roll'], self.error_data['pitch'], self.error_data['yaw'],
            self.wrench_data['fx'], self.wrench_data['fy'], self.wrench_data['fz'],
            self.wrench_data['tx'], self.wrench_data['ty'], self.wrench_data['tz']
        ]

        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def close(self):
        """Close the CSV file."""
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()

    def __del__(self):
        """Cleanup on deletion."""
        self.close()
