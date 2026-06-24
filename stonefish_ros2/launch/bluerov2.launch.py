#!/usr/bin/env python3
# Copyright 2025
# SPDX-License-Identifier: GPL-3.0-or-later
#
# BlueROV2 (underwater ROV) simulation — thin wrapper over vehicle.launch.py.
#
# Starts the Stonefish simulator with the BlueROV2 scenario, the thruster
# manager (optional), and the base_link->base_link_frd static TF. All shared
# logic lives in vehicle.launch.py; this file only pins BlueROV2 defaults.
#
# Usage:
#   ros2 launch stonefish_ros2 bluerov2.launch.py
#   ros2 launch stonefish_ros2 bluerov2.launch.py start_thruster_manager:=false

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for the BlueROV2 simulation."""
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('stonefish_ros2'), 'launch', 'vehicle.launch.py'
                ])
            ),
            launch_arguments={
                'vehicle_name': 'bluerov2',
                'scenario': 'bluerov2_infrastructure',
                'window_res_y': '1080',
                'enable_base_link_frd': 'true',
            }.items(),
        ),
    ])
