#!/usr/bin/env python3
# Copyright 2025
# SPDX-License-Identifier: GPL-3.0-or-later
#
# BlueBoat (surface vehicle) simulation — thin wrapper over vehicle.launch.py.
#
# Starts the Stonefish simulator with the BlueBoat scenario and the thruster
# manager (optional). BlueBoat is a surface vehicle, so it does NOT publish the
# base_link_frd TF. All shared logic lives in vehicle.launch.py.
#
# Usage:
#   ros2 launch stonefish_ros2 blueboat.launch.py
#   ros2 launch stonefish_ros2 blueboat.launch.py start_thruster_manager:=false

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for the BlueBoat simulation."""
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('stonefish_ros2'), 'launch', 'vehicle.launch.py'
                ])
            ),
            launch_arguments={
                'vehicle_name': 'blueboat',
                'scenario': 'blueboat_sea',
                'window_res_y': '1056',
                'enable_base_link_frd': 'false',
            }.items(),
        ),
    ])
