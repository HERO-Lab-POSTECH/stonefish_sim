#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Control stack launch — the hybrid controller only.

This is the "control" half of the runtime split: it brings up the vehicle
controller that turns trajectory setpoints into thrust commands. It launches NO
simulator and NO path nodes — pair it with path.launch.py (path generation +
following) and a simulator/vehicle bringup. The top-level
stonefish_ros2/bringup.launch.py wires all three together.

Usage:
    # control only (sim + path started separately)
    ros2 launch stonefish_control control.launch.py
    ros2 launch stonefish_control control.launch.py vehicle_name:=blueboat use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vehicle_name = LaunchConfiguration('vehicle_name')
    controller_config = LaunchConfiguration('controller_config')
    dynamics_config = LaunchConfiguration('dynamics_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # controller_config/dynamics_config default to vehicle_name-dependent paths,
    # so they MUST resolve at launch time via PathJoinSubstitution — not
    # os.path.join at import time, which would freeze the vehicle segment and
    # silently load the wrong YAML when vehicle_name is overridden.
    args = [
        DeclareLaunchArgument(
            'vehicle_name', default_value='bluerov2',
            description='Vehicle name (also the node namespace).'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock simulation time; set true when the simulator runs.'),
        DeclareLaunchArgument(
            'controller_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('stonefish_control'),
                'config', vehicle_name, 'hybrid_controller.yaml'
            ]),
            description='Path to the hybrid controller config (defaults per vehicle_name).'),
        DeclareLaunchArgument(
            'dynamics_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('stonefish_description'),
                'data', 'robots', vehicle_name, 'config', 'dynamics_params.yaml'
            ]),
            description='Path to the vehicle dynamics config (defaults per vehicle_name).'),
    ]

    hybrid_controller_node = Node(
        package='stonefish_control',
        executable='hybrid_controller_node',
        name='hybrid_controller',
        namespace=vehicle_name,
        output='screen',
        parameters=[dynamics_config, controller_config, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(args + [hybrid_controller_node])
