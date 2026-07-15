#!/usr/bin/env python3
# Copyright 2025
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Fixed-attitude E2E smoke test (Milestone 1): starts the Stonefish
# simulator (albc_empty.scn) and the albc_bridge policy loop together,
# so a rosbag of the run can be judged against the 4 success criteria.
#
# The bridge is TF-independent (frames.py bakes the Stonefish->Isaac
# rotation as a constant, see albc_bridge/albc_bridge/frames.py), so
# simulator.launch.py is used as-is -- no base_link_frd static TF needed.
#
# Usage:
#   ros2 launch albc_bridge albc_smoke.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for the fixed-attitude E2E smoke test."""

    args = [
        DeclareLaunchArgument(
            'scenario_desc',
            default_value='/workspace/src/stonefish_sim/stonefish_description/scenarios/albc_empty.scn',
            description='Absolute path to the .scn scenario file to load.'),
        DeclareLaunchArgument(
            'simulation_data',
            default_value='/workspace/src/stonefish_sim/stonefish_description/',
            description='Root path of simulation assets (stonefish_description share dir).'),
        DeclareLaunchArgument(
            'simulation_rate', default_value='100.0',
            description='Physics step rate in Hz.'),
        DeclareLaunchArgument(
            'gpu', default_value='false',
            description='true = rendered GPU simulator; false = headless nogpu build.'),
    ]

    # 1. Stonefish simulator loading albc_empty.scn.
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('stonefish_ros2'), 'launch', 'simulator.launch.py'
            ])
        ),
        launch_arguments={
            'scenario_desc': LaunchConfiguration('scenario_desc'),
            'simulation_data': LaunchConfiguration('simulation_data'),
            'simulation_rate': LaunchConfiguration('simulation_rate'),
            'gpu': LaunchConfiguration('gpu'),
        }.items(),
    )

    # 2. albc_bridge policy loop (fixed attitude command, ang_cmd default [0,0,0]).
    bridge = Node(
        package='albc_bridge',
        executable='albc_bridge',
        name='albc_bridge',
        parameters=[{'ang_cmd': [0.0, 0.0, 0.0]}],
    )

    return LaunchDescription(args + [simulator, bridge])
