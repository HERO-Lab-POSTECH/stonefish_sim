#!/usr/bin/env python3
# Copyright 2025
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Stonefish simulator bringup (leaf launch).
#
# Starts the Stonefish C++ simulator. A boolean `gpu` argument selects the
# rendered GPU build (`stonefish_simulator`) or the headless build
# (`stonefish_simulator_nogpu`). This replaces the former pair of
# simulator_gpu.launch.py / simulator_nogpu.launch.py (the nogpu file was an
# orphan that nothing could reach).
#
# Usage:
#   ros2 launch stonefish_ros2 simulator.launch.py \
#       scenario_desc:=/abs/path/to/world.scn
#   ros2 launch stonefish_ros2 simulator.launch.py gpu:=false   # headless

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the Stonefish simulator."""

    simulation_data = LaunchConfiguration('simulation_data')
    scenario_desc = LaunchConfiguration('scenario_desc')
    simulation_rate = LaunchConfiguration('simulation_rate')
    window_res_x = LaunchConfiguration('window_res_x')
    window_res_y = LaunchConfiguration('window_res_y')
    rendering_quality = LaunchConfiguration('rendering_quality')
    gpu = LaunchConfiguration('gpu')

    args = [
        DeclareLaunchArgument(
            'simulation_data', default_value='',
            description='Root path of simulation assets (stonefish_description share dir).'),
        DeclareLaunchArgument(
            'scenario_desc', default_value='',
            description='Absolute path to the .scn scenario file to load.'),
        DeclareLaunchArgument(
            'simulation_rate', default_value='100.0',
            description='Physics step rate in Hz.'),
        DeclareLaunchArgument(
            'window_res_x', default_value='960',
            description='Render window width in px (GPU build only).'),
        DeclareLaunchArgument(
            'window_res_y', default_value='1056',
            description='Render window height in px (GPU build only).'),
        DeclareLaunchArgument(
            'rendering_quality', default_value='high',
            description='Render quality preset: low|medium|high (GPU build only).'),
        DeclareLaunchArgument(
            'gpu', default_value='true',
            description='true = rendered GPU simulator; false = headless nogpu build.'),
    ]

    # NOTE: the `arguments=` lists below are the C++ executable's positional argv
    # contract — the simulator reads them by index, so the ORDER IS LOAD-BEARING.
    # Do not reorder. GPU build takes 6 argv; the headless build takes 3
    # (no window/rendering argv).
    simulator_gpu_node = Node(
        package='stonefish_ros2',
        executable='stonefish_simulator',
        namespace='stonefish_ros2',
        name='stonefish_simulator',
        arguments=[simulation_data, scenario_desc, simulation_rate,
                   window_res_x, window_res_y, rendering_quality],
        output='screen',
        condition=IfCondition(gpu),
    )

    simulator_nogpu_node = Node(
        package='stonefish_ros2',
        executable='stonefish_simulator_nogpu',
        namespace='stonefish_ros2',
        name='stonefish_simulator_nogpu',
        arguments=[simulation_data, scenario_desc, simulation_rate],
        output='screen',
        condition=UnlessCondition(gpu),
    )

    return LaunchDescription(args + [simulator_gpu_node, simulator_nogpu_node])
