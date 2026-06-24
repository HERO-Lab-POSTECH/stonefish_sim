#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Path stack launch — path generation + path following.

This is the "path" half of the runtime split: it loads waypoints, generates the
reference path, and runs ILOS path following to emit trajectory setpoints. It
also publishes the world_ned -> map static TF used by RViz/visualization. It
launches NO simulator and NO controller — pair it with control.launch.py (the
hybrid controller) and a simulator/vehicle bringup. The top-level
stonefish_ros2/bringup.launch.py wires all three together.

Usage:
    # path only (sim + control started separately)
    ros2 launch stonefish_trajectory_manager path.launch.py
    ros2 launch stonefish_trajectory_manager path.launch.py waypoint_file:=/path/to/wps.yaml use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    trajectory_manager_share = FindPackageShare('stonefish_trajectory_manager')

    vehicle_name = LaunchConfiguration('vehicle_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument(
            'waypoint_file',
            default_value=PathJoinSubstitution([
                trajectory_manager_share, 'config', 'examples', 'krit_lawnmower.yaml'
            ]),
            description='Path to the YAML waypoint file.'),
        DeclareLaunchArgument(
            'vehicle_name', default_value='bluerov2',
            description='Vehicle name/namespace (path following runs in this namespace).'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock simulation time; set true when the simulator runs.'),
        # Path generator tuning args. Defaults mirror path_generator.yaml so the
        # override dict is behavior-equivalent to the YAML unless set explicitly.
        DeclareLaunchArgument(
            'interpolation_method', default_value='lipb',
            description='Interpolation method: linear, lipb, cubic, or cs.'),
        DeclareLaunchArgument(
            'publish_rate', default_value='1.0',
            description='Marker publish rate in Hz.'),
        DeclareLaunchArgument(
            'sample_step', default_value='0.01',
            description='Parametric step size for path sampling.'),
        DeclareLaunchArgument(
            'initial_waypoint_distance_threshold', default_value='0.5',
            description='Distance threshold to add the robot position as WP0 (m).'),
    ]

    path_generator_config = PathJoinSubstitution([
        trajectory_manager_share, 'config', 'path_generator.yaml'
    ])
    path_following_config = PathJoinSubstitution([
        trajectory_manager_share, 'config', 'path_following.yaml'
    ])

    # Static TF: world_ned -> map. Connects Stonefish's world_ned frame to the
    # standard map frame used by RViz/visualization. Identity (no rotation).
    tf_world_ned_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_ned_to_map_broadcaster',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world_ned',
            '--child-frame-id', 'map'
        ],
        output='screen',
    )

    path_generator_node = Node(
        package='stonefish_trajectory_manager',
        executable='path_generator_node',
        name='path_generator_node',
        output='screen',
        parameters=[
            path_generator_config,
            {
                'waypoint_file': LaunchConfiguration('waypoint_file'),
                'vehicle_name': vehicle_name,
                'interpolation_method': LaunchConfiguration('interpolation_method'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'sample_step': LaunchConfiguration('sample_step'),
                'initial_waypoint_distance_threshold':
                    LaunchConfiguration('initial_waypoint_distance_threshold'),
                'use_sim_time': use_sim_time,
            }
        ],
    )

    path_following_node = Node(
        package='stonefish_trajectory_manager',
        executable='path_following_node',
        name='path_following_node',
        namespace=vehicle_name,
        output='screen',
        parameters=[path_following_config, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(
        args + [tf_world_ned_to_map, path_generator_node, path_following_node]
    )
