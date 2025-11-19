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
Path Following Launch File (ILOS + Hybrid Controller)

Usage:
    ros2 launch stonefish_trajectory_manager path_following.launch.py
    ros2 launch stonefish_trajectory_manager path_following.launch.py waypoint_file:=/path/to/waypoints.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    trajectory_manager_share = FindPackageShare('stonefish_trajectory_manager')
    control_share = FindPackageShare('stonefish_control')

    # =========================================
    # Launch Arguments
    # =========================================
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=PathJoinSubstitution([
            trajectory_manager_share, 'config', 'examples', 'krit_lawnmower.yaml'
        ]),
        description='Path to YAML waypoint file'
    )

    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='bluerov2',
        description='Vehicle name/namespace'
    )

    start_simulator_arg = DeclareLaunchArgument(
        'start_simulator',
        default_value='false',
        description='Start Stonefish simulator (set to true if not already running)'
    )

    # Configuration files
    path_generator_config = PathJoinSubstitution([
        trajectory_manager_share, 'config', 'path_generator.yaml'
    ])

    path_following_config = PathJoinSubstitution([
        trajectory_manager_share, 'config', 'path_following.yaml'
    ])

    hybrid_controller_config = PathJoinSubstitution([
        control_share, 'config', 'bluerov2', 'hybrid_controller.yaml'
    ])

    dynamics_config = PathJoinSubstitution([
        FindPackageShare('stonefish_description'),
        'data', 'robots', 'bluerov2', 'config', 'dynamics_params.yaml'
    ])

    # =========================================
    # Nodes
    # =========================================
    # Path generator node
    path_generator_node = Node(
        package='stonefish_trajectory_manager',
        executable='path_generator_node',
        name='path_generator_node',
        output='screen',
        parameters=[
            path_generator_config,
            {
                'waypoint_file': LaunchConfiguration('waypoint_file'),
                'vehicle_name': LaunchConfiguration('vehicle_name'),
            }
        ]
    )

    # Path following node
    path_following_node = Node(
        package='stonefish_trajectory_manager',
        executable='path_following_node',
        name='path_following_node',
        namespace=LaunchConfiguration('vehicle_name'),
        output='screen',
        parameters=[path_following_config]
    )

    # Hybrid controller node
    hybrid_controller_node = Node(
        package='stonefish_control',
        executable='hybrid_controller_node',
        name='hybrid_controller',
        namespace=LaunchConfiguration('vehicle_name'),
        output='screen',
        parameters=[dynamics_config, hybrid_controller_config]
    )

    # =========================================
    # Optional: Stonefish Simulator
    # =========================================
    # Note: Simulator launch includes thruster allocator automatically
    stonefish_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('stonefish_ros2'),
                'launch',
                'bluerov2.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('start_simulator'))
    )

    return LaunchDescription([
        # Arguments
        waypoint_file_arg,
        vehicle_name_arg,
        start_simulator_arg,

        # Simulator (optional, includes thruster allocator)
        stonefish_launch,

        # Path following pipeline (hybrid architecture)
        path_generator_node,
        path_following_node,
        hybrid_controller_node,
    ])
