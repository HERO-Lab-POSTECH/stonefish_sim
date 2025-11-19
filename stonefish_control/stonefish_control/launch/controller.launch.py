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
Hybrid Controller Launch File

Usage:
    ros2 launch stonefish_control controller.launch.py
    ros2 launch stonefish_control controller.launch.py start_simulator:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    stonefish_control_dir = get_package_share_directory('stonefish_control')
    stonefish_description_dir = get_package_share_directory('stonefish_description')

    # Launch arguments
    start_simulator_arg = DeclareLaunchArgument(
        'start_simulator',
        default_value='false',
        description='Start Stonefish simulator (set to true if not already running)'
    )

    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='bluerov2',
        description='Vehicle name (namespace)'
    )

    controller_config_arg = DeclareLaunchArgument(
        'controller_config',
        default_value=os.path.join(
            stonefish_control_dir, 'config', 'bluerov2', 'hybrid_controller.yaml'
        ),
        description='Path to hybrid controller configuration file'
    )

    dynamics_config_arg = DeclareLaunchArgument(
        'dynamics_config',
        default_value=os.path.join(
            stonefish_description_dir, 'data', 'robots', 'bluerov2', 'config', 'dynamics_params.yaml'
        ),
        description='Path to vehicle dynamics configuration file'
    )

    # LaunchConfiguration variables
    start_simulator = LaunchConfiguration('start_simulator')
    vehicle_name = LaunchConfiguration('vehicle_name')
    controller_config = LaunchConfiguration('controller_config')
    dynamics_config = LaunchConfiguration('dynamics_config')

    # =====================================================
    # Stonefish Simulator (Optional)
    # =====================================================
    stonefish_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('stonefish_ros2'),
                'launch',
                'bluerov2.launch.py'
            )
        ),
        launch_arguments={
            'vehicle_name': vehicle_name,
        }.items(),
        condition=IfCondition(start_simulator)
    )

    # Hybrid Controller
    hybrid_controller_node = Node(
        package='stonefish_control',
        executable='hybrid_controller_node',
        name='hybrid_controller',
        namespace=vehicle_name,
        output='screen',
        parameters=[dynamics_config, controller_config]
    )

    return LaunchDescription([
        # Launch arguments
        start_simulator_arg,
        vehicle_name_arg,
        controller_config_arg,
        dynamics_config_arg,

        # Nodes
        stonefish_launch,         # Optional (only if start_simulator:=true)
        hybrid_controller_node,
    ])
