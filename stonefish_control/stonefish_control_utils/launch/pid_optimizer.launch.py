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

"""Launch file for PID optimizer with controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for PID optimizer.

    This launch file starts:
    1. PID controller (the one being optimized)
    2. PID optimizer node (performs optimization)

    The thruster allocator should be launched separately via stonefish_ros2.
    """

    # Declare arguments
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='bluerov2',
        description='Name of the vehicle (used as namespace)')

    optimizer_config_arg = DeclareLaunchArgument(
        'optimizer_config',
        default_value='gwo_optimizer.yaml',
        description='Optimizer config file name (in config/optimizer/ directory)')

    controller_config_arg = DeclareLaunchArgument(
        'controller_config',
        default_value=[LaunchConfiguration('vehicle_name'), '/pid_params.yaml'],
        description='PID controller config file name (in config/ directory, defaults to {vehicle_name}/pid_params.yaml)')

    # Get launch configurations
    vehicle_name = LaunchConfiguration('vehicle_name')
    optimizer_config = LaunchConfiguration('optimizer_config')
    controller_config = LaunchConfiguration('controller_config')

    # Path to optimizer config file
    optimizer_config_path = PathJoinSubstitution([
        FindPackageShare('stonefish_control_utils'),
        'config', 'optimizer',
        optimizer_config
    ])

    # Include PID controller launch file
    # This PID controller is used for:
    #   1. Step response scenarios (optimizer controls it via set_pid_params service)
    #   2. Trajectory scenarios also use this, but path_following.launch.py is launched with start_pid:=false
    pid_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('stonefish_control'),
                'launch',
                'pid_control.launch.py'
            ])
        ]),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'config_file': controller_config,
        }.items()
    )

    # PID optimizer node
    # Note: For trajectory scenarios, this will auto-launch path_following.launch.py with start_pid:=false
    pid_optimizer_node = Node(
        package='stonefish_control_utils',
        executable='pid_optimizer',
        name='pid_optimizer',
        output='screen',
        parameters=[
            {'config_file': optimizer_config_path}
        ],
    )

    return LaunchDescription([
        vehicle_name_arg,
        optimizer_config_arg,
        controller_config_arg,
        pid_controller_launch,
        pid_optimizer_node,
    ])
