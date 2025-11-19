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

"""Launch file for thruster allocator node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description."""
    # Declare launch arguments
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='bluerov2',
        description='Name of the vehicle'
    )

    tam_file_arg = DeclareLaunchArgument(
        'tam_file',
        default_value='',
        description='Path to TAM YAML file (empty = use default based on vehicle_name)'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='setpoint/pwm',
        description='Output topic for thruster commands (relative to namespace)'
    )

    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='50.0',
        description='Update rate in Hz'
    )

    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='1.0',
        description='Timeout for zeroing thrusters (0 = disabled)'
    )

    max_thrust_arg = DeclareLaunchArgument(
        'max_thrust',
        default_value='100.0',
        description='Maximum thrust force per thruster [N]'
    )

    # Create node
    thruster_allocator_node = Node(
        package='stonefish_thruster_manager',
        executable='thruster_allocator',
        name='thruster_allocator',
        namespace=LaunchConfiguration('vehicle_name'),
        output='screen',
        parameters=[{
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'tam_file': LaunchConfiguration('tam_file'),
            'output_topic': LaunchConfiguration('output_topic'),
            'update_rate': LaunchConfiguration('update_rate'),
            'timeout': LaunchConfiguration('timeout'),
            'max_thrust': LaunchConfiguration('max_thrust'),
        }],
        remappings=[
            ('~/input', 'thruster_manager/input'),
            ('~/input_stamped', 'thruster_manager/input_stamped'),
        ]
    )

    return LaunchDescription([
        vehicle_name_arg,
        tam_file_arg,
        output_topic_arg,
        update_rate_arg,
        timeout_arg,
        max_thrust_arg,
        thruster_allocator_node,
    ])
