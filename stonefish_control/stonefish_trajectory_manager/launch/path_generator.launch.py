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
Launch file for path generator node.

This launch file starts the path generator node which:
1. Loads waypoints from a YAML file
2. Generates path using specified interpolation method
3. Publishes path as nav_msgs/Path topic
4. Publishes visualization markers for RViz

Usage:
    ros2 launch stonefish_trajectory_manager path_generator.launch.py \
        waypoint_file:=/path/to/waypoints.yaml \
        interpolation_method:=lipb

Parameters:
    - waypoint_file: Path to YAML waypoint file (required)
    - interpolation_method: 'linear', 'lipb', 'cubic' (default: 'lipb')
    - publish_rate: Rate to publish markers in Hz (default: 1.0)
    - sample_step: Path resolution (default: 0.01)

Note: Always generates 4DOF paths (X,Y,Z,Yaw) for BlueROV2.
      Orientation is for visualization only.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory for portable path resolution
    pkg_share = get_package_share_directory('stonefish_trajectory_manager')
    default_waypoint_file = os.path.join(pkg_share, 'config', 'examples', 'krit_lawnmower.yaml')

    # Declare launch arguments
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=default_waypoint_file,
        description='Path to YAML waypoint file (required)'
    )

    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='bluerov2',
        description='Vehicle namespace for odometry subscription'
    )

    interpolation_method_arg = DeclareLaunchArgument(
        'interpolation_method',
        default_value='lipb',
        description='Interpolation method: linear, lipb, cubic, or cs'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Rate to publish markers in Hz'
    )

    sample_step_arg = DeclareLaunchArgument(
        'sample_step',
        default_value='0.01',
        description='Parametric step size for path sampling'
    )

    distance_threshold_arg = DeclareLaunchArgument(
        'initial_waypoint_distance_threshold',
        default_value='0.5',
        description='Distance threshold to add robot position as WP0 (meters)'
    )

    # Static TF: world_ned -> map
    # This connects Stonefish's world_ned frame to the standard map frame
    # used by navigation and localization nodes
    tf_world_ned_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_ned_to_map_broadcaster',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'world_ned',
            '--child-frame-id', 'map'
        ],
        output='screen'
    )

    # Path generator node
    path_generator_node = Node(
        package='stonefish_trajectory_manager',
        executable='path_generator_node',
        name='path_generator_node',
        output='screen',
        parameters=[{
            'waypoint_file': LaunchConfiguration('waypoint_file'),
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'interpolation_method': LaunchConfiguration('interpolation_method'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'sample_step': LaunchConfiguration('sample_step'),
            'initial_waypoint_distance_threshold': LaunchConfiguration('initial_waypoint_distance_threshold'),
        }]
    )

    return LaunchDescription([
        waypoint_file_arg,
        vehicle_name_arg,
        interpolation_method_arg,
        publish_rate_arg,
        sample_step_arg,
        distance_threshold_arg,
        tf_world_ned_to_map,
        path_generator_node
    ])
