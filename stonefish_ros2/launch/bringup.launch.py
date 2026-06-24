#!/usr/bin/env python3
# Copyright 2025
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Top-level simulation bringup for stonefish_sim.
#
# Thin orchestration layer (Nav2 bringup pattern): declares the commonly-changed
# arguments and includes the subsystem launch files. It launches NO nodes of its
# own — every node is owned by a component launch file that stays runnable
# standalone. Use this to bring the simulator + control stack up together with a
# single, consistent set of arguments (namespace, use_sim_time, vehicle).
#
# Usage:
#   ros2 launch stonefish_ros2 bringup.launch.py vehicle:=bluerov2
#   ros2 launch stonefish_ros2 bringup.launch.py vehicle:=bluerov2 use_sim_time:=true
#   ros2 launch stonefish_ros2 bringup.launch.py start_control:=false   # sim only

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the top-level simulation bringup description."""

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    vehicle = LaunchConfiguration('vehicle')

    args = [
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level ROS namespace for the whole stack (empty for single-AUV).'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulator /clock as the time source. Default true here '
                        'because this bringup always runs the simulator; set false only '
                        'for unusual offline setups.'),
        DeclareLaunchArgument(
            'vehicle', default_value='bluerov2',
            description='Vehicle to bring up: bluerov2 | blueboat.'),
        DeclareLaunchArgument(
            'start_control', default_value='true',
            description='Start the control stack (control.launch.py — the controller).'),
        DeclareLaunchArgument(
            'start_path', default_value='true',
            description='Start the path stack (path.launch.py — generator + following).'),
        DeclareLaunchArgument(
            'start_thruster_manager', default_value='true',
            description='Start the thruster manager (forwarded to the vehicle launch).'),
    ]

    # The vehicle simulation (simulator + optional thruster manager + FRD TF).
    # Selected by name so vehicle:=blueboat picks blueboat.launch.py.
    vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('stonefish_ros2'), 'launch', [vehicle, '.launch.py']
            ])
        ),
        launch_arguments={
            'start_thruster_manager': LaunchConfiguration('start_thruster_manager'),
        }.items(),
    )

    # The control stack (the hybrid controller). The vehicle launch above already
    # runs the simulator, so control.launch.py starts no simulator of its own.
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('stonefish_control'), 'launch', 'control.launch.py'
            ])
        ),
        launch_arguments={
            'vehicle_name': vehicle,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_control')),
    )

    # The path stack (path generation + following + world_ned->map TF).
    path_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('stonefish_trajectory_manager'), 'launch', 'path.launch.py'
            ])
        ),
        launch_arguments={
            'vehicle_name': vehicle,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_path')),
    )

    bringup = GroupAction([
        PushRosNamespace(namespace),
        SetParameter('use_sim_time', use_sim_time),
        # Keep /tf and /tf_static global when a namespace is pushed (standard
        # single-tree convention); harmless when namespace is empty.
        SetRemap('/tf', 'tf'),
        SetRemap('/tf_static', 'tf_static'),
        vehicle_launch,
        control_launch,
        path_launch,
    ])

    return LaunchDescription(args + [bringup])
