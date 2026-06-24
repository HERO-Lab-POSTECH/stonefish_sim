#!/usr/bin/env python3
# Copyright 2025
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Parameterized vehicle simulation bringup.
#
# Composes the Stonefish simulator with a vehicle scenario plus an optional
# thruster manager, and an optional base_link->base_link_frd static TF (needed
# by the BlueROV2). This factors out the block that blueboat.launch.py and
# bluerov2.launch.py used to duplicate verbatim; those files are now thin
# wrappers that forward per-vehicle defaults to this file.
#
# Usage (normally invoked via blueboat.launch.py / bluerov2.launch.py):
#   ros2 launch stonefish_ros2 vehicle.launch.py \
#       vehicle_name:=bluerov2 scenario:=bluerov2_infrastructure \
#       window_res_y:=1080 enable_base_link_frd:=true

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for a parameterized vehicle simulation."""

    vehicle_name = LaunchConfiguration('vehicle_name')
    scenario = LaunchConfiguration('scenario')

    args = [
        DeclareLaunchArgument(
            'vehicle_name', default_value='bluerov2',
            description='Vehicle name; used for the FRD TF frame ids and forwarded '
                        'to the thruster manager.'),
        DeclareLaunchArgument(
            'scenario', default_value='bluerov2_infrastructure',
            description='Scenario name without the .scn extension; resolved under '
                        'stonefish_description/scenarios/.'),
        DeclareLaunchArgument(
            'start_thruster_manager', default_value='true',
            description='Start the thruster manager for this vehicle.'),
        DeclareLaunchArgument(
            'simulation_rate', default_value='100.0',
            description='Physics step rate in Hz.'),
        DeclareLaunchArgument(
            'gpu', default_value='true',
            description='true = rendered GPU simulator; false = headless nogpu build.'),
        DeclareLaunchArgument(
            'window_res_y', default_value='1056',
            description='Render window height in px (per-vehicle; BlueROV2 uses 1080).'),
        DeclareLaunchArgument(
            'enable_base_link_frd', default_value='false',
            description='Publish the base_link->base_link_frd static TF (BlueROV2 needs it).'),
    ]

    # 1. Stonefish simulator (gpu/nogpu selected inside simulator.launch.py).
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('stonefish_ros2'), 'launch', 'simulator.launch.py'
            ])
        ),
        launch_arguments={
            # The trailing '' yields the stonefish_description share dir as the
            # simulator's asset root; this is the C++ exe's path contract — keep it.
            'simulation_data': PathJoinSubstitution([
                FindPackageShare('stonefish_description'), ''
            ]),
            'scenario_desc': PathJoinSubstitution([
                FindPackageShare('stonefish_description'),
                'scenarios',
                [scenario, '.scn']
            ]),
            'simulation_rate': LaunchConfiguration('simulation_rate'),
            'window_res_x': '960',
            'window_res_y': LaunchConfiguration('window_res_y'),
            'rendering_quality': 'high',
            'gpu': LaunchConfiguration('gpu'),
        }.items(),
    )

    # 2. Thruster manager (conditional).
    thruster_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('stonefish_thruster_manager'),
                'launch', 'thruster_manager.launch.py'
            ])
        ),
        launch_arguments={'vehicle_name': vehicle_name}.items(),
        condition=IfCondition(LaunchConfiguration('start_thruster_manager')),
    )

    # 3. Static TF: base_link -> base_link_frd (FRD coordinate frame), conditional.
    #
    # Problem: Stonefish publishes base_link with wrong orientation:
    #   Current: X=down, Y=back, Z=left
    #   Desired (FRD): X=forward, Y=right, Z=down
    #
    # Rotation matrix calculation:
    #   X_frd = -Y_base (forward = opposite of back)
    #   Y_frd = -Z_base (right = opposite of left)
    #   Z_frd = X_base (down = same as down)
    #
    # Rotation matrix: R = [[ 0, -1,  0],
    #                       [ 0,  0, -1],
    #                       [ 1,  0,  0]]
    #
    # Correct quaternion (verified via scipy):
    #   qx=0.5, qy=-0.5, qz=0.5, qw=-0.5
    #
    # Rotation matrix (column-wise interpretation):
    #   Col 0 (child X): [0, -1, 0] = -parent Y (forward) ✓
    #   Col 1 (child Y): [0, 0, -1] = -parent Z (right) ✓
    #   Col 2 (child Z): [1, 0, 0] = parent X (down) ✓
    base_link_frd_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_frd_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0.5', '--qy', '-0.5', '--qz', '0.5', '--qw', '-0.5',
            '--frame-id', [vehicle_name, '/base_link'],
            '--child-frame-id', [vehicle_name, '/base_link_frd']
        ],
        condition=IfCondition(LaunchConfiguration('enable_base_link_frd')),
    )

    return LaunchDescription(args + [simulator, thruster_manager, base_link_frd_publisher])
