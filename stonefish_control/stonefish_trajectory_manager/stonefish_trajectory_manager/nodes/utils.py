#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Shared utilities for trajectory nodes.

Reduces code duplication between trajectory_publisher and trajectory_follower.
"""

from typing import Dict, Optional
import numpy as np


def declare_trajectory_parameters(node, param_config: Optional[Dict] = None) -> Dict:
    """Declare common trajectory parameters.

    Args:
        node: ROS2 Node instance
        param_config: Optional dict with custom default values

    Returns:
        Dict with parameter values
    """
    defaults = param_config or {}

    # Common parameters
    node.declare_parameter('waypoint_file', defaults.get('waypoint_file', ''))
    node.declare_parameter('vehicle_name', defaults.get('vehicle_name', 'bluerov2'))
    node.declare_parameter('interpolation_method', defaults.get('interpolation_method', 'lipb'))
    node.declare_parameter('update_rate', defaults.get('update_rate', 50.0))

    # Return values
    return {
        'waypoint_file': node.get_parameter('waypoint_file').value,
        'vehicle_name': node.get_parameter('vehicle_name').value,
        'interpolation_method': node.get_parameter('interpolation_method').value,
        'update_rate': node.get_parameter('update_rate').value
    }


def load_waypoints(logger, filename: str, use_clock=None):
    """Load waypoints from YAML file with validation.

    Args:
        logger: ROS2 logger instance
        filename: Path to waypoint YAML file
        use_clock: Optional ROS2 clock for timestamps

    Returns:
        WaypointSet object

    Raises:
        RuntimeError: If file loading fails
    """
    from ..common import WaypointSet

    wp_set = WaypointSet(clock=use_clock) if use_clock else WaypointSet()

    if not wp_set.read_from_file(filename):
        logger.error(f'Failed to load waypoint file: {filename}')
        raise RuntimeError('Failed to load waypoint file')

    # Calculate total distance
    total_distance = 0.0
    for i in range(wp_set.num_waypoints - 1):
        wp1 = wp_set.get_waypoint(i)
        wp2 = wp_set.get_waypoint(i + 1)
        total_distance += np.linalg.norm(wp2.pos - wp1.pos)

    logger.info(f'✓ Loaded {wp_set.num_waypoints} waypoints ({total_distance:.2f}m)')
    return wp_set


