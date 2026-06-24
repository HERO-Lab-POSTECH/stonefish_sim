#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Common data structures and utilities for trajectory management.

This module contains shared components used across path generation and following:

Data Structures:
- Waypoint: Individual waypoint data (position, speed, heading)
- WaypointSet: Collection of waypoints with YAML loading support
- TrajectoryPoint: Point on a trajectory (position, velocity, acceleration, orientation)

Generators:
- WPTrajectoryGenerator: Generates trajectories from waypoints using various interpolation methods

Based on UUV Simulator framework.
"""

from .waypoint import Waypoint
from .waypoint_set import WaypointSet
from .trajectory_point import TrajectoryPoint
from .trajectory_generator import WPTrajectoryGenerator

__all__ = [
    'Waypoint',
    'WaypointSet',
    'TrajectoryPoint',
    'WPTrajectoryGenerator'
]
