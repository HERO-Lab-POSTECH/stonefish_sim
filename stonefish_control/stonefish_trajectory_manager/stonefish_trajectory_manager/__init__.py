# SPDX-FileCopyrightText: 2016-2019 The UUV Simulator Authors
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""
Stonefish Trajectory Manager

Provides trajectory generation and management for underwater vehicles.

Modules:
- common: Common data structures and trajectory generation utilities
- nodes: ROS2 nodes for path generation and following
- path_generator: Path interpolation methods (linear, cubic, lipb, etc.)
- path_following: Path following algorithms (Simple LOS guidance)
"""

# Import common components
from .common import (
    Waypoint,
    WaypointSet,
    TrajectoryPoint,
    WPTrajectoryGenerator
)

# Import path generator
from .path_generator import PathGenerator

__all__ = [
    'Waypoint',
    'WaypointSet',
    'TrajectoryPoint',
    'WPTrajectoryGenerator',
    'PathGenerator'
]

__version__ = '0.4.0'
