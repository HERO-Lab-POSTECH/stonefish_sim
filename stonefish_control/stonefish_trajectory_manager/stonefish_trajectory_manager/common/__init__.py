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
