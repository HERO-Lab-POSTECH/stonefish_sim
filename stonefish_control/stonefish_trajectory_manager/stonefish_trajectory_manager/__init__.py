# Copyright (c) 2016-2019 The UUV Simulator Authors.
# All rights reserved.
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

__version__ = '0.3.0'
