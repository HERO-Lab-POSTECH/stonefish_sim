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
Path generation and interpolation algorithms.

This module provides various path interpolation methods for marine vehicle navigation.

Supported Interpolation Methods:
- LinearInterpolator ('linear'): Piecewise linear segments, sharp corners
- LIPBInterpolator ('lipb'): Log-Interpolated Polynomial Bezier, smooth corners (Recommended for 4DOF)
- CSInterpolator ('cubic'): Cubic spline interpolation, fully smooth curves

Helper Classes:
- PathGenerator: Base class for all interpolators
- LineSegment: Line segment representation (used internally)
- BezierCurve: Bezier curve utilities (used by LIPB and cubic spline)

Coordinate System: NED (North-East-Down)
Frame ID: world_ned
"""

from .path_generator import PathGenerator
from .line_segment import LineSegment
from .bezier_curve import BezierCurve
from .linear_interpolator import LinearInterpolator
from .cs_interpolator import CSInterpolator
from .lipb_interpolator import LIPBInterpolator

__all__ = [
    'PathGenerator',
    'LineSegment',
    'BezierCurve',
    'LinearInterpolator',
    'CSInterpolator',
    'LIPBInterpolator',
]
