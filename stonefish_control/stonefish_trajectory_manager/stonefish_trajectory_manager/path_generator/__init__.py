# SPDX-FileCopyrightText: 2016-2019 The UUV Simulator Authors
#
# SPDX-License-Identifier: GPL-3.0-or-later
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
