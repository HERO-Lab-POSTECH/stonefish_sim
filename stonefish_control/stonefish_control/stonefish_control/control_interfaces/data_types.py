#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Stonefish Control Contributors
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Data Types for Unified 6DOF Controller

Provides angle-wrapping helper for pure Python control logic.

Reference:
- Fossen (2011) "Handbook of Marine Craft Hydrodynamics and Motion Control"
"""

import numpy as np


def angle_wrap(angle: float) -> float:
    """Wrap angle to [-pi, pi].

    Args:
        angle: Angle in radians

    Returns:
        Wrapped angle in [-pi, pi]
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi
