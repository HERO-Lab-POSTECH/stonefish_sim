# SPDX-FileCopyrightText: 2016-2019 The UUV Simulator Authors
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Control interfaces for trajectory tracking and dynamic positioning."""

# 4DOF lightweight dynamics loader (for underactuated vehicles).
# LIVE: imported by hybrid_controller_node / position_controller_node.
# rclpy/nav_msgs-free leaf, so eager import is safe.
from .dynamics_loader import DynamicsLoader

__all__ = [
    'DynamicsLoader',
]
