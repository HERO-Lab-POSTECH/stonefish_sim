# Copyright 2025
# Copyright (c) 2016-2019 The UUV Simulator Authors (original code)
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
Proportional thruster model.

Adapted from UUV Simulator for use with Stonefish ROS2.
"""

import numpy as np
from .thruster import Thruster


class ThrusterProportional(Thruster):
    """
    Proportional thruster model.

    This model uses a linear relation between |command|*command and thrust force:
        thrust = gain * |command| * command

    Where command is typically angular velocity (rad/s).

    Args:
        index (int): Thruster's ID.
        gain (float): Constant proportionality gain.

    Raises:
        ValueError: If gain is not provided.
    """

    LABEL = 'proportional'

    def __init__(self, index, **kwargs):
        """Initialize proportional thruster model."""
        super().__init__(index, **kwargs)

        if 'gain' not in kwargs:
            raise ValueError('Thruster gain not provided')

        self._gain = kwargs['gain']

        if self._gain <= 0:
            raise ValueError(f'Thruster gain must be positive, got {self._gain}')

    @property
    def gain(self):
        """Get thruster gain."""
        return self._gain

    def get_command_value(self, thrust):
        """
        Compute the angular velocity necessary for the desired thrust force.

        Formula: command = sign(thrust) * sqrt(|thrust| / gain)

        Args:
            thrust (float): Thrust force magnitude in N.

        Returns:
            float: Angular velocity set-point for the thruster in rad/s.
        """
        return np.sign(thrust) * np.sqrt(np.abs(thrust) / self._gain)

    def get_thrust_value(self, command):
        """
        Compute thrust force for the given angular velocity set-point.

        Formula: thrust = gain * |command| * command

        Args:
            command (float): Angular velocity set-point for the thruster in rad/s.

        Returns:
            float: Thrust force magnitude in N.
        """
        return self._gain * np.abs(command) * command
