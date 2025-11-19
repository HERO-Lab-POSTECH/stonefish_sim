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
Thruster models for converting thrust forces to thruster commands.

Adapted from UUV Simulator for use with Stonefish ROS2.
"""

import numpy as np


class Thruster:
    """
    Abstract base class for all thruster models.

    Thruster models convert between thrust force (N) and thruster commands
    (e.g., angular velocity in rad/s or PWM values).

    The instance of a thruster model must use the factory method.

    Args:
        index (int): Thruster's ID.
        **kwargs: Model-specific parameters.
    """

    LABEL = ''

    def __init__(self, index, **kwargs):
        """Initialize thruster model."""
        self._index = index
        self._command = 0.0
        self._thrust = 0.0

    @property
    def index(self):
        """Get thruster index."""
        return self._index

    @staticmethod
    def create_thruster(model_name, *args, **kwargs):
        """
        Factory method for thruster models.

        Args:
            model_name (str): Name identifier of the thruster model.
            *args: Positional arguments for the model.
            **kwargs: Keyword arguments for the model.

        Returns:
            Thruster: Thruster model instance.

        Raises:
            ValueError: If the model name is invalid.
        """
        for thruster_class in Thruster.__subclasses__():
            if model_name == thruster_class.LABEL:
                return thruster_class(*args, **kwargs)
        raise ValueError(f'Invalid thruster model: {model_name}')

    def get_command_value(self, thrust):
        """
        Convert desired thrust force to input command.

        Override this method to implement custom models.

        Args:
            thrust (float): Thrust force in N.

        Returns:
            float: Command value (e.g., angular velocity in rad/s).
        """
        raise NotImplementedError()

    def get_thrust_value(self, command):
        """
        Compute thrust force for the given command.

        Args:
            command (float): Command value (e.g., angular velocity in rad/s).

        Returns:
            float: Thrust force in N.
        """
        raise NotImplementedError()

    def get_curve(self, min_value, max_value, n_points):
        """
        Sample the conversion curve and return the values.

        Args:
            min_value (float): Minimum command value.
            max_value (float): Maximum command value.
            n_points (int): Number of sample points.

        Returns:
            tuple: (input_values, output_values) as lists.
        """
        if min_value >= max_value or n_points <= 0:
            return [], []

        input_values = np.linspace(min_value, max_value, n_points)
        output_values = [self.get_thrust_value(value) for value in input_values]

        return input_values.tolist(), output_values

    def update(self, thrust):
        """
        Update thruster state with desired thrust.

        Args:
            thrust (float): Desired thrust force in N.

        Returns:
            float: Calculated command value.
        """
        self._thrust = thrust
        self._command = self.get_command_value(thrust)
        return self._command
