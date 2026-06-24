# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Thruster Allocation Matrix (TAM) manager.

Provides utilities for loading and managing TAM from YAML files.
"""

import numpy as np
import yaml
from pathlib import Path


class ThrusterManager:
    """
    Manager for Thruster Allocation Matrix (TAM).

    Handles loading TAM from YAML configuration files and provides
    utilities for thruster allocation calculations.
    """

    def __init__(self, tam_file_path=None, tam_matrix=None):
        """
        Initialize ThrusterManager.

        Args:
            tam_file_path (str|Path, optional): Path to TAM YAML file.
            tam_matrix (np.ndarray, optional): TAM matrix (6xN).

        Raises:
            ValueError: If neither tam_file_path nor tam_matrix is provided.
        """
        self._tam = None
        self._tam_pinv = None
        self._n_thrusters = 0

        if tam_file_path is not None:
            self.load_tam_from_file(tam_file_path)
        elif tam_matrix is not None:
            self.set_tam(tam_matrix)
        else:
            raise ValueError('Either tam_file_path or tam_matrix must be provided')

    @property
    def tam(self):
        """Get TAM matrix."""
        return self._tam

    @property
    def tam_pseudo_inverse(self):
        """Get pseudo-inverse of TAM matrix."""
        return self._tam_pinv

    @property
    def n_thrusters(self):
        """Get number of thrusters."""
        return self._n_thrusters

    def load_tam_from_file(self, file_path):
        """
        Load TAM from YAML file.

        Expected YAML format:
            tam:
              - [row 0 values...]  # X (Surge)
              - [row 1 values...]  # Y (Sway)
              - [row 2 values...]  # Z (Heave)
              - [row 3 values...]  # Roll
              - [row 4 values...]  # Pitch
              - [row 5 values...]  # Yaw

        Args:
            file_path (str|Path): Path to TAM YAML file.

        Raises:
            FileNotFoundError: If file doesn't exist.
            ValueError: If TAM format is invalid.
        """
        file_path = Path(file_path)

        if not file_path.exists():
            raise FileNotFoundError(f'TAM file not found: {file_path}')

        with open(file_path, 'r') as f:
            config = yaml.safe_load(f)

        if 'tam' not in config:
            raise ValueError('TAM YAML file must contain "tam" key')

        tam_list = config['tam']

        if not isinstance(tam_list, list):
            raise ValueError('TAM must be a list of lists')

        if len(tam_list) != 6:
            raise ValueError(f'TAM must have 6 rows (6DOF), got {len(tam_list)}')

        tam_matrix = np.array(tam_list, dtype=float)

        self.set_tam(tam_matrix)

    def set_tam(self, tam_matrix):
        """
        Set TAM matrix and compute its pseudo-inverse.

        Args:
            tam_matrix (np.ndarray): TAM matrix with shape (6, n_thrusters).

        Raises:
            ValueError: If TAM shape is invalid.
        """
        if tam_matrix.shape[0] != 6:
            raise ValueError(f'TAM must have 6 rows, got {tam_matrix.shape[0]}')

        self._tam = np.array(tam_matrix, dtype=float)
        self._n_thrusters = tam_matrix.shape[1]

        # Compute pseudo-inverse: pinv(TAM) such that thrust = pinv(TAM) @ wrench
        self._tam_pinv = np.linalg.pinv(self._tam)

    def compute_thrust_forces(self, wrench):
        """
        Compute thruster forces from 6DOF wrench using pseudo-inverse.

        Formula: thrust = pinv(TAM) @ wrench

        Args:
            wrench (np.ndarray): 6DOF wrench [Fx, Fy, Fz, Tx, Ty, Tz].

        Returns:
            np.ndarray: Thruster forces with shape (n_thrusters,).

        Raises:
            ValueError: If wrench has invalid shape.
        """
        wrench = np.array(wrench, dtype=float)

        if wrench.shape != (6,):
            raise ValueError(f'Wrench must have shape (6,), got {wrench.shape}')

        return self._tam_pinv @ wrench

    def compute_wrench(self, thrust_forces):
        """
        Compute 6DOF wrench from thruster forces.

        Formula: wrench = TAM @ thrust

        Args:
            thrust_forces (np.ndarray): Thruster forces with shape (n_thrusters,).

        Returns:
            np.ndarray: 6DOF wrench [Fx, Fy, Fz, Tx, Ty, Tz].

        Raises:
            ValueError: If thrust_forces has invalid shape.
        """
        thrust_forces = np.array(thrust_forces, dtype=float)

        if thrust_forces.shape != (self._n_thrusters,):
            raise ValueError(
                f'Thrust forces must have shape ({self._n_thrusters},), '
                f'got {thrust_forces.shape}'
            )

        return self._tam @ thrust_forces
