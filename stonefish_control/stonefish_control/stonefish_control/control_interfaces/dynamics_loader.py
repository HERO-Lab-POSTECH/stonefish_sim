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
Simple Dynamics Parameter Loader for 4DOF Controllers

This is a lightweight alternative to the full Vehicle class.
Designed for 4DOF underactuated controllers that only need:
- Mass
- Yaw inertia (Izz)
- COG, COB (for reference/logging)

For 6DOF model-based controllers, use the full Vehicle class instead.

Reference:
    dynamics_params.yaml - SINGLE SOURCE OF TRUTH for vehicle dynamics
"""

from typing import Dict
import numpy as np


class DynamicsLoader:
    """
    Lightweight dynamics parameter loader for 4DOF controllers

    Loads only essential parameters from ROS parameters:
    - mass: Vehicle mass (kg)
    - inertial.izz: Yaw axis inertia (kg·m²)
    - cog: Center of gravity [x, y, z] (m)
    - cob: Center of buoyancy [x, y, z] (m)
    - volume: Buoyant volume (m³)

    Parameters must be loaded in launch file from dynamics_params.yaml

    Usage:
        dynamics = DynamicsLoader(node)
        controller = PID4DOF(mass=dynamics.mass, inertia_zz=dynamics.inertia_zz)
    """

    def __init__(self, node):
        """
        Initialize dynamics loader

        Args:
            node: ROS2 Node instance (for parameter access)

        Raises:
            ValueError: If required parameters are missing or invalid
        """
        self._node = node
        self._logger = node.get_logger()

        # ============================================================
        # Load Mass
        # ============================================================
        self._node.declare_parameter('mass', 0.0)
        self._mass = self._node.get_parameter('mass').value
        if self._mass <= 0:
            raise ValueError(f'Invalid mass: {self._mass}. Must be positive.')

        # ============================================================
        # Load Inertia (Yaw axis only for 4DOF)
        # ============================================================
        self._node.declare_parameter('inertial.ixx', 0.0)
        self._node.declare_parameter('inertial.iyy', 0.0)
        self._node.declare_parameter('inertial.izz', 0.0)
        self._node.declare_parameter('inertial.ixy', 0.0)
        self._node.declare_parameter('inertial.ixz', 0.0)
        self._node.declare_parameter('inertial.iyz', 0.0)

        self._inertial = {
            'ixx': self._node.get_parameter('inertial.ixx').value,
            'iyy': self._node.get_parameter('inertial.iyy').value,
            'izz': self._node.get_parameter('inertial.izz').value,
            'ixy': self._node.get_parameter('inertial.ixy').value,
            'ixz': self._node.get_parameter('inertial.ixz').value,
            'iyz': self._node.get_parameter('inertial.iyz').value,
        }

        if self._inertial['izz'] <= 0:
            raise ValueError(f'Invalid yaw inertia (izz): {self._inertial["izz"]}. Must be positive.')

        # ============================================================
        # Load COG, COB (for reference and logging)
        # ============================================================
        self._node.declare_parameter('cog', [0.0, 0.0, 0.0])
        self._cog = np.array(self._node.get_parameter('cog').value)
        if len(self._cog) != 3:
            raise ValueError(f'Invalid COG: {self._cog}. Must be [x, y, z].')

        self._node.declare_parameter('cob', [0.0, 0.0, 0.0])
        self._cob = np.array(self._node.get_parameter('cob').value)
        if len(self._cob) != 3:
            raise ValueError(f'Invalid COB: {self._cob}. Must be [x, y, z].')

        # ============================================================
        # Load Volume (for buoyancy calculations if needed)
        # ============================================================
        self._node.declare_parameter('volume', 0.0)
        self._volume = self._node.get_parameter('volume').value
        if self._volume <= 0:
            raise ValueError(f'Invalid volume: {self._volume}. Must be positive.')

        # ============================================================
        # Load Fluid Properties (optional)
        # ============================================================
        self._node.declare_parameter('density', 1028.0)
        self._density = self._node.get_parameter('density').value

        self._node.declare_parameter('gravity', 9.82)
        self._gravity = self._node.get_parameter('gravity').value

        # ============================================================
        # Calculate Passive Stability (for verification)
        # ============================================================
        self._metacentric_height = self._cob[2] - self._cog[2]
        self._buoyancy = self._density * self._gravity * self._volume
        self._weight = self._mass * self._gravity
        self._restoring_coefficient = (self._buoyancy - self._weight) * self._metacentric_height

        # ============================================================
        # Log Summary
        # ============================================================
        self._logger.info('=' * 60)
        self._logger.info('Dynamics Loaded (4DOF)')
        self._logger.info('=' * 60)
        self._logger.info(f'Mass: {self._mass:.2f} kg')
        self._logger.info(f'Inertia (Yaw): {self._inertial["izz"]:.4f} kg·m²')
        self._logger.info(f'COG: [{self._cog[0]:.4f}, {self._cog[1]:.4f}, {self._cog[2]:.4f}] m')
        self._logger.info(f'COB: [{self._cob[0]:.4f}, {self._cob[1]:.4f}, {self._cob[2]:.4f}] m')
        self._logger.info(f'Volume: {self._volume:.6f} m³')
        self._logger.info('-' * 60)
        self._logger.info(f'Metacentric Height (GM): {self._metacentric_height:.4f} m')
        self._logger.info(f'Buoyancy: {self._buoyancy:.2f} N')
        self._logger.info(f'Weight: {self._weight:.2f} N')
        self._logger.info(f'Net Force: {self._buoyancy - self._weight:.2f} N')
        self._logger.info(f'Restoring Coeff: {self._restoring_coefficient:.4f} Nm/rad')

        # Passive stability check
        if abs(self._metacentric_height) >= 0.15:
            self._logger.info('✓ PASSIVE STABILITY OK (|GM| >= 0.15m)')
        else:
            self._logger.warn(f'⚠ PASSIVE STABILITY WEAK (|GM| = {abs(self._metacentric_height):.3f}m < 0.15m)')
        self._logger.info('=' * 60)

    # ============================================================
    # Properties (read-only)
    # ============================================================
    @property
    def mass(self) -> float:
        """Vehicle mass (kg)"""
        return self._mass

    @property
    def inertial(self) -> Dict[str, float]:
        """Inertia tensor components (kg·m²)"""
        return self._inertial

    @property
    def inertia_zz(self) -> float:
        """Yaw axis inertia (kg·m²)"""
        return self._inertial['izz']

    @property
    def cog(self) -> np.ndarray:
        """Center of gravity [x, y, z] (m)"""
        return self._cog

    @property
    def cob(self) -> np.ndarray:
        """Center of buoyancy [x, y, z] (m)"""
        return self._cob

    @property
    def volume(self) -> float:
        """Buoyant volume (m³)"""
        return self._volume

    @property
    def metacentric_height(self) -> float:
        """Metacentric height GM = COB_z - COG_z (m)"""
        return self._metacentric_height

    @property
    def buoyancy(self) -> float:
        """Buoyancy force (N)"""
        return self._buoyancy

    @property
    def weight(self) -> float:
        """Weight (N)"""
        return self._weight

    def get_summary(self) -> Dict:
        """Get dynamics summary as dictionary"""
        return {
            'mass': self._mass,
            'inertia': self._inertial,
            'cog': self._cog.tolist(),
            'cob': self._cob.tolist(),
            'volume': self._volume,
            'gm': self._metacentric_height,
            'buoyancy': self._buoyancy,
            'weight': self._weight,
        }
