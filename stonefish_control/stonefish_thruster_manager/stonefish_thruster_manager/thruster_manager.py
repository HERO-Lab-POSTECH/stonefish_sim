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


def force_to_pwm(forces, max_thrust):
    """추력 명령[N] → Stonefish 추진기 setpoint(rpm 분율, -1~1) 역추력맵.

    Stonefish `Thruster`는 setpoint를 최대 회전수 분율 n/n_max로 해석하고
    정적 추력은 T = ρ·kT·n|n|·D⁴ ∝ n² 이다. 따라서 힘 F를 얻으려면
    pwm = sign(F)·√(|F|/T_max) 가 필요하다 — 종전의 선형 나눗셈(F/scale)은
    실제 추력을 제곱으로 왜곡했다(예: scale=100일 때 100 N 명령 → 실추력 7.3 N).

    Args:
        forces: 추진기별 추력 명령 [N] (array-like).
        max_thrust: 추진기당 물리 최대 추력 T_max [N]. |F| > T_max는 ±1로 클립.

    Returns:
        np.ndarray: setpoint ∈ [-1, 1].
    """
    forces = np.asarray(forces, dtype=float)
    ratio = np.minimum(np.abs(forces) / max_thrust, 1.0)
    return np.sign(forces) * np.sqrt(ratio)


def scale_thrust_to_limit(forces, max_thrust):
    """다축 동시명령이 추진기당 한계를 넘으면 방향보존 균등 스케일링.

    element-wise 클립은 초과한 추진기만 잘라 wrench 방향(선회 기하)을
    왜곡한다 — 예: surge 55 N + yaw 8 N·m 동시 명령은 추진기당 31.5 N
    (T_max의 153%)을 요구하는데, 클립하면 yaw가 4.4 N·m로 붕괴한다.
    균등 스케일링은 크기만 줄이고 wrench 방향을 보존한다.

    Returns:
        (scaled_forces, factor): factor < 1.0 이면 스케일링이 발동한 것.
    """
    forces = np.asarray(forces, dtype=float)
    peak = np.max(np.abs(forces)) if forces.size else 0.0
    if peak <= max_thrust:
        return forces, 1.0
    factor = max_thrust / peak
    return forces * factor, factor
