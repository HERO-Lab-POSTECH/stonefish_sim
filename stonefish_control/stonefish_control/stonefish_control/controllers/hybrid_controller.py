#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Stonefish Control Contributors
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Hybrid Controller - switches between velocity and position modes"""

import numpy as np
from typing import Tuple, Optional
from .position_controller import PositionController


class HybridController:
    """Hybrid controller with velocity/position mode switching"""

    def __init__(
        self,
        Kp_vel: np.ndarray,
        Kd_vel: np.ndarray,
        Ki_vel: np.ndarray,
        Kb_vel: np.ndarray,
        Kp_pos: np.ndarray,
        Kd_pos: np.ndarray,
        Ki_pos: np.ndarray,
        Kb_pos: np.ndarray,
        mass: float,
        inertia_zz: float,
        max_force_vel: float = 800.0,
        max_torque_vel: float = 160.0,
        max_force_pos: float = 200.0,
        max_torque_pos: float = 50.0,
        integral_safety_factor_vel: float = 0.5,
        integral_safety_factor_pos: float = 2.0,
        initial_mode: str = 'velocity'
    ):
        self.velocity_controller = PositionController(
            Kp=Kp_vel, Kd=Kd_vel, Ki=Ki_vel, Kb=Kb_vel,
            mass=mass, inertia_zz=inertia_zz,
            max_force=max_force_vel, max_torque=max_torque_vel,
            integral_safety_factor=integral_safety_factor_vel,
            control_mode='velocity'
        )

        self.position_controller = PositionController(
            Kp=Kp_pos, Kd=Kd_pos, Ki=Ki_pos, Kb=Kb_pos,
            mass=mass, inertia_zz=inertia_zz,
            max_force=max_force_pos, max_torque=max_torque_pos,
            integral_safety_factor=integral_safety_factor_pos,
            control_mode="position"
        )

        self.control_mode = initial_mode
        self.mode_switch_count = 0

    def set_mode(self, mode: str):
        if mode not in ['velocity', 'position']:
            raise ValueError(f"Invalid mode: {mode}")

        if mode != self.control_mode:
            if mode == 'velocity':
                self.velocity_controller.reset()
            else:
                self.position_controller.reset()

            self.control_mode = mode
            self.mode_switch_count += 1

    def compute_control(
        self,
        pose_des: np.ndarray,
        pose_curr: np.ndarray,
        vel_curr: np.ndarray,
        dt: float,
        vel_des: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, dict]:
        if self.control_mode == 'velocity':
            tau, info = self.velocity_controller.compute_control(
                pose_des, pose_curr, vel_curr, dt, vel_des
            )
        else:
            tau, info = self.position_controller.compute_control(
                pose_des, pose_curr, vel_curr, dt, vel_des
            )

        info['active_mode'] = self.control_mode
        info['mode_switches'] = self.mode_switch_count

        return tau, info

    def reset(self):
        self.velocity_controller.reset()
        self.position_controller.reset()
        self.mode_switch_count = 0

    def get_status(self) -> dict:
        return {
            'mode': self.control_mode,
            'switches': self.mode_switch_count
        }
