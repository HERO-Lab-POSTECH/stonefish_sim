#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""Cascade Controller — outer 위치-P → inner 속도-PI 2단 보상기 (Fossen).

Architecture:
    Position Error → (outer P) → Velocity Setpoint → (inner PI) → Force/Torque

핵심 설계(설계 SSOT §1~§3):
    - outer: 순수 비례(P-only). 적분은 inner 한 곳만(cascade integrator windup 차단).
    - inner: 속도오차 PI(+선택 D) + back-calculation anti-windup(F1과 동일 메커니즘).
    - vel_ff: ILOS path-tangent feedforward 속도 [u, 0, w_d, r]. sway(인덱스1)=0.
    - v_sp clamp는 ff 합산 후 적용(inner 포화 사전차단).

Frame:
    World: NED, Body: FRD. 단위 SI.
"""
import numpy as np
from typing import Tuple, Optional
from scipy.spatial.transform import Rotation

from ..control_interfaces.data_types import angle_wrap


class CascadeController:
    """위치→속도→힘 2단 cascade 보상기 (fully-actuated UUV용)."""

    def __init__(
        self,
        Kp_outer: np.ndarray,
        Kp_inner: np.ndarray,
        Ki_inner: np.ndarray,
        Kb_inner: np.ndarray,
        Kd_inner: np.ndarray,
        mass: float,
        inertia_zz: float,
        v_sp_limit: np.ndarray,
        max_force: float = 800.0,
        max_torque: float = 160.0,
        integral_safety_factor: float = 0.5,
    ):
        """
        Args:
            Kp_outer: [4] 위치 P 게인 (x,y,z,yaw) → 속도 setpoint
            Kp_inner, Ki_inner, Kd_inner, Kb_inner: [4] inner 속도 PI(+D)+back-calc
            mass, inertia_zz: 시그니처 동형성용 (현 설계에서 inner는 M·a ff 미사용)
            v_sp_limit: [4] 속도 setpoint clamp [u,v,w,r]
            max_force, max_torque: 힘/토크 포화 한계
            integral_safety_factor: inner 적분 한계 자동계산 배율
        """
        self.Kp_outer = np.asarray(Kp_outer, dtype=float)
        self.Kp_inner = np.asarray(Kp_inner, dtype=float)
        self.Ki_inner = np.asarray(Ki_inner, dtype=float)
        self.Kd_inner = np.asarray(Kd_inner, dtype=float)
        self.Kb_inner = np.asarray(Kb_inner, dtype=float)

        # M·a feedforward는 P4 이월 — 시그니처 동형성용으로만 보관
        self.M = np.diag([mass, mass, mass, inertia_zz])

        self.v_sp_limit = np.asarray(v_sp_limit, dtype=float)
        self.max_force = max_force
        self.max_torque = max_torque
        self.sat_limit = np.array([max_force, max_force, max_force, max_torque])

        # inner 적분 한계 (F1 position_controller.py:115-116 동일 공식)
        Ki_diag = self.Ki_inner + 1e-6
        self.integral_limit = self.sat_limit / Ki_diag * integral_safety_factor

        # 상태
        self.integral_inner = np.zeros(4)
        self.prev_e_inner = np.zeros(4)
        self.saturated_count = 0

    def reset(self):
        """inner 적분·이전오차 초기화 (모드 진입 시 bumpless)."""
        self.integral_inner = np.zeros(4)
        self.prev_e_inner = np.zeros(4)
        self.saturated_count = 0

    def compute_control(
        self,
        pose_des: np.ndarray,
        pose_curr: np.ndarray,
        vel_curr: np.ndarray,
        dt: float,
        vel_ff: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, dict]:
        """
        Args:
            pose_des: [x,y,z,yaw] (NED world)
            pose_curr: [x,y,z,roll,pitch,yaw] (NED world)
            vel_curr: [u,v,w,p,q,r] (FRD body)
            dt: 시간 스텝 (s)
            vel_ff: [u,v,w,r] (FRD body) path-tangent feedforward, 또는 None
        Returns:
            (tau_6dof [Fx,Fy,Fz,0,0,Mz], debug_info)
        """
        roll, pitch, yaw = pose_curr[3], pose_curr[4], pose_curr[5]

        # ===== OUTER: 위치오차 → 속도 setpoint (body FRD) =====
        R = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_matrix()
        e_pos_world = pose_des[0:3] - pose_curr[0:3]
        e_pos_body = R.T @ e_pos_world                    # F2와 동일
        e_yaw = angle_wrap(pose_des[3] - yaw)
        e_outer = np.array([e_pos_body[0], e_pos_body[1], e_pos_body[2], e_yaw])

        v_sp = self.Kp_outer * e_outer                    # P-only

        if vel_ff is not None:
            v_sp = v_sp + np.asarray(vel_ff, dtype=float)  # path-tangent ff

        v_sp = np.clip(v_sp, -self.v_sp_limit, self.v_sp_limit)  # ff 합산 후 포화

        # ===== INNER: 속도오차 → 힘/토크 (body FRD) =====
        v_body = np.array([vel_curr[0], vel_curr[1], vel_curr[2], vel_curr[5]])
        e_inner = v_sp - v_body

        p_in = self.Kp_inner * e_inner
        d_in = self.Kd_inner * (-v_body)                  # 기본 Kd_inner=0
        self.integral_inner += 0.5 * (e_inner + self.prev_e_inner) * dt  # 사다리꼴(F3 동일)
        self.integral_inner = np.clip(self.integral_inner,
                                      -self.integral_limit, self.integral_limit)
        i_in = self.Ki_inner * self.integral_inner
        self.prev_e_inner = e_inner.copy()

        tau = p_in + d_in + i_in
        tau_sat = np.clip(tau, -self.sat_limit, self.sat_limit)

        saturated = not np.allclose(tau, tau_sat, atol=0.01)
        if saturated:
            self.saturated_count += 1
            # back-calculation (F1 position_controller.py:266-274 동일)
            excess = tau - tau_sat
            Ki_diag = self.Ki_inner + 1e-9
            self.integral_inner -= (excess / Ki_diag) * self.Kb_inner

        tau_6dof = np.array([
            tau_sat[0], tau_sat[1], tau_sat[2], 0.0, 0.0, tau_sat[3]
        ])

        debug_info = {
            'v_sp': v_sp,
            'e_outer': e_outer,
            'e_inner': e_inner,
            'integral_inner': self.integral_inner.copy(),
            'tau': tau,
            'tau_sat': tau_sat,
            'saturated': saturated,
            'saturation_count': self.saturated_count,
        }
        return tau_6dof, debug_info
