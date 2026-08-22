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
    - vel_ff: ILOS feedforward 속도 [u, v_sway_ff, w_d, r]. sway(인덱스1)=곡률 ff(P6부터 비0).
    - v_sp clamp는 ff 합산 후 적용(inner 포화 사전차단).
    - accel ff (P2 모델 주입, 기본 비활성): clip 후 v_sp를 내부에서 수치미분
      (1차 LPF)해 M_eff·v̇_sp를 inner에 합산. inner가 추종하는 setpoint의
      미분이므로 matched feedforward다 — guidance 원신호 미분은 outer surge
      상시 clip(리뷰 BLOCKER-1) 때문에 unmatched disturbance가 되어 금지.
      ⚠ 폐루프 어블레이션(2026-08-22 runE/runF)에서 ON(2 Hz)이 직선 leg
      사행을 유발(e_y RMS 0.456→OFF 0.227, v_y_std 0.15→≤0.03): v_sp는
      outer P를 통해 차량 위치의 함수라, v_sp 미분은 차량 운동을 힘으로
      되먹이는 폐루프 자기되먹임이 된다(개루프 오라클 테스트로는 안 잡힘).
      기본 cutoff 0(off) — 켜려면 재검증 필수.
      한계: 강체 가속만 반영 — Coriolis/구심 항(ω×v, 선회 시 sway r·u)은 미모델.
    - damping/static ff (P2): d1·v_sp + d2·v_sp|v_sp| + 정적(부력) — 정상상태
      유지력을 모델이 선지불해 적분기 상한(sat·safety_factor)에 걸린 정상상태
      droop을 제거. 계수는 실측(dynamics_params.yaml), 미공급 시 0.

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
        max_force: float = 55.0,
        max_torque: float = 13.7,
        integral_safety_factor: float = 0.5,
        M_eff_diag: Optional[np.ndarray] = None,
        accel_ff_cutoff_hz: float = 0.0,
        d1_diag: Optional[np.ndarray] = None,
        d2_diag: Optional[np.ndarray] = None,
        static_ff: Optional[np.ndarray] = None,
    ):
        """
        Args:
            Kp_outer: [4] 위치 P 게인 (x,y,z,yaw) → 속도 setpoint
            Kp_inner, Ki_inner, Kd_inner, Kb_inner: [4] inner 속도 PI(+D)+back-calc
            mass, inertia_zz: M_eff_diag 미공급 시 M·a ff의 fallback 질량
            v_sp_limit: [4] 속도 setpoint clamp [u,v,w,r]
            max_force, max_torque: 힘/토크 포화 한계
            integral_safety_factor: inner 적분 한계 자동계산 배율
            M_eff_diag: [4] 실측 유효질량 대각 [m+Ma_u, m+Ma_v, m+Ma_w, Izz_eff]
                — accel ff의 M. 미공급 시 강체 질량만 사용(부가질량 무시).
            accel_ff_cutoff_hz: v_sp 미분 저역필터 차단주파수 [Hz].
                0 이하 = accel ff 비활성. 기본 0 — 폐루프 어블레이션에서
                사행 유발 실측(모듈 docstring 참조), 켜려면 재검증 필수.
            d1_diag, d2_diag: [4] 실측 감쇠 대각 — damping ff
                `d1·v_sp + d2·v_sp|v_sp|` (v_sp 유지에 필요한 정상상태 힘을
                모델이 선지불, 적분기 부담 제거). 미공급 시 0.
            static_ff: [4] 정적 상쇄력 (heave 잔류부력 등). 미공급 시 0.
        """
        self.Kp_outer = np.asarray(Kp_outer, dtype=float)
        self.Kp_inner = np.asarray(Kp_inner, dtype=float)
        self.Ki_inner = np.asarray(Ki_inner, dtype=float)
        self.Kd_inner = np.asarray(Kd_inner, dtype=float)
        self.Kb_inner = np.asarray(Kb_inner, dtype=float)

        # M·a feedforward 질량 (P2 모델 주입): 실측 유효질량 우선, 없으면 강체
        if M_eff_diag is not None:
            self.M = np.diag(np.asarray(M_eff_diag, dtype=float))
        else:
            self.M = np.diag([mass, mass, mass, inertia_zz])

        self.v_sp_limit = np.asarray(v_sp_limit, dtype=float)
        self.max_force = max_force
        self.max_torque = max_torque
        self.sat_limit = np.array([max_force, max_force, max_force, max_torque])

        # inner 적분 한계 (F1 position_controller.py:115-116 동일 공식)
        Ki_diag = self.Ki_inner + 1e-6
        self.integral_limit = self.sat_limit / Ki_diag * integral_safety_factor

        self.accel_ff_cutoff_hz = accel_ff_cutoff_hz
        self.d1 = (np.asarray(d1_diag, dtype=float)
                   if d1_diag is not None else np.zeros(4))
        self.d2 = (np.asarray(d2_diag, dtype=float)
                   if d2_diag is not None else np.zeros(4))
        self.static_ff = (np.asarray(static_ff, dtype=float)
                          if static_ff is not None else np.zeros(4))

        # 상태
        self.integral_inner = np.zeros(4)
        self.prev_e_inner = np.zeros(4)
        self.saturated_count = 0
        self._prev_v_sp = None            # accel ff 수치미분용 직전 v_sp
        self._acc_ff = np.zeros(4)        # accel ff 저역필터 상태

    def reset(self):
        """inner 적분·이전오차·accel ff 미분기 초기화 (모드 진입 시 bumpless)."""
        self.integral_inner = np.zeros(4)
        self.prev_e_inner = np.zeros(4)
        self.saturated_count = 0
        self._prev_v_sp = None
        self._acc_ff = np.zeros(4)

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

        accel ff는 인자가 아니라 내부에서 clip 후 v_sp를 미분해 생성한다
        (모듈 docstring 참조).
        """
        roll, pitch, yaw = pose_curr[3], pose_curr[4], pose_curr[5]

        # ===== OUTER: 위치오차 → 속도 setpoint (body FRD) =====
        R = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_matrix()
        e_pos_world = pose_des[0:3] - pose_curr[0:3]
        e_pos_body = R.T @ e_pos_world                    # F2와 동일
        e_yaw = angle_wrap(pose_des[3] - yaw)

        # [결함 C] yaw 정렬 전 sway 위치명령 차단 (cos 게이트, sway 채널만).
        # body sway가 world cross-track으로 기여하는 성분이 정확히 cos(e_yaw).
        # yaw가 90°↑ 틀린 채 sway를 내면 차량을 코너 바깥으로 밀어 동그란
        # overshoot가 생긴다. max(.,0)으로 e_yaw>90°의 역방향 명령을 막는다.
        # surge·heave·yaw·vel_ff는 무관 — sway 채널에만 작용.
        yaw_gate = max(np.cos(e_yaw), 0.0)
        e_outer = np.array([
            e_pos_body[0], e_pos_body[1] * yaw_gate, e_pos_body[2], e_yaw
        ])

        v_sp = self.Kp_outer * e_outer                    # P-only

        if vel_ff is not None:
            v_sp = v_sp + np.asarray(vel_ff, dtype=float)  # path-tangent ff

        v_sp = np.clip(v_sp, -self.v_sp_limit, self.v_sp_limit)  # ff 합산 후 포화

        # ===== accel ff: clip 후 v_sp 수치미분 + 1차 LPF (P2 모델 주입) =====
        # inner가 추종하는 setpoint 자체의 미분이라 matched ff. raw는 물리 달성
        # 가능 최대 |a| = F_max/M_eff ≈ 0.9 m/s²의 2배로 clamp — pose_des 점프
        # (경로 재시작 등) 시 1틱 스파이크가 적분기를 오염시키는 것을 차단.
        if self.accel_ff_cutoff_hz > 0.0 and dt > 0.0:
            if self._prev_v_sp is not None:
                raw = np.clip((v_sp - self._prev_v_sp) / dt, -2.0, 2.0)
                alpha = 1.0 - np.exp(-2.0 * np.pi * self.accel_ff_cutoff_hz * dt)
                self._acc_ff += alpha * (raw - self._acc_ff)
            self._prev_v_sp = v_sp.copy()

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

        # 모델 feedforward (P2): 가속(M_eff·v̇_sp) + 감쇠(d1·v_sp+d2·v_sp|v_sp|)
        # + 정적(부력) — setpoint 유지·가감속 부하를 피드백이 아닌 모델이 선지불
        ff = (self.M @ self._acc_ff
              + self.d1 * v_sp + self.d2 * v_sp * np.abs(v_sp)
              + self.static_ff)

        tau = p_in + d_in + i_in + ff
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
            'tau_ff': ff,
            'tau': tau,
            'tau_sat': tau_sat,
            'saturated': saturated,
            'saturation_count': self.saturated_count,
        }
        return tau_6dof, debug_info
