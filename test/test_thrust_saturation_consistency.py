# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""컨트롤러 포화 한계 ↔ 추진기 물리 한계 정합 게이트 (fix/thrust-map).

hybrid_controller.yaml의 max_force/max_torque가 TAM×T_max로 계산한 축별
물리 한계를 넘으면, anti-windup이 실효 영역에서 발동하지 않아 적분 폭주가
재발한다(2026-08-22 진단: 종전 800 N/160 N·m ≈ 물리의 14배). 이 게이트는
설정 drift로 그 상태가 되돌아오는 것을 막는다.
"""
import math
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]
HYBRID_YAML = REPO / "stonefish_control/stonefish_control/config/bluerov2/hybrid_controller.yaml"
TAM_YAML = REPO / "stonefish_description/data/robots/bluerov2/config/TAM.yaml"
PF_YAML = REPO / "stonefish_control/stonefish_trajectory_manager/config/path_following.yaml"

# bluerov2.scn specs: thrust_coeff=0.167, max_rpm=3600 (n_max=60 rev/s),
# propeller D=0.076 m, 해수 ρ=1027.9 kg/m³ → T_max = ρ·kT·n²·D⁴
T_MAX = 1027.9 * 0.167 * 60.0**2 * 0.076**4  # ≈ 20.62 N


def _params():
    return yaml.safe_load(HYBRID_YAML.read_text())["/**"]["ros__parameters"]


def _axis_limits():
    """TAM 행별 |계수| 합 × T_max = 축별 최대 힘/토크."""
    tam = np.array(yaml.safe_load(TAM_YAML.read_text())["tam"])
    return np.sum(np.abs(tam), axis=1) * T_MAX  # [Fx,Fy,Fz,Tx,Ty,Tz]


def test_derived_axis_limits_match_documented_values():
    lim = _axis_limits()
    assert math.isclose(lim[0], 58.3, abs_tol=0.1)   # surge
    assert math.isclose(lim[1], 58.3, abs_tol=0.1)   # sway
    assert math.isclose(lim[2], 82.5, abs_tol=0.1)   # heave
    assert math.isclose(lim[5], 13.73, abs_tol=0.05)  # yaw


def test_all_modes_saturation_within_physical_limits():
    p = _params()
    lim = _axis_limits()
    surge_sway_max = min(lim[0], lim[1])
    yaw_max = lim[5]
    modes = [p["velocity_mode"], p["position_mode"], p["cascade"]]
    for mode in modes:
        assert mode["max_force"] <= surge_sway_max, mode
        assert mode["max_torque"] <= yaw_max, mode
        # 0이나 음수로 무력화하는 drift도 차단
        assert mode["max_force"] > 0 and mode["max_torque"] > 0


def test_cascade_surge_limit_covers_cruise_speed():
    """v_sp_limit[surge] ≥ cruise_speed — 위반 시 상시 windup (종전 0.5 vs 1.0)."""
    p = _params()
    cruise = yaml.safe_load(PF_YAML.read_text())["/**"]["ros__parameters"]["cruise_speed"]
    assert p["cascade"]["v_sp_limit"][0] >= cruise
