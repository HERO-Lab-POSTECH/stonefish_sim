# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""컨트롤러 포화 한계 ↔ 추진기 물리 한계 정합 게이트 (fix/thrust-map).

컨트롤러 config의 max_force/max_torque가 TAM×T_max로 계산한 축별 **단일축**
상한을 넘으면, anti-windup이 실효 영역에서 발동하지 않아 적분 폭주가
재발한다(2026-08-22 진단: 종전 800 N/160 N·m ≈ 물리의 14배). 이 게이트는
설정 drift로 그 상태가 되돌아오는 것을 막는다. 다축 동시명령의 초과분은
allocator의 방향보존 균등 스케일링이 처리한다(별도 유닛테스트).
"""
import math
import re
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]
CONTROL_CONFIG_DIR = REPO / "stonefish_control/stonefish_control/config"
HYBRID_YAML = CONTROL_CONFIG_DIR / "bluerov2/hybrid_controller.yaml"
TAM_YAML = REPO / "stonefish_description/data/robots/bluerov2/config/TAM.yaml"
PF_YAML = REPO / "stonefish_control/stonefish_trajectory_manager/config/path_following.yaml"
ALLOC_NODE = REPO / ("stonefish_control/stonefish_thruster_manager/"
                     "stonefish_thruster_manager/nodes/thruster_allocator_node.py")
ALLOC_LAUNCH = REPO / "stonefish_control/stonefish_thruster_manager/launch/thruster_manager.launch.py"

# bluerov2.scn specs: thrust_coeff=0.167, max_rpm=3600 (n_max=60 rev/s),
# propeller D=0.076 m, environment.scn water density ρ=1031.0 kg/m³
# → T_max = ρ·kT·n²·D⁴ ≈ 20.68 N
T_MAX = 1031.0 * 0.167 * 60.0**2 * 0.076**4


def _axis_limits():
    """TAM 행별 |계수| 합 × T_max = 축별 단일축 최대 힘/토크."""
    tam = np.array(yaml.safe_load(TAM_YAML.read_text())["tam"])
    return np.sum(np.abs(tam), axis=1) * T_MAX  # [Fx,Fy,Fz,Tx,Ty,Tz]


def _iter_saturations(node, path=""):
    """중첩 dict에서 (경로, max_force/max_torque 값) 쌍을 재귀 수집."""
    if isinstance(node, dict):
        for k, v in node.items():
            if k in ("max_force", "max_torque"):
                yield f"{path}.{k}", k, v
            else:
                yield from _iter_saturations(v, f"{path}.{k}")


def test_derived_single_axis_limits_match_documented_values():
    """TAM·scn 사양이 바뀌면 문서화된 상한 수치도 재유도해야 한다."""
    lim = _axis_limits()
    assert math.isclose(lim[0], 58.5, abs_tol=0.1)   # surge
    assert math.isclose(lim[1], 58.5, abs_tol=0.1)   # sway
    assert math.isclose(lim[2], 82.7, abs_tol=0.1)   # heave
    assert math.isclose(lim[5], 13.77, abs_tol=0.05)  # yaw


def test_all_controller_configs_within_physical_limits():
    """control config 전체 glob — 어느 yaml이든 물리 초과 포화는 fail."""
    lim = _axis_limits()
    surge_sway_max = min(lim[0], lim[1])
    yaw_max = lim[5]
    configs = sorted(CONTROL_CONFIG_DIR.rglob("*.yaml"))
    assert configs, "control config yaml이 없다 — 경로 확인"
    checked = 0
    for cfg in configs:
        for path, key, value in _iter_saturations(yaml.safe_load(cfg.read_text())):
            limit = surge_sway_max if key == "max_force" else yaw_max
            assert 0 < value <= limit, f"{cfg.name}:{path} = {value} > {limit:.2f}"
            checked += 1
    assert checked >= 8  # hybrid 3모드 + position, 각 force/torque


def test_allocator_max_thrust_defaults_match_physical_tmax():
    """node·launch의 max_thrust 기본값이 유도 T_max와 어긋나면 fail.

    누가 기본값을 구 정규화 척도(100/200)로 되돌리면 제곱 왜곡이
    재현되므로, 프로덕션 기본값 자체를 게이트한다.
    """
    node_m = re.search(r"declare_parameter\('max_thrust',\s*([\d.]+)\)",
                       ALLOC_NODE.read_text())
    launch_m = re.search(r"'max_thrust',\n(?:\s*#.*\n)*\s*default_value='([\d.]+)'",
                         ALLOC_LAUNCH.read_text())
    assert node_m and launch_m, "max_thrust 기본값 선언을 찾지 못함"
    assert math.isclose(float(node_m.group(1)), T_MAX, abs_tol=0.05)
    assert math.isclose(float(launch_m.group(1)), T_MAX, abs_tol=0.05)


def test_cascade_surge_limit_covers_cruise_speed():
    """v_sp_limit[surge] ≥ cruise_speed — 위반 시 상시 windup (종전 0.5 vs 1.0)."""
    p = yaml.safe_load(HYBRID_YAML.read_text())["/**"]["ros__parameters"]
    cruise = yaml.safe_load(PF_YAML.read_text())["/**"]["ros__parameters"]["cruise_speed"]
    assert p["cascade"]["v_sp_limit"][0] >= cruise


DYN_YAML = REPO / "stonefish_description/data/robots/bluerov2/config/dynamics_params.yaml"


def test_cascade_inner_kp_matches_measured_effective_mass():
    """P2 게이트: cascade inner Kp(병진 3축) = (mass+added_mass_diag)·ω_c (ω_c=1).

    실측 질량(dynamics_params.yaml)과 게인(hybrid_controller.yaml)의 drift 방지.
    ω_c=2는 폐루프에서 반증(allocator per-thruster 예산 초과 → 기동 정렬
    불안정, yaml 주석 참조) — 1 rad/s로 하향.
    yaw는 I_zz 실측 불확실로 경험값 유지라 제외(설정 주석 참조).
    """
    dyn = yaml.safe_load(DYN_YAML.read_text())["/**"]["ros__parameters"]
    hyb = yaml.safe_load(HYBRID_YAML.read_text())["/**"]["ros__parameters"]
    mass = dyn["mass"]
    ma = dyn["added_mass_diag"]
    assert len(ma) == 6 and all(v >= 0.0 for v in ma)
    kp_inner = hyb["cascade"]["inner_loop"]["Kp"]
    omega_c = 1.0
    for i in range(3):
        expected = (mass + ma[i]) * omega_c
        assert math.isclose(kp_inner[i], expected, rel_tol=0.02), (
            f"axis {i}: Kp {kp_inner[i]} != (m+Ma)·ω_c {expected:.1f}")


def test_cascade_surge_limit_within_measured_max_speed():
    """P2 게이트: v_sp_limit[0] ≤ 실측 최대속도 0.911 m/s (55 N 프로브).

    도달 불가 setpoint 상한은 상시 포화·windup 압력원(P2에서 1.2→0.7 교정).
    """
    hyb = yaml.safe_load(HYBRID_YAML.read_text())["/**"]["ros__parameters"]
    assert hyb["cascade"]["v_sp_limit"][0] <= 0.911


HYBRID_NODE = REPO / ("stonefish_control/stonefish_control/stonefish_control/"
                      "nodes/hybrid_controller_node.py")


def _node_declared_default(param_name):
    """hybrid_controller_node.py의 declare_parameter 기본값 리스트 파싱."""
    src = HYBRID_NODE.read_text()
    m = re.search(
        rf"declare_parameter\('{re.escape(param_name)}',\s*(\[[^\]]*\])", src)
    assert m, f"declare_parameter('{param_name}', ...) not found"
    return [float(x) for x in re.findall(r"[-\d.]+", m.group(1))]


def test_node_defaults_match_yaml_for_p2_values():
    """P2 게이트(리뷰 MAJOR-2): node declare 기본값 == YAML (silent-fallback 방어).

    YAML만 갱신하면 launch 경유(정상)에서는 안 보이지만, YAML 누락 시
    구 플랜트 게인이 부활한다 — 기본값 drift 자체를 게이트로 차단.
    """
    hyb = yaml.safe_load(HYBRID_YAML.read_text())["/**"]["ros__parameters"]
    for pname, ypath in [
        ("cascade.inner_loop.Kp", ("cascade", "inner_loop", "Kp")),
        ("cascade.inner_loop.Ki", ("cascade", "inner_loop", "Ki")),
        ("cascade.v_sp_limit", ("cascade", "v_sp_limit")),
    ]:
        node_val = _node_declared_default(pname)
        yaml_val = hyb
        for k in ypath:
            yaml_val = yaml_val[k]
        assert node_val == [float(v) for v in yaml_val], (
            f"{pname}: node default {node_val} != yaml {yaml_val}")
