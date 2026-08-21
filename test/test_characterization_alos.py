#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""Characterization: ALOSGuidance yaw rate feedforward 부호 (감사 버그 2).

ALOS는 ILOS를 상속하지만 `compute_guidance`를 통째로 오버라이드하므로 P7 결함 A
수정(r_d에 부호 있는 곡률 사용)을 물려받지 못했다. 좌/우 코너에서 r_d 부호가
반대인지를 골든으로 고정한다 — 부호 없는 곡률 회귀 시 RED.

로드 방식:
  alos_guidance.py는 `from .ilos_guidance import ...` 상대 import를 쓰므로
  conftest의 load_module(단일 파일 로드)로는 실행할 수 없다. 합성 패키지
  네임스페이스를 만들어 두 모듈을 함께 올린다(형제 ROS 모듈은 여전히 미로드).
"""

import importlib.util
import sys
import types
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent

_PF_DIR = REPO_ROOT / ('stonefish_control/stonefish_trajectory_manager/'
                       'stonefish_trajectory_manager/path_following')

_PKG = '_alos_char_pkg'


@pytest.fixture
def alos_mod():
    """alos_guidance를 형제 ilos_guidance와 함께 합성 패키지로 로드."""
    pkg = types.ModuleType(_PKG)
    pkg.__path__ = [str(_PF_DIR)]
    sys.modules[_PKG] = pkg
    names = [_PKG]
    try:
        for name in ('ilos_guidance', 'alos_guidance'):
            full = f'{_PKG}.{name}'
            spec = importlib.util.spec_from_file_location(
                full, str(_PF_DIR / f'{name}.py'))
            mod = importlib.util.module_from_spec(spec)
            sys.modules[full] = mod
            names.append(full)
            spec.loader.exec_module(mod)
        yield sys.modules[f'{_PKG}.alos_guidance']
    finally:
        for n in names:
            sys.modules.pop(n, None)


def _left_turn_path():
    """+X로 가다 -Y로 꺾는 좌회전 경로 (NED: North→West, κ_signed<0)."""
    return np.array([
        [0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [10.0, 0.0, 0.0],
        [10.0, -5.0, 0.0], [10.0, -10.0, 0.0],
    ])


def _right_turn_path():
    """+X로 가다 +Y로 꺾는 우회전 경로 (NED: North→East, κ_signed>0)."""
    return np.array([
        [0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [10.0, 0.0, 0.0],
        [10.0, 5.0, 0.0], [10.0, 10.0, 0.0],
    ])


def _make_alos(mod):
    return mod.ALOSGuidance(
        lookahead_distance=5.0,
        cruise_speed=1.0,
        adaptive_lookahead=False,
    )


def _rd_at_corner(mod, path, s_at):
    """코너 지점에서 compute_guidance를 1틱 돌려 r_d와 실측 κ_signed를 얻는다."""
    g = _make_alos(mod)
    g.set_path(path)
    g._mode = mod.PathFollowingMode.FOLLOW
    g._path_parameter_s = s_at
    g._vehicle_pos = g._interpolate_from_parameter(s_at)
    # piecewise-linear 경로의 곡률은 꼭짓점에 델타로 몰린다 — 코너 지점에서 직접
    # 평가해 필터를 시드한다(1틱 필터 lag 제거, 부호만 검증).
    kappa = g._estimate_signed_curvature(s_at)
    g._signed_curvature_filtered = kappa
    _, _, vel = g.compute_guidance(dt=0.1)
    return vel[3], kappa


def test_alos_rd_sign_opposite_on_left_vs_right(alos_mod):
    """좌회전과 우회전에서 r_d 부호가 반대다 (부호 없는 곡률 회귀 차단).

    부호 없는 `_current_curvature`(max-preview 크기)를 쓰면 두 경우 r_d가 모두
    양수라 실패한다.
    """
    rd_left, k_left = _rd_at_corner(alos_mod, _left_turn_path(), 10.0)
    rd_right, k_right = _rd_at_corner(alos_mod, _right_turn_path(), 10.0)

    assert k_left < 0 and k_right > 0, \
        f'경로 부호 실측 실패: 좌={k_left}, 우={k_right} (SSOT: 좌<0, 우>0)'
    assert rd_left * rd_right < 0, \
        (f'r_d가 좌/우에서 같은 부호 — 부호 없는 곡률 회귀 '
         f'(좌={rd_left}, 우={rd_right})')
    # FRD 관례: 우회전 r>0(starboard), 좌회전 r<0.
    assert rd_left < 0 < rd_right, \
        f'r_d 부호 관례 위반 (좌={rd_left}는 <0, 우={rd_right}는 >0이어야)'


def test_alos_rd_zero_on_straight(alos_mod):
    """직선 경로(κ_signed=0)에서 r_d=0 (직선 거동 무손상 회귀)."""
    mod = alos_mod
    g = _make_alos(mod)
    g.set_path(np.array([[float(i), 0.0, 0.0] for i in range(11)]))
    g._mode = mod.PathFollowingMode.FOLLOW
    g._path_parameter_s = 2.0
    g._vehicle_pos = np.array([2.0, 0.0, 0.0])
    g._signed_curvature_filtered = 0.0
    _, _, vel = g.compute_guidance(dt=0.1)
    assert vel[3] == pytest.approx(0.0, abs=1e-12), '직선 κ=0 → r_d=0'


def test_alos_speed_profile_unaffected_by_turn_direction(alos_mod):
    """좌/우 코너의 desired_speed는 동일 — 속도 프로파일은 부호 없는 곡률 유지.

    r_d만 signed로 바꾸고 속도 프로파일러는 건드리지 않았음을 수치로 고정한다.
    """
    mod = alos_mod

    def _speed(path):
        g = _make_alos(mod)
        g.set_path(path)
        g._mode = mod.PathFollowingMode.FOLLOW
        g._path_parameter_s = 10.0
        g._vehicle_pos = g._interpolate_from_parameter(10.0)
        _, _, vel = g.compute_guidance(dt=0.1)
        return vel[0]

    s_left = _speed(_left_turn_path())
    s_right = _speed(_right_turn_path())
    assert s_left == pytest.approx(s_right, abs=1e-12), \
        (f'좌/우 desired_speed 비대칭 ({s_left} vs {s_right}) — '
         f'속도 프로파일러가 부호 있는 곡률을 소비함')
