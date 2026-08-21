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


def test_alos_reset_clears_curvature_state(alos_mod):
    """ALOS.reset()이 super() 경유로 곡률 상태를 지운다 (staleness 상속 고정).

    수정은 ILOS.reset() 한 곳이므로, ALOS가 super().reset()을 우회하도록
    바뀌면 이 테스트가 RED로 잡는다.
    """
    g = _make_alos(alos_mod)
    g._signed_curvature_filtered = 0.7
    g._current_curvature = 0.9
    g.reset()
    assert g._signed_curvature_filtered == 0.0
    assert g._current_curvature == 0.0


def _arc_path(direction):
    """+X 직선 10 m 후 반경 5 m 원호로 꺾는 경로 (direction=+1 우 / -1 좌).

    꼭짓점 델타가 아닌 매끈한 곡률 구간 — 필터가 seed 없이 스스로 수렴할
    수 있는 경로를 준다.
    """
    lead = [[float(i), 0.0, 0.0] for i in range(11)]
    R = 5.0
    # 곡률 추정기는 s±0.1 m 3점 샘플 — 세그먼트가 0.1 m보다 짧아야
    # 원호 위 어느 s에서도 꼭짓점을 걸쳐 κ≠0이 나온다.
    arc = [[10.0 + R * np.sin(phi), direction * (R - R * np.cos(phi)), 0.0]
           for phi in np.linspace(0.02, np.pi / 2, 80)]
    return np.array(lead + arc, dtype=float)


@pytest.mark.parametrize('direction, sign', [(+1, +1), (-1, -1)],
                         ids=['right_arc', 'left_arc'])
def test_alos_arc_filter_converges_without_seed(alos_mod, direction, sign):
    """원호 경로에서 곡률 필터가 seed 없이 옳은 부호로 수렴해 r_d에 전파된다.

    기존 코너 골든은 필터를 수동 seed해 부호만 봤다(리뷰 MEDIUM) —
    이 골든은 compute_guidance의 필터 갱신 경로 자체를 고정한다.
    """
    mod = alos_mod
    g = _make_alos(mod)
    g.set_path(_arc_path(direction))
    g._mode = mod.PathFollowingMode.FOLLOW
    # 필터 입력은 s가 아니라 s+lookahead 지점의 곡률 — 그 지점이 원호 내부에
    # 오도록 초기 원호에서 평가한다(경로 끝 클램프 시 κ=0이 들어감).
    s_eval = 11.0
    assert sign * g._estimate_signed_curvature(
        s_eval + g._lookahead_distance) > 0, \
        '경로 곡률 부호 전제 실패 (경로 구성 오류)'
    rd = 0.0
    for _ in range(30):  # tau_up=0.3 s, dt=0.1 → 3 s면 수렴 충분
        g._path_parameter_s = s_eval
        g._vehicle_pos = g._interpolate_from_parameter(s_eval)
        _, _, vel = g.compute_guidance(dt=0.1)
        rd = vel[3]
    assert sign * g._signed_curvature_filtered > 0, \
        '필터가 경로 곡률 부호로 수렴하지 않음'
    assert sign * rd > 0, f'r_d 부호가 회전 방향과 불일치 (r_d={rd})'
