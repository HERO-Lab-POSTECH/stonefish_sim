#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Characterization: ILOSGuidance.compute_guidance 동작 동결 (P3 안전망).

god-method 분해(P3) 전 현재 동작을 바이트 수준으로 동결한다.
분해 후 이 테스트가 모두 GREEN이면 동작보존이 증명된다.

테스트 구성 (시나리오 7개):
  S1: 경로 미설정(None) — 즉시 반환 early-exit
  S2: 직선 수평 경로, 차량이 경로 위 (zero CTE)
  S3: 직선 경로, 차량이 경로 오른쪽 1m — CTE 보정 헤딩·sway 속도
  S4: L자형 경로 (90° 커브), 직선 구간 내 차량 — 속도 프로파일링
  S5: ALIGN 모드 — 경로 시작 접선 고정 헤딩, 적분 갱신 없음
  S6: 3D 하강 경로 — heave 속도(피드포워드 + depth 보정)
  S7: 경로 끝 부근 — lookahead 클램프 + 최소 속도(10%) 보장

오라클 설계 원칙:
  - 현재 동작이 정답; 버그처럼 보여도 동결만 하고 수정 안 함
  - np.testing.assert_allclose(atol=1e-9) 로 수치 정밀 비교
  - 상태 부작용(_cross_track_error, _mode, _integral_ey 등) 포함 검증
  - conftest.load_module fixture 로 모듈 직접 로드 (ROS/gtsam 오염 우회)

로드 방식:
  ilos_guidance.py 는 numpy·enum·transforms3d 만 import 하므로 stub 불필요.
  load_module fixture 가 spec_from_file_location 으로 직접 실행한다.
"""

from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent

_ILOS_PATH = (
    'stonefish_control/stonefish_trajectory_manager/'
    'stonefish_trajectory_manager/path_following/ilos_guidance.py'
)


# ── 공통 헬퍼 ────────────────────────────────────────────────────────────────

def _make_guidance(mod, *, adaptive=False):
    """기본 파라미터로 ILOSGuidance 인스턴스 생성 (adaptive_lookahead 선택 가능)."""
    return mod.ILOSGuidance(
        lookahead_distance=5.0,
        cruise_speed=1.0,
        curvature_gain=6.0,
        lateral_gain=0.6,
        integral_gain=0.0,
        adaptive_lookahead=adaptive,
    )


def _straight_path(n=11):
    """X 축 방향 직선 수평 경로 (0..n-1, y=0, z=0)."""
    return np.array([[float(i), 0.0, 0.0] for i in range(n)], dtype=float)


def _l_shape_path():
    """L 자형 경로: +X 방향 5m → +Y 방향 5m."""
    seg1 = [[float(i), 0.0, 0.0] for i in range(6)]
    seg2 = [[5.0, float(j), 0.0] for j in range(1, 6)]
    return np.array(seg1 + seg2, dtype=float)


# ── S1: 경로 미설정 early-exit ───────────────────────────────────────────────

def test_empty_path_returns_zero_outputs(load_module):
    """경로가 None인 상태에서 compute_guidance는 초기화된 zeros를 반환한다."""
    mod = load_module(_ILOS_PATH, 'char_ilos')
    g = _make_guidance(mod)
    # set_path 호출 없이 바로 compute_guidance 호출
    pos, yaw, vel = g.compute_guidance(dt=0.1)

    np.testing.assert_allclose(pos, [0.0, 0.0, 0.0], atol=1e-9,
                               err_msg='empty path: pos must be zeros')
    assert yaw == pytest.approx(0.0), 'empty path: yaw must be 0.0'
    np.testing.assert_allclose(vel, [0.0, 0.0, 0.0, 0.0], atol=1e-9,
                               err_msg='empty path: vel must be zeros')


# ── S2: 직선 경로, 차량이 경로 위 (zero CTE) ────────────────────────────────

def test_straight_path_on_path_heading_zero_and_cruise_speed(load_module):
    """직선 +X 경로에서 차량이 경로 위에 있을 때 헤딩은 0, 속도는 cruise_speed."""
    mod = load_module(_ILOS_PATH, 'char_ilos')
    PathFollowingMode = mod.PathFollowingMode

    g = _make_guidance(mod, adaptive=False)
    g.set_path(_straight_path())
    g._vehicle_pos = np.array([3.0, 0.0, 0.0])
    g._vehicle_yaw = 0.0
    g._vehicle_velocity = np.array([1.0, 0.0, 0.0])
    g._mode = PathFollowingMode.FOLLOW
    g._path_parameter_s = 3.0

    pos, yaw, vel = g.compute_guidance(dt=0.1)

    # lookahead point: s=3 + 5=8 → [8, 0, 0]
    np.testing.assert_allclose(pos, [8.0, 0.0, 0.0], atol=1e-9,
                               err_msg='S2: lookahead point should be [8,0,0]')
    # No CTE → ILOS heading = chi_p = arctan2(0,1) = 0
    assert yaw == pytest.approx(0.0, abs=1e-9), 'S2: heading must be 0 on straight path'
    # surge = cruise_speed=1.0 (speed_factor≈1.0 since lookahead_dist=5m = lookahead_distance)
    assert vel[0] == pytest.approx(1.0, abs=1e-9), 'S2: surge must be cruise_speed'
    # No lateral error → sway ≈ 0
    assert vel[1] == pytest.approx(0.0, abs=1e-9), 'S2: sway must be 0 with zero CTE'
    # Flat path → heave ≈ 0 (no depth error)
    assert vel[2] == pytest.approx(0.0, abs=1e-9), 'S2: heave must be 0 on flat path'
    # Straight path → yaw rate = 0
    assert vel[3] == pytest.approx(0.0, abs=1e-9), 'S2: yaw rate must be 0 on straight'


def test_straight_path_on_path_cte_is_zero(load_module):
    """차량이 직선 경로 위에 있으면 cross-track error가 0이어야 한다."""
    mod = load_module(_ILOS_PATH, 'char_ilos')
    PathFollowingMode = mod.PathFollowingMode

    g = _make_guidance(mod, adaptive=False)
    g.set_path(_straight_path())
    g._vehicle_pos = np.array([3.0, 0.0, 0.0])
    g._vehicle_yaw = 0.0
    g._vehicle_velocity = np.array([1.0, 0.0, 0.0])
    g._mode = PathFollowingMode.FOLLOW
    g._path_parameter_s = 3.0

    g.compute_guidance(dt=0.1)

    assert g._cross_track_error == pytest.approx(0.0, abs=1e-9), \
        'S2: CTE must be exactly 0 when vehicle is on path'


# ── S3: 직선 경로, 차량 오른쪽 1m — CTE 보정 헤딩·sway ─────────────────────

def test_right_offset_heading_is_pure_path_tangent(load_module):
    """[축소] 차량이 경로 오른쪽 1m여도 ILOS 헤딩은 순수 path-tangent χ_p=0.

    설계 SSOT §4: cross-track의 heading 채널(arctan) 제거. e_y 보정은 cascade
    outer가 전담하므로 ILOS는 chi_d=chi_p만 출력한다(직선 +X 경로 → χ_p=0).
    """
    mod = load_module(_ILOS_PATH, 'char_ilos')
    PathFollowingMode = mod.PathFollowingMode

    g = _make_guidance(mod, adaptive=False)
    g.set_path(_straight_path())
    g._vehicle_pos = np.array([3.0, 1.0, 0.0])   # 1m starboard
    g._vehicle_yaw = 0.0
    g._vehicle_velocity = np.array([1.0, 0.0, 0.0])
    g._mode = PathFollowingMode.FOLLOW
    g._path_parameter_s = 3.0

    pos, yaw, vel = g.compute_guidance(dt=0.1)

    # 축소 후: chi_d = chi_p = arctan2(0,1) = 0 (CTE와 무관, arctan 항 제거)
    assert yaw == pytest.approx(0.0, abs=1e-9), \
        'S3[축소]: heading must be pure path tangent χ_p=0 (no cross-track arctan)'


def test_right_offset_cte_computed_but_sway_zero(load_module):
    """[축소] 차량이 오른쪽 1m: CTE=+1.0은 여전히 계산, sway 출력은 0.

    설계 SSOT §4: cross-track의 sway 채널(PID) 제거. desired_velocity[1]=0.0.
    cascade outer e_pos_body[1]이 sway를 전담(이중보정 제거).
    """
    mod = load_module(_ILOS_PATH, 'char_ilos')
    PathFollowingMode = mod.PathFollowingMode

    g = _make_guidance(mod, adaptive=False)
    g.set_path(_straight_path())
    g._vehicle_pos = np.array([3.0, 1.0, 0.0])
    g._vehicle_yaw = 0.0
    g._vehicle_velocity = np.array([1.0, 0.0, 0.0])
    g._mode = PathFollowingMode.FOLLOW
    g._path_parameter_s = 3.0

    pos, yaw, vel = g.compute_guidance(dt=0.1)

    assert g._cross_track_error == pytest.approx(1.0, abs=1e-9), \
        'S3[축소]: CTE must still be computed (+1.0) for logging/diagnostics'
    assert vel[1] == pytest.approx(0.0, abs=1e-9), \
        'S3[축소]: sway must be 0 (cross-track sway channel removed)'


# ── S4: L자형 경로, 커브 전 속도 프로파일링 ─────────────────────────────────

def test_l_shape_path_speed_reduced_by_curvature_preview(load_module):
    """L자형 경로에서 커브 전방 preview로 속도가 cruise_speed 미만으로 감소한다."""
    mod = load_module(_ILOS_PATH, 'char_ilos')
    PathFollowingMode = mod.PathFollowingMode

    g = _make_guidance(mod, adaptive=False)
    g.set_path(_l_shape_path())
    g._vehicle_pos = np.array([2.0, 0.0, 0.0])
    g._vehicle_yaw = 0.0
    g._vehicle_velocity = np.array([1.0, 0.0, 0.0])
    g._mode = PathFollowingMode.FOLLOW
    g._path_parameter_s = 2.0

    pos, yaw, vel = g.compute_guidance(dt=0.1)

    # Oracle: desired_speed ≈ 0.721 due to lookahead speed reduction
    np.testing.assert_allclose(vel[0], 0.72111025509279, atol=1e-9,
                               err_msg='S4: surge must match speed-profiled oracle')
    # Lookahead point: s=2+5=7 → in the +Y segment → [5, 2, 0]
    np.testing.assert_allclose(pos, [5.0, 2.0, 0.0], atol=1e-9,
                               err_msg='S4: lookahead point should be [5,2,0]')


def test_l_shape_path_heading_still_east_in_straight_segment(load_module):
    """L자형 경로의 직선 구간에서 헤딩은 동쪽(0 rad)이어야 한다."""
    mod = load_module(_ILOS_PATH, 'char_ilos')
    PathFollowingMode = mod.PathFollowingMode

    g = _make_guidance(mod, adaptive=False)
    g.set_path(_l_shape_path())
    g._vehicle_pos = np.array([2.0, 0.0, 0.0])
    g._vehicle_yaw = 0.0
    g._vehicle_velocity = np.array([1.0, 0.0, 0.0])
    g._mode = PathFollowingMode.FOLLOW
    g._path_parameter_s = 2.0

    _, yaw, _ = g.compute_guidance(dt=0.1)

    # chi_p at s=2 on straight +X segment → arctan2(0,1) = 0
    assert yaw == pytest.approx(0.0, abs=1e-9), \
        'S4: heading must be 0 rad in straight +X segment'


# ── S5: ALIGN モード ─────────────────────────────────────────────────────────

def test_align_mode_heading_is_path_start_tangent(load_module):
    """ALIGN モード에서 헤딩은 경로 시작 접선(+X → 0 rad)으로 고정된다."""
    mod = load_module(_ILOS_PATH, 'char_ilos')

    g = _make_guidance(mod, adaptive=False)
    g.set_path(_straight_path())
    g._vehicle_pos = np.array([0.5, 0.0, 0.0])
    g._vehicle_yaw = np.pi / 4   # 45° — not aligned
    g._vehicle_velocity = np.zeros(3)
    g._path_parameter_s = 0.0
    # mode is ALIGN by default

    _, yaw, vel = g.compute_guidance(dt=0.1)

    assert yaw == pytest.approx(0.0, abs=1e-9), \
        'S5: ALIGN mode heading must be path start tangent (0 rad for +X path)'
    # ALIGN mode: surge overridden to 0.3 m/s
    assert vel[0] == pytest.approx(0.3, abs=1e-9), \
        'S5: ALIGN mode surge must be 0.3 m/s (slow approach)'


def test_align_mode_does_not_update_ilos_integral(load_module):
    """ALIGN モード에서는 ILOS 적분(_integral_ey)이 갱신되지 않는다."""
    mod = load_module(_ILOS_PATH, 'char_ilos')

    g = _make_guidance(mod, adaptive=False)
    g.set_path(_straight_path())
    g._vehicle_pos = np.array([0.5, 0.5, 0.0])  # 0.5m right offset
    g._vehicle_yaw = np.pi / 4
    g._vehicle_velocity = np.zeros(3)
    g._path_parameter_s = 0.0
    # integral_ey starts at 0.0

    g.compute_guidance(dt=0.1)

    assert g._integral_ey == pytest.approx(0.0, abs=1e-9), \
        'S5: _integral_ey must stay 0 in ALIGN mode (no integral accumulation)'


def test_align_to_follow_transition_when_heading_within_threshold(load_module):
    """헤딩 오차가 10° 미만이면 ALIGN → FOLLOW로 전환된다."""
    mod = load_module(_ILOS_PATH, 'char_ilos')

    g = _make_guidance(mod, adaptive=False)
    g.set_path(_straight_path())
    g._vehicle_pos = np.array([0.5, 0.0, 0.0])
    g._vehicle_yaw = 0.005   # ≈ 0.29° — well within 10° threshold
    g._vehicle_velocity = np.zeros(3)
    g._path_parameter_s = 0.0
    # mode is ALIGN

    g.compute_guidance(dt=0.1)

    assert g._mode == mod.PathFollowingMode.FOLLOW, \
        'S5→FOLLOW: mode must transition to FOLLOW when heading error < threshold'


# ── S6: 3D 하강 경로 — heave 속도 ────────────────────────────────────────────

def test_descending_3d_path_heave_velocity(load_module):
    """3D 하강 경로에서 heave 속도는 경로 기울기 피드포워드 + depth 보정 합이다."""
    mod = load_module(_ILOS_PATH, 'char_ilos')
    PathFollowingMode = mod.PathFollowingMode

    # Path: 10m forward, 5m down (z increases = NED down)
    path = np.array([[float(i), 0.0, float(i) * 0.5] for i in range(11)], dtype=float)

    g = _make_guidance(mod, adaptive=False)
    g.set_path(path)
    # Vehicle on path at s≈3.35 (3D arc-length for [3,0,1.5])
    g._vehicle_pos = np.array([3.0, 0.0, 1.5])
    g._vehicle_yaw = 0.0
    g._vehicle_velocity = np.array([1.0, 0.0, 0.5])
    g._mode = PathFollowingMode.FOLLOW
    g._path_parameter_s = 3.35  # approximate arc-length

    pos, yaw, vel = g.compute_guidance(dt=0.1)

    # Oracle from actual execution: heave = 0.25 (clamped to max_heave=0.25)
    assert vel[2] == pytest.approx(0.25, abs=1e-9), \
        'S6: heave must be 0.25 (clamped to max_heave_velocity)'
    # Flat horizontal heading (path descends but no horizontal turn)
    assert yaw == pytest.approx(0.0, abs=1e-9), \
        'S6: heading must be 0 (path tangent in horizontal plane is +X)'


def test_descending_3d_path_surge_near_cruise_speed(load_module):
    """3D 하강 경로(직선)에서 surge는 cruise_speed에 가까워야 한다."""
    mod = load_module(_ILOS_PATH, 'char_ilos')
    PathFollowingMode = mod.PathFollowingMode

    path = np.array([[float(i), 0.0, float(i) * 0.5] for i in range(11)], dtype=float)

    g = _make_guidance(mod, adaptive=False)
    g.set_path(path)
    g._vehicle_pos = np.array([3.0, 0.0, 1.5])
    g._vehicle_yaw = 0.0
    g._vehicle_velocity = np.array([1.0, 0.0, 0.5])
    g._mode = PathFollowingMode.FOLLOW
    g._path_parameter_s = 3.35

    _, _, vel = g.compute_guidance(dt=0.1)

    # Oracle: surge ≈ 0.9992 (minor speed factor reduction due to lookahead distance)
    np.testing.assert_allclose(vel[0], 9.99179607e-01, atol=1e-6,
                               err_msg='S6: surge must be near cruise_speed on straight 3D path')


# ── S7: 경로 끝 부근 — lookahead 클램프 + 최소 속도 ─────────────────────────

def test_near_end_lookahead_clamped_to_path_end(load_module):
    """경로 끝 부근에서 lookahead point는 경로 끝점([10,0,0])으로 클램프된다."""
    mod = load_module(_ILOS_PATH, 'char_ilos')
    PathFollowingMode = mod.PathFollowingMode

    g = _make_guidance(mod, adaptive=False)
    g.set_path(_straight_path())
    g._vehicle_pos = np.array([9.5, 0.0, 0.0])
    g._vehicle_yaw = 0.0
    g._vehicle_velocity = np.array([1.0, 0.0, 0.0])
    g._mode = PathFollowingMode.FOLLOW
    g._path_parameter_s = 9.5

    pos, _, _ = g.compute_guidance(dt=0.1)

    np.testing.assert_allclose(pos, [10.0, 0.0, 0.0], atol=1e-9,
                               err_msg='S7: lookahead must clamp to path end [10,0,0]')


def test_near_end_minimum_speed_floor_applied(load_module):
    """경로 끝 부근에서 lookahead가 짧아도 최소 속도 10%가 보장된다."""
    mod = load_module(_ILOS_PATH, 'char_ilos')
    PathFollowingMode = mod.PathFollowingMode

    g = _make_guidance(mod, adaptive=False)
    g.set_path(_straight_path())
    g._vehicle_pos = np.array([9.5, 0.0, 0.0])
    g._vehicle_yaw = 0.0
    g._vehicle_velocity = np.array([1.0, 0.0, 0.0])
    g._mode = PathFollowingMode.FOLLOW
    g._path_parameter_s = 9.5

    _, _, vel = g.compute_guidance(dt=0.1)

    # Oracle: surge=0.1 (speed_factor = 0.5/5.0 = 0.1 → exactly at 10% floor)
    assert vel[0] == pytest.approx(0.1, abs=1e-9), \
        'S7: surge must be 0.1 (10% floor of cruise_speed when lookahead is small)'


# ── S8: 곡률 sway feedforward (P6) ─────────────────────────────────────────────

def test_sway_ff_gain_param_stored(load_module):
    """sway_ff_gain 생성자 파라미터가 _sway_ff_gain에 저장된다 (기본 0.1)."""
    mod = load_module(_ILOS_PATH, 'char_ilos')
    g = mod.ILOSGuidance(sway_ff_gain=0.25)
    assert g._sway_ff_gain == 0.25
    g_default = mod.ILOSGuidance()
    assert g_default._sway_ff_gain == 0.1, 'sway_ff_gain 기본값은 0.1 (≈m/Kp_inner)'
