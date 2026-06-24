"""Characterization: LIPBInterpolator.init_interpolator 동작 동결 (P3 god-method 분해 안전망).

분해 *전* 현재 동작을 동결한다. 이 테스트가 GREEN인 한 분해 후 동작보존이 보증된다.

로드 전략:
  - lipb_interpolator.py는 상대 import (from ..common.waypoint …)를 쓰므로
    conftest의 load_module(path-load) fixture를 직접 쓸 수 없다.
  - 대신 테스트 파일 상단에서 fake 패키지 계층을 sys.modules에 주입한 뒤
    importlib.util.spec_from_file_location 으로 각 모듈을 로드한다
    (slam P4c가 cv_bridge를 stub한 패턴의 확장).
  - ROS msg 패키지(visualization_msgs, stonefish_control_msgs, nav_msgs,
    geometry_msgs, builtin_interfaces) 전부 stub으로 대체.

커버리지:
  S1  None waypoints     → False (경계: waypoints 미초기화)
  S2  waypoint 1개       → False (경계: num_waypoints < 2)
  S3  직선 2-waypoint    → True, 단일 LineSegment, 총 길이 10 m
  S4  대각선 2-waypoint  → True, 총 길이 5 m, duration = length/mean_vel
  S5  L자형 3-waypoint   → True, Line+Bezier+Line 3 세그먼트 패턴
  S6  고정 heading 3-wp  → True, spline 보간으로 heading 복원
"""
from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

import numpy as np
import pytest

# ─────────────────────────────────────────────────────────────────────────────
# 1. ROS/외부 메시지 패키지 stub
# ─────────────────────────────────────────────────────────────────────────────

def _stub(name: str, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules.setdefault(name, m)  # 이미 주입된 경우 덮어쓰지 않음
    return m


_stub('visualization_msgs')
_stub('visualization_msgs.msg',
      MarkerArray=type('MarkerArray', (), {'__init__': lambda s: setattr(s, 'markers', [])}),
      Marker=type('Marker', (), {'SPHERE': 2, 'ADD': 0, 'DELETE': 3}))

_stub('stonefish_control_msgs')
_stub('stonefish_control_msgs.msg',
      Waypoint=type('Waypoint', (), {}),
      WaypointSet=type('WaypointSet', (), {}),
      TrajectoryPoint=type('TrajectoryPoint', (), {}))

_stub('nav_msgs')
_stub('nav_msgs.msg', Path=type('Path', (), {}))

_stub('geometry_msgs')
_stub('geometry_msgs.msg',
      PoseStamped=type('PoseStamped', (), {}),
      Point=type('Point', (), {}),
      Quaternion=type('Quaternion', (), {}),
      Vector3=type('Vector3', (), {}))

_stub('builtin_interfaces')
_stub('builtin_interfaces.msg',
      Time=type('Time', (), {'__init__': lambda s, **kw: None}))

# ─────────────────────────────────────────────────────────────────────────────
# 2. 가짜 패키지 계층 + 파일 로더 (상대 import 해소)
# ─────────────────────────────────────────────────────────────────────────────

_REPO = Path(__file__).resolve().parent.parent
_STM = (
    'stonefish_control/stonefish_trajectory_manager'
    '/stonefish_trajectory_manager'
)
_PKG = 'stonefish_trajectory_manager'
_COMMON = _PKG + '.common'
_PG = _PKG + '.path_generator'

for _pname in [_PKG, _COMMON, _PG]:
    sys.modules.setdefault(_pname, types.ModuleType(_pname))


def _load_into(relpath: str, pkg_name: str, mod_name: str):
    """relpath를 pkg_name.mod_name 으로 로드해 sys.modules에 등록."""
    full = pkg_name + '.' + mod_name
    path = _REPO / relpath
    spec = importlib.util.spec_from_file_location(
        full, str(path), submodule_search_locations=[])
    mod = importlib.util.module_from_spec(spec)
    mod.__package__ = pkg_name
    sys.modules[full] = mod
    spec.loader.exec_module(mod)
    setattr(sys.modules[pkg_name], mod_name, mod)
    return mod


_waypoint_mod = _load_into(f'{_STM}/common/waypoint.py',       _COMMON, 'waypoint')
_wp_set_mod   = _load_into(f'{_STM}/common/waypoint_set.py',   _COMMON, 'waypoint_set')
_tp_mod       = _load_into(f'{_STM}/common/trajectory_point.py', _COMMON, 'trajectory_point')
_pg_mod       = _load_into(f'{_STM}/path_generator/path_generator.py', _PG, 'path_generator')
_ls_mod       = _load_into(f'{_STM}/path_generator/line_segment.py',   _PG, 'line_segment')
_bz_mod       = _load_into(f'{_STM}/path_generator/bezier_curve.py',   _PG, 'bezier_curve')
_lipb_mod     = _load_into(f'{_STM}/path_generator/lipb_interpolator.py', _PG, 'lipb_interpolator')

Waypoint    = _waypoint_mod.Waypoint
WaypointSet = _wp_set_mod.WaypointSet
LIPBInterpolator = _lipb_mod.LIPBInterpolator

# ─────────────────────────────────────────────────────────────────────────────
# 3. 헬퍼
# ─────────────────────────────────────────────────────────────────────────────

def _make(waypoints: list) -> LIPBInterpolator:
    """WaypointSet을 주입한 LIPBInterpolator 인스턴스 반환 (init_interpolator 미호출)."""
    obj = LIPBInterpolator()
    ws = WaypointSet()
    for wp in waypoints:
        ws.add_waypoint(wp)
    obj._waypoints = ws
    return obj


# ─────────────────────────────────────────────────────────────────────────────
# 4. Characterization 테스트
# ─────────────────────────────────────────────────────────────────────────────

def test_init_interpolator_returns_false_when_waypoints_is_none():
    """waypoints가 None이면 False를 반환한다 (경계: 미초기화 상태)."""
    obj = LIPBInterpolator()
    assert obj._waypoints is None
    assert obj.init_interpolator() is False


def test_init_interpolator_returns_false_for_single_waypoint():
    """waypoint가 1개(num_waypoints < 2)이면 False를 반환한다."""
    obj = _make([Waypoint(0, 0, 0, max_forward_speed=1.0)])
    assert obj.init_interpolator() is False


def test_init_interpolator_returns_true_for_two_waypoints_straight_line():
    """직선 2-waypoint 경로는 True를 반환하고 단일 LineSegment를 생성한다."""
    obj = _make([
        Waypoint(0, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
    ])
    assert obj.init_interpolator() is True


def test_two_waypoints_straight_creates_single_line_segment():
    """직선 2-waypoint: 정확히 1개의 LineSegment 세그먼트가 생성된다."""
    obj = _make([
        Waypoint(0, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
    ])
    obj.init_interpolator()
    segs = obj._interp_fcns['pos']
    assert len(segs) == 1
    assert type(segs[0]).__name__ == 'LineSegment'


def test_two_waypoints_straight_segment_to_wp_map():
    """직선 2-waypoint: _segment_to_wp_map이 [0, 1]이다."""
    obj = _make([
        Waypoint(0, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
    ])
    obj.init_interpolator()
    assert obj._segment_to_wp_map == [0, 1]


def test_two_waypoints_straight_parametric_s():
    """직선 2-waypoint: 매개변수 _s가 [0, 1]로 정규화된다."""
    obj = _make([
        Waypoint(0, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
    ])
    obj.init_interpolator()
    np.testing.assert_allclose(obj._s, [0.0, 1.0], atol=1e-9)


def test_two_waypoints_straight_total_path_length():
    """직선 2-waypoint (10 m 간격): total_path_length = 10.0."""
    obj = _make([
        Waypoint(0, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
    ])
    obj.init_interpolator()
    np.testing.assert_allclose(obj._total_path_length, 10.0, atol=1e-9)


def test_two_waypoints_straight_duration_equals_length_over_speed():
    """직선 2-waypoint: duration = path_length / mean_speed (= 10 / 1 = 10 s)."""
    obj = _make([
        Waypoint(0, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
    ])
    obj.init_interpolator()
    np.testing.assert_allclose(obj._duration, 10.0, atol=1e-9)


def test_two_waypoints_straight_start_time_is_zero():
    """직선 2-waypoint: 사전에 start_time이 설정되지 않으면 0.0으로 초기화된다."""
    obj = _make([
        Waypoint(0, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
    ])
    obj.init_interpolator()
    assert obj._start_time == 0.0


def test_two_waypoints_straight_generate_pos_endpoints():
    """직선 2-waypoint: s=0.0 → 시작점, s=1.0 → 끝점."""
    obj = _make([
        Waypoint(0, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
    ])
    obj.init_interpolator()
    np.testing.assert_allclose(obj.generate_pos(0.0), [0, 0, 0], atol=1e-9)
    np.testing.assert_allclose(obj.generate_pos(1.0), [10, 0, 0], atol=1e-9)


def test_two_waypoints_straight_generate_pos_midpoint():
    """직선 2-waypoint: s=0.5 → 중간점 (5, 0, 0)."""
    obj = _make([
        Waypoint(0, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
    ])
    obj.init_interpolator()
    np.testing.assert_allclose(obj.generate_pos(0.5), [5, 0, 0], atol=1e-9)


def test_two_waypoints_straight_heading_is_zero_auto_mode():
    """직선 2-waypoint, heading_offset=0: heading 함수는 상수 0.0을 반환한다 (auto-heading)."""
    obj = _make([
        Waypoint(0, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
    ])
    obj.init_interpolator()
    for s in [0.0, 0.5, 1.0]:
        assert obj._interp_fcns['heading'](s) == 0.0, f's={s}'


def test_two_waypoints_diagonal_total_path_length():
    """대각선 2-waypoint (3,4,0): total_path_length = 5.0 (피타고라스)."""
    obj = _make([
        Waypoint(0, 0, 0, max_forward_speed=2.0),
        Waypoint(3, 4, 0, max_forward_speed=2.0),
    ])
    obj.init_interpolator()
    np.testing.assert_allclose(obj._total_path_length, 5.0, atol=1e-9)


def test_two_waypoints_diagonal_duration_equals_length_over_speed():
    """대각선 2-waypoint: duration = 5.0 / 2.0 = 2.5 s."""
    obj = _make([
        Waypoint(0, 0, 0, max_forward_speed=2.0),
        Waypoint(3, 4, 0, max_forward_speed=2.0),
    ])
    obj.init_interpolator()
    np.testing.assert_allclose(obj._duration, 2.5, atol=1e-9)


def test_two_waypoints_diagonal_pos_endpoints():
    """대각선 2-waypoint: s=0 → (0,0,0), s=1 → (3,4,0)."""
    obj = _make([
        Waypoint(0, 0, 0, max_forward_speed=2.0),
        Waypoint(3, 4, 0, max_forward_speed=2.0),
    ])
    obj.init_interpolator()
    np.testing.assert_allclose(obj.generate_pos(0.0), [0, 0, 0], atol=1e-9)
    np.testing.assert_allclose(obj.generate_pos(1.0), [3, 4, 0], atol=1e-9)


def test_three_waypoints_L_shape_returns_true():
    """L자형 3-waypoint 경로는 True를 반환한다."""
    obj = _make([
        Waypoint(0,  0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 10, 0, max_forward_speed=1.0),
    ])
    assert obj.init_interpolator() is True


def test_three_waypoints_L_shape_segment_count_and_types():
    """L자형 3-waypoint: 세그먼트가 3개(Line + Bezier + Line)이다."""
    obj = _make([
        Waypoint(0,  0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 10, 0, max_forward_speed=1.0),
    ])
    obj.init_interpolator()
    segs = obj._interp_fcns['pos']
    assert len(segs) == 3
    assert type(segs[0]).__name__ == 'LineSegment'
    assert type(segs[1]).__name__ == 'BezierCurve'
    assert type(segs[2]).__name__ == 'LineSegment'


def test_three_waypoints_L_shape_segment_to_wp_map():
    """L자형 3-waypoint: _segment_to_wp_map이 [0, 1, 1, 2]이다."""
    obj = _make([
        Waypoint(0,  0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 10, 0, max_forward_speed=1.0),
    ])
    obj.init_interpolator()
    assert obj._segment_to_wp_map == [0, 1, 1, 2]


def test_three_waypoints_L_shape_parametric_s():
    """L자형 3-waypoint: _s가 4개 원소이고 [0, ..., 1]로 단조증가한다."""
    obj = _make([
        Waypoint(0,  0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 10, 0, max_forward_speed=1.0),
    ])
    obj.init_interpolator()
    s = obj._s
    assert len(s) == 4
    np.testing.assert_allclose(s[0], 0.0, atol=1e-9)
    np.testing.assert_allclose(s[-1], 1.0, atol=1e-9)
    # 단조증가 확인
    assert np.all(np.diff(s) > 0)
    # 분기점 오라클 동결
    np.testing.assert_allclose(s, [0.0, 0.27486745, 0.72513255, 1.0], atol=1e-6)


def test_three_waypoints_L_shape_total_path_length():
    """L자형 3-waypoint: total_path_length 오라클 동결 (Bezier 코너 포함)."""
    obj = _make([
        Waypoint(0,  0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 10, 0, max_forward_speed=1.0),
    ])
    obj.init_interpolator()
    np.testing.assert_allclose(obj._total_path_length, 18.190585757566, atol=1e-6)


def test_three_waypoints_L_shape_pos_endpoints():
    """L자형 3-waypoint: s=0 → (0,0,0), s=1 → (10,10,0)."""
    obj = _make([
        Waypoint(0,  0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 10, 0, max_forward_speed=1.0),
    ])
    obj.init_interpolator()
    np.testing.assert_allclose(obj.generate_pos(0.0), [0, 0, 0], atol=1e-9)
    np.testing.assert_allclose(obj.generate_pos(1.0), [10, 10, 0], atol=1e-9)


def test_three_waypoints_L_shape_pos_midpoint():
    """L자형 3-waypoint: s=0.5 중간점 오라클 동결 (Bezier 코너 직후)."""
    obj = _make([
        Waypoint(0,  0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 10, 0, max_forward_speed=1.0),
    ])
    obj.init_interpolator()
    np.testing.assert_allclose(
        obj.generate_pos(0.5), [8.77977902, 1.22022098, 0.0], atol=1e-6)


def test_three_waypoints_L_shape_heading_is_zero_auto_mode():
    """L자형 3-waypoint, heading_offset=0: heading 함수는 0.0을 반환한다 (auto-heading)."""
    obj = _make([
        Waypoint(0,  0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 10, 0, max_forward_speed=1.0),
    ])
    obj.init_interpolator()
    for s in [0.0, 0.5, 1.0]:
        assert obj._interp_fcns['heading'](s) == 0.0, f's={s}'


def test_three_waypoints_fixed_heading_spline_created():
    """고정 heading_offset이 있는 3-waypoint: _heading_spline이 생성된다."""
    obj = _make([
        Waypoint(0,  0, 0, max_forward_speed=1.0, heading_offset=0.1),
        Waypoint(10, 0, 0, max_forward_speed=1.0, heading_offset=0.2),
        Waypoint(10, 10, 0, max_forward_speed=1.0, heading_offset=0.3),
    ])
    obj.init_interpolator()
    assert obj._heading_spline is not None


def test_three_waypoints_fixed_heading_spline_interpolates_endpoints():
    """고정 heading_offset 3-waypoint: spline이 경계값(0.1, 0.3)을 복원한다."""
    obj = _make([
        Waypoint(0,  0, 0, max_forward_speed=1.0, heading_offset=0.1),
        Waypoint(10, 0, 0, max_forward_speed=1.0, heading_offset=0.2),
        Waypoint(10, 10, 0, max_forward_speed=1.0, heading_offset=0.3),
    ])
    obj.init_interpolator()
    np.testing.assert_allclose(obj._interp_fcns['heading'](0.0), 0.1, atol=1e-9)
    np.testing.assert_allclose(obj._interp_fcns['heading'](1.0), 0.3, atol=1e-9)


def test_three_waypoints_fixed_heading_spline_interpolates_midpoint():
    """고정 heading_offset 3-waypoint: spline 중간값 오라클 동결 (~0.2)."""
    obj = _make([
        Waypoint(0,  0, 0, max_forward_speed=1.0, heading_offset=0.1),
        Waypoint(10, 0, 0, max_forward_speed=1.0, heading_offset=0.2),
        Waypoint(10, 10, 0, max_forward_speed=1.0, heading_offset=0.3),
    ])
    obj.init_interpolator()
    np.testing.assert_allclose(obj._interp_fcns['heading'](0.5), 0.2, atol=1e-9)


def test_init_interpolator_initializes_markers_msg():
    """init_interpolator 호출 시 _markers_msg가 MarkerArray로 (재)초기화된다."""
    obj = _make([
        Waypoint(0, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
    ])
    obj.init_interpolator()
    # MarkerArray stub 인스턴스로 초기화되었는지 확인
    assert obj._markers_msg is not None
    assert hasattr(obj._markers_msg, 'markers')


def test_init_interpolator_resets_marker_id_to_zero():
    """init_interpolator 호출 시 _marker_id가 0으로 재설정된다."""
    obj = _make([
        Waypoint(0, 0, 0, max_forward_speed=1.0),
        Waypoint(10, 0, 0, max_forward_speed=1.0),
    ])
    obj._marker_id = 99  # 임의의 이전 값
    obj.init_interpolator()
    assert obj._marker_id == 0
