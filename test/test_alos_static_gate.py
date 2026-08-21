#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""ALOS yaw rate feedforward의 정적 게이트 (AST 검사).

부모 ILOS의 P7 결함 A 수정(r_d에 부호 있는 곡률)을 ALOS에서도 고정한다.
`test_cascade_static_gate.py::test_rd_uses_signed_curvature`의 ALOS 대응물.
"""
import ast
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

_ALOS = REPO_ROOT / ('stonefish_control/stonefish_trajectory_manager/'
                     'stonefish_trajectory_manager/path_following/alos_guidance.py')


def _compute_guidance():
    for node in ast.walk(ast.parse(_ALOS.read_text())):
        if isinstance(node, ast.FunctionDef) and node.name == 'compute_guidance':
            return node
    return None


def _refs_self_attr(value_node, attr):
    for n in ast.walk(value_node):
        if (isinstance(n, ast.Attribute) and n.attr == attr
                and isinstance(n.value, ast.Name) and n.value.id == 'self'):
            return True
    return False


def test_alos_rd_uses_signed_curvature():
    """r_d 산식이 _signed_curvature_filtered를 참조한다 (부호 없는 회귀 차단).

    부호 없는 `_current_curvature`(max-preview 크기)는 회전 방향을 모르므로
    좌회전에서 yaw rate feedforward가 반대로 나간다.
    """
    func = _compute_guidance()
    assert func is not None, 'alos_guidance.compute_guidance 사라짐'

    rd_assigns = [node.value for node in ast.walk(func)
                  if isinstance(node, ast.Assign)
                  and any(isinstance(t, ast.Name) and t.id == 'r_d'
                          for t in node.targets)]
    assert rd_assigns, 'r_d 대입이 compute_guidance에 없음'
    # r_d = 0.0 같은 초기화 대입은 무시하고, 최소 하나가 signed를 참조하면 통과.
    assert any(_refs_self_attr(v, '_signed_curvature_filtered') for v in rd_assigns), \
        ('ALOS r_d 산식이 _signed_curvature_filtered를 참조하지 않음 — '
         '부호 없는 _current_curvature 회귀 (좌회전 FF 역방향)')


def test_alos_signed_curvature_filter_updated():
    """_signed_curvature_filtered가 compute_guidance에서 갱신된다.

    r_d가 이 상태를 소비하므로, 갱신이 사라지면 필터가 0에 고정돼 FF가 죽는다.
    """
    func = _compute_guidance()
    updated = False
    for node in ast.walk(func):
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if (isinstance(t, ast.Attribute)
                        and t.attr == '_signed_curvature_filtered'
                        and isinstance(t.value, ast.Name) and t.value.id == 'self'):
                    updated = True
    assert updated, \
        '_signed_curvature_filtered 갱신이 없음 — r_d feedforward가 0에 고정됨'


def test_alos_velocity_profile_keeps_unsigned_curvature():
    """속도 프로파일(_compute_speed)은 부호 없는 max-preview 곡률을 유지한다.

    부호 있는 곡률을 속도 감속에 넣으면 좌회전에서 감속이 사라진다 —
    r_d만 signed로 바꾼다.
    """
    func = _compute_guidance()
    speed_calls = [n for n in ast.walk(func)
                   if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                   and n.func.attr == '_compute_speed']
    assert speed_calls, 'compute_guidance에 _compute_speed 호출이 없음'
    assert all(_refs_self_attr(c, '_current_curvature') for c in speed_calls), \
        ('_compute_speed가 _current_curvature(부호 없는 max-preview)를 '
         '소비하지 않음 — 속도 프로파일 회귀')
