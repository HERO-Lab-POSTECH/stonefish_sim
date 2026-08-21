#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""position_controller_node의 dt 클램프 정적 게이트 (AST 검사).

rclpy 미설치 환경에서 노드를 실행할 수 없으므로 회귀 방지 계약을 AST 구조로
고정한다(주석 처리·리네임 우회 차단). 방식은 `test_cascade_static_gate.py`와 같다.
"""
import ast
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

_POS_NODE = REPO_ROOT / ('stonefish_control/stonefish_control/stonefish_control/'
                         'nodes/position_controller_node.py')
_HYBRID_NODE = REPO_ROOT / ('stonefish_control/stonefish_control/stonefish_control/'
                            'nodes/hybrid_controller_node.py')


def _control_loop(path):
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.FunctionDef) and node.name == 'control_loop':
            return node
    return None


def _dt_clamp_bounds(func):
    """control_loop 안의 `dt = max(lo, min(dt, hi))` 대입에서 (lo, hi)를 뽑는다.

    Returns:
        tuple | None: 클램프 대입이 없으면 None.
    """
    for node in ast.walk(func):
        if not isinstance(node, ast.Assign):
            continue
        if not any(isinstance(t, ast.Name) and t.id == 'dt' for t in node.targets):
            continue
        v = node.value
        if not (isinstance(v, ast.Call) and isinstance(v.func, ast.Name)
                and v.func.id == 'max' and len(v.args) == 2):
            continue
        lo, inner = v.args
        if not (isinstance(inner, ast.Call) and isinstance(inner.func, ast.Name)
                and inner.func.id == 'min' and len(inner.args) == 2):
            continue
        if not (isinstance(inner.args[0], ast.Name) and inner.args[0].id == 'dt'):
            continue
        return ast.unparse(lo), ast.unparse(inner.args[1])
    return None


def test_position_controller_clamps_dt():
    """control_loop이 dt를 max(lo, min(dt, hi))로 클램프한다.

    클램프가 없으면 일시정지·부하로 벌어진 wall-clock 간격이 그대로
    PositionController의 사다리꼴 적분에 들어가 적분항이 폭주한다.
    """
    func = _control_loop(_POS_NODE)
    assert func is not None, 'position_controller_node.control_loop 사라짐'
    assert _dt_clamp_bounds(func) is not None, (
        'control_loop이 dt를 클램프하지 않음 — max(lo, min(dt, hi)) 가드 부재. '
        '무클램프 dt가 사다리꼴 적분으로 전달되면 적분항 폭주')


def test_position_controller_dt_clamp_matches_hybrid():
    """클램프 경계가 형제 hybrid_controller_node와 동일하다 (drift 차단)."""
    pos = _dt_clamp_bounds(_control_loop(_POS_NODE))
    hybrid = _dt_clamp_bounds(_control_loop(_HYBRID_NODE))
    assert hybrid is not None, \
        'hybrid_controller_node의 dt 클램프가 사라짐 (SSOT 패턴 소실)'
    assert pos == hybrid, \
        f'dt 클램프 경계가 형제와 불일치: position={pos}, hybrid={hybrid}'
