#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""ILOS 축소 + cascade 모드 전환의 정적 게이트 (AST/소스 검사).

rclpy 미설치 환경에서 동작보존·회귀를 정적으로 고정한다. 설계 SSOT §8C.

핵심 회귀(적분기 갱신 부활)는 AST로 구조 검사하여 주석 처리·리네임 우회를
막고, 산식/모드 문자열은 소스 문자열로 고정한다(git 원본 패턴과 일치 확인됨).
"""
import ast
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

_ILOS = REPO_ROOT / ('stonefish_control/stonefish_trajectory_manager/'
                     'stonefish_trajectory_manager/path_following/ilos_guidance.py')
_PF_NODE = REPO_ROOT / ('stonefish_control/stonefish_trajectory_manager/'
                        'stonefish_trajectory_manager/nodes/path_following_node.py')
_HYBRID = REPO_ROOT / ('stonefish_control/stonefish_control/stonefish_control/'
                       'controllers/hybrid_controller.py')
_HYBRID_NODE = REPO_ROOT / ('stonefish_control/stonefish_control/stonefish_control/'
                            'nodes/hybrid_controller_node.py')
_CASCADE = REPO_ROOT / ('stonefish_control/stonefish_control/stonefish_control/'
                        'controllers/cascade_controller.py')


def _self_attr_assignments(tree):
    """self.<attr> 대입(=, +=, -= 등)을 (attr, value_src) 쌍으로 수집.

    초기화(=0.0)와 실제 갱신(+= / decay 재대입)을 AST로 구분하기 위해,
    대입 우변을 unparse한 소스를 함께 돌려준다. 주석 처리된 코드는 AST에
    노드로 남지 않으므로 문자열 검사와 달리 회귀로 오판하지 않는다.
    """
    out = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            targets, value = node.targets, node.value
        elif isinstance(node, ast.AugAssign):
            targets, value = [node.target], node.value
        else:
            continue
        for t in targets:
            if (isinstance(t, ast.Attribute) and isinstance(t.value, ast.Name)
                    and t.value.id == 'self'):
                kind = '+=' if isinstance(node, ast.AugAssign) else '='
                out.append((t.attr, kind, ast.unparse(value)))
    return out


def _ilos_tree():
    return ast.parse(_ILOS.read_text())


def test_ilos_integral_ey_not_updated():
    """_integral_ey / _integral_ey_lateral은 초기화(=0.0) 외 갱신이 없다 (AST).

    회귀(leaky-integrator 부활: `self._integral_ey = decay * self._integral_ey ...`
    또는 `self._integral_ey_lateral += e_y * dt`)는 우변이 '0.0'이 아닌 대입으로
    나타난다. 주석 처리·리네임으로 우회되지 않도록 구조로 검사한다.
    """
    for attr, kind, value in _self_attr_assignments(_ilos_tree()):
        if attr in ('_integral_ey', '_integral_ey_lateral'):
            assert kind == '=' and value.strip() == '0.0', (
                f"ILOS cross-track 적분 갱신 잔존: self.{attr} {kind} {value} "
                f"(초기화 =0.0만 허용 — cross-track 채널은 cascade outer로 이관됨)")


def test_ilos_integral_ez_preserved():
    """_integral_ez(depth)는 유지 — += 갱신이 존재해야 한다 (AST, F10·설계 A 보정)."""
    has_update = any(
        attr == '_integral_ez' and kind == '+='
        for attr, kind, _ in _self_attr_assignments(_ilos_tree()))
    assert has_update, \
        'depth 적분(_integral_ez)의 += 갱신이 사라짐 — depth는 유지해야 함'


def test_ilos_sway_pid_removed():
    """sway PID 산식(-lateral_gain * e_y ...)이 제거됨 (소스 문자열).

    [P6] sway PID feedback 부활 금지 — cross-track feedback은 cascade 단독(이중보정 방지).
    """
    src = _ILOS.read_text()
    assert '-self._lateral_gain * e_y' not in src and \
           '- self._lateral_gain * e_y' not in src, \
        'sway PID 산식이 잔존 (v_lateral=0.0으로 대체 대상)'


def test_ilos_heading_arctan_removed():
    """heading cross-track arctan 항(np.arctan(-e_y/...))이 제거됨 (소스 문자열)."""
    src = _ILOS.read_text()
    assert 'np.arctan(-e_y' not in src, \
        'ILOS heading의 cross-track arctan 항이 잔존 (chi_d=chi_p로 대체 대상)'


def test_path_following_node_no_hybrid_string():
    """publisher 경로에서 'hybrid' 모드 문자열이 사라짐(드롭 방지)."""
    src = _PF_NODE.read_text()
    assert "'hybrid'" not in src and '"hybrid"' not in src, \
        "path_following_node에 'hybrid' 문자열 잔존 (subscriber가 드롭함)"


def test_hybrid_set_mode_accepts_cascade():
    """HybridController.set_mode 화이트리스트에 'cascade'가 포함됨."""
    src = _HYBRID.read_text()
    assert "'cascade'" in src, "HybridController가 'cascade'를 수용하지 않음"


def test_hybrid_node_mode_callback_accepts_cascade():
    """hybrid_controller_node mode_callback 화이트리스트에 'cascade'가 포함됨(F5b)."""
    src = _HYBRID_NODE.read_text()
    assert "'cascade'" in src, "hybrid_controller_node가 'cascade'를 수용하지 않음"


def test_ilos_sway_feedforward_present():
    """[P6] 곡률 sway feedforward(_sway_ff_gain · v² · κ)가 _compute_body_velocities 산식에 실제 참조된다 (AST).

    문자열 검색이 아닌 AST로 확인하여 생성자 저장·주석만 남기고 산식에서
    제거되는 false-pass를 막는다.
    """
    tree = _ilos_tree()

    # _compute_body_velocities 함수 노드를 찾는다.
    body_vel_func = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == '_compute_body_velocities':
            body_vel_func = node
            break
    assert body_vel_func is not None, \
        '_compute_body_velocities 함수가 ilos_guidance.py에서 사라짐'

    # 함수 본문에서 self._sway_ff_gain 참조(Attribute)를 확인한다.
    def _has_self_attr(func_node, attr_name):
        for n in ast.walk(func_node):
            if (isinstance(n, ast.Attribute)
                    and n.attr == attr_name
                    and isinstance(n.value, ast.Name)
                    and n.value.id == 'self'):
                return True
        return False

    assert _has_self_attr(body_vel_func, '_sway_ff_gain'), \
        '_compute_body_velocities 산식에서 _sway_ff_gain을 소비하지 않음 (P6 회귀)'
    assert _has_self_attr(body_vel_func, '_signed_curvature_filtered'), \
        '_compute_body_velocities 산식에서 _signed_curvature_filtered를 소비하지 않음 (P6 회귀)'


def test_rd_uses_signed_curvature():
    """[결함 A] _compute_body_velocities의 r_d 산식이 _signed_curvature_filtered를
    참조한다 (부호 없는 _current_curvature 회귀 차단, AST).

    r_d 대입 우변에 _signed_curvature_filtered Attribute가 있어야 한다.
    """
    tree = _ilos_tree()
    body_vel_func = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == '_compute_body_velocities':
            body_vel_func = node
            break
    assert body_vel_func is not None, '_compute_body_velocities 사라짐'

    # r_d = ... 대입을 찾아 우변에 _signed_curvature_filtered 참조 확인
    rd_assigns = []
    for node in ast.walk(body_vel_func):
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id == 'r_d':
                    rd_assigns.append(node.value)
    assert rd_assigns, 'r_d 대입이 _compute_body_velocities에 없음'

    def _refs_attr(value_node, attr):
        for n in ast.walk(value_node):
            if (isinstance(n, ast.Attribute) and n.attr == attr
                    and isinstance(n.value, ast.Name) and n.value.id == 'self'):
                return True
        return False

    # r_d 대입 중 _signed_curvature_filtered를 참조하는 대입이 적어도 하나 존재해야 한다.
    # r_d=0.0 같은 초기화 대입은 무시한다.
    assert any(_refs_attr(v, '_signed_curvature_filtered') for v in rd_assigns), \
        ('r_d 산식이 _signed_curvature_filtered를 참조하지 않음 — '
         '부호 없는 _current_curvature 회귀 (결함 A)')


def test_cascade_outer_sway_yaw_gated():
    """[결함 C] cascade compute_control이 e_yaw로 sway 위치명령을 게이트한다 (AST).

    compute_control 본문에 np.cos(e_yaw) 형태 Call이 존재해야 한다.
    """
    tree = ast.parse(_CASCADE.read_text())
    func = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == 'compute_control':
            func = node
            break
    assert func is not None, 'compute_control 사라짐'

    # np.cos(e_yaw) 형태의 Call이 본문에 있는지
    has_cos_eyaw = False
    for n in ast.walk(func):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == 'cos'):
            for arg in n.args:
                if isinstance(arg, ast.Name) and arg.id == 'e_yaw':
                    has_cos_eyaw = True
    assert has_cos_eyaw, \
        'compute_control에 np.cos(e_yaw) 게이트 없음 — 결함 C 회귀'
