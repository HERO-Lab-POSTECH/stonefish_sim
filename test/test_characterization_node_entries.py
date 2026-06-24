"""Characterization: console_scripts 엔트리·노드 정적 불변식 동결 (P3 안전망).

T2(dead 삭제)·T3(노드 이동 + import 교정)의 회귀 가드.
rclpy 부재 환경이라 모듈 실제 import가 불가능 → static_import_gate의 정적 분석으로 동결.

architect 적대 검증(2026-06-23) 권고 반영:
  - dead-state pin은 import 문자열이 아니라 **resolved 타겟 부재**로 (rename 견딤, 부활 시 RED)
  - py_compile + resolved-import-target-set diff로 절대→상대 변환의 off-by-one 포착
  - __init__.py eager node-import 금지 게이트

⚠️ 한계: 정적 게이트는 경로/엔트리/타겟집합 불변만 증명, 런타임 심볼 binding은 P4(colcon+ros2).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import static_import_gate as g  # noqa: E402

CTRL = 'stonefish_control/stonefish_control/stonefish_control'
THR = 'stonefish_control/stonefish_thruster_manager/stonefish_thruster_manager'
TRAJ = 'stonefish_control/stonefish_trajectory_manager/stonefish_trajectory_manager'

# 현재(P3 실행 전) console_scripts 엔트리 — T2/T3로 변하는 우변은 여기서 갱신
_CONTROL_ENTRIES = {
    'hybrid_controller_node': 'stonefish_control.nodes.hybrid_controller_node:main',
    'position_controller_node': 'stonefish_control.nodes.position_controller_node:main',
    # velocity/unified dead 컨트롤러는 P4 T2.2에서 삭제됨(test_g2_dead_controllers_deleted가 동결).
}
_THRUSTER_ENTRY = {
    # T3에서 thruster_allocator → nodes/thruster_allocator_node 로 rename(좌변 thruster_allocator 동결)
    'thruster_allocator': 'stonefish_thruster_manager.nodes.thruster_allocator_node:main',
}


def test_g1_control_entry_chain_resolves():
    """G1: control console_scripts 각 엔트리 우변이 파일로 해석되고 top-level main 보유."""
    for name, rhs in _CONTROL_ENTRIES.items():
        f, func = g.resolve_entry(rhs)
        assert f is not None, f'{name}: 모듈경로 {rhs}가 파일로 해석 안 됨'
        assert func == 'main', f'{name}: 엔트리 함수가 main이 아님 ({func})'
        assert g.top_level_main_symbol(f), f'{name}: {f}에 top-level main 없음'


def test_g1_thruster_entry_chain_resolves():
    """G1: thruster console_scripts 엔트리 해석 + main 보유 (T3 rename 전 기준점)."""
    for name, rhs in _THRUSTER_ENTRY.items():
        f, func = g.resolve_entry(rhs)
        assert f is not None, f'{name}: {rhs} 해석 실패'
        assert g.top_level_main_symbol(f), f'{name}: top-level main 없음'


def test_g2_dead_controllers_deleted():
    """G2: dead 컨트롤러(velocity/unified)가 P4 T2.2에서 삭제됐음을 동결.

    velocity_controller_node(broken: pid_4dof 부재)와 unified_controller(+node, orphan)는
    상속자·console_scripts·launch 참조 0으로 dead 판정(docs/LIVENESS_AUDIT.md) 후 삭제됐다.
    누가 다시 만들면(= 부활 = 토픽그래프 변경) 이 테스트가 RED.
    """
    for rel in ('controllers/velocity_controller_node.py',
                'controllers/unified_controller.py',
                'controllers/unified_controller_node.py'):
        assert not (g.REPO_ROOT / CTRL / rel).exists(), \
            f'{rel}가 다시 존재함 — dead 컨트롤러 부활(P4 사건)'


def test_g2_dynamics_loader_pid4dof_is_docstring_only():
    """G2 동반: dynamics_loader의 PID4DOF 참조가 live code가 아니라 docstring 예시임을 동결.

    architect가 지적한 dangling 참조. live import가 되면(= 실제 import문 추가) RED.
    """
    src = Path(g.REPO_ROOT / f'{CTRL}/control_interfaces/dynamics_loader.py').read_text()
    # PID4DOF를 실제로 import하는 문장이 없어야 함 (docstring 안의 예시 텍스트만 허용)
    import ast
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            for a in node.names:
                assert a.name != 'PID4DOF', 'dynamics_loader가 PID4DOF를 실제 import함 — dead 모듈 의존'


def test_g3_node_modules_have_no_toplevel_dynamic_refs():
    """G3: 이동 대상 노드들의 모듈 top-level에 동적 topic/path 참조 없음.

    이 불변식이 참일 때만 "파일 이동 = 토픽그래프 불변"이 per-file로 성립한다(architect).
    """
    nodes = [
        f'{CTRL}/nodes/position_controller_node.py',
        f'{CTRL}/nodes/hybrid_controller_node.py',
        f'{THR}/nodes/thruster_allocator_node.py',
    ]
    for rel in nodes:
        offenders = g.module_toplevel_dynamic_refs(rel)
        assert offenders == [], f'{rel}: top-level 동적 참조 발견 {offenders}'


def test_g4_control_nodes_init_is_inert():
    """G4: stonefish_control nodes/__init__.py가 노드를 eager import하지 않음(inert)."""
    assert not g.init_eager_imports_node(f'{CTRL}/nodes/__init__.py'), \
        'control nodes/__init__.py가 노드를 eager import함 — import-time rclpy 부작용'


def test_g4_trajectory_nodes_init_eager_is_known_p4():
    """G4 known-deviation: trajectory nodes/__init__.py는 현재 eager import한다(P4_FLAGS 기록).

    이건 동작보존 중립이 아닌 결함이나 P3에서 손대지 않기로 결정(import-time 동작 변경=P4).
    상태를 동결 — P3가 실수로 이걸 바꾸면(고치면) RED로 잡아 P4 격리를 강제.
    """
    assert g.init_eager_imports_node(f'{TRAJ}/nodes/__init__.py'), \
        'trajectory nodes/__init__.py가 더는 eager가 아님 — P3에서 P4 사항을 건드림'


def test_g5_node_modules_py_compile():
    """G5: 이동 대상 노드들이 py_compile로 바이트컴파일됨(rclpy 실행 없이 문법 검증)."""
    nodes = [
        f'{CTRL}/nodes/position_controller_node.py',
        f'{CTRL}/nodes/hybrid_controller_node.py',
        f'{THR}/nodes/thruster_allocator_node.py',
    ]
    for rel in nodes:
        assert g.py_compiles(rel), f'{rel}: py_compile 실패'


def test_g5_hybrid_node_repo_import_targets_frozen():
    """G5 binding-proxy: hybrid 노드의 repo-내부 import 타겟 집합을 동결.

    T3 이동/변환 후 이 집합이 동일해야 동작보존(off-by-one level이면 집합이 달라져 RED).
    절대→상대 변환 시에도 resolved 완전수식 타겟은 불변이어야 한다.
    """
    targets = g.repo_import_targets(f'{CTRL}/nodes/hybrid_controller_node.py')
    expected = {
        ('REPO', 'stonefish_control.control_interfaces', ('DynamicsLoader',)),
        ('REPO', 'stonefish_control.controllers.hybrid_controller', ('HybridController',)),
    }
    assert targets == expected, f'hybrid 노드 import 타겟 변동: {targets ^ expected}'
