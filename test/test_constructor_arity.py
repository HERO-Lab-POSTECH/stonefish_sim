"""생성자 인자 정합 정적 게이트 (T1.1, P4 Phase 1, golden-blind [T1']).

rclpy 부재 환경에선 노드를 실제 import할 수 없어 construction-time TypeError를
fake-node 골든으로 못 덮는다(golden-blind). 대신 **AST 정적 분석**으로 노드가
컨트롤러 생성자에 넘기는 키워드 인자 집합이 ``__init__`` 수용 파라미터 집합의
부분집합인지 검증한다.

T1.1: ``position_controller_node.py:87``이 ``PositionController(...,
integral_limit=...)``를 넘기나 ``position_controller.py:__init__``은
``integral_limit``를 받지 않는다(``integral_safety_factor``만). → ``ros2 run``
즉시 ``TypeError``. setup.py 등록된 노드인데 기동 불가.

이 게이트는 같은 클래스 생성자에 잘못된 kwarg를 넘기는 모든 노드를 일반적으로
포착한다(H4류 재발 방지).
"""
import ast
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
_SRC = REPO_ROOT / "stonefish_control/stonefish_control/stonefish_control"


def _init_params(class_file, class_name):
    """class_file 안 class_name의 __init__ 수용 파라미터 이름 집합(self 제외)."""
    tree = ast.parse(Path(class_file).read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == "__init__":
                    args = item.args
                    names = {a.arg for a in args.args if a.arg != "self"}
                    names |= {a.arg for a in args.kwonlyargs}
                    # **kwargs를 받으면 임의 kwarg 허용
                    if args.kwarg is not None:
                        return names, True
                    return names, False
    raise AssertionError(f"{class_name}.__init__ not found in {class_file}")


def _call_kwargs(caller_file, callee_name):
    """caller_file에서 callee_name(...) 호출의 키워드 인자 이름 집합."""
    tree = ast.parse(Path(caller_file).read_text())
    kwargs = set()
    found = False
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            func = node.func
            name = func.id if isinstance(func, ast.Name) else getattr(func, "attr", None)
            if name == callee_name:
                found = True
                for kw in node.keywords:
                    if kw.arg is not None:  # **kwargs 전달이 아닌 명시 kwarg
                        kwargs.add(kw.arg)
    assert found, f"no call to {callee_name} found in {caller_file}"
    return kwargs


def test_position_controller_node_kwargs_subset_of_init():
    """position_controller_node가 넘기는 kwarg ⊆ PositionController.__init__ param.

    RED: 현재 integral_limit이 init에 없어 실패. GREEN: integral_safety_factor로 정정 후 통과.
    """
    accepted, has_var_kw = _init_params(
        _SRC / "controllers/position_controller.py", "PositionController"
    )
    passed = _call_kwargs(
        _SRC / "nodes/position_controller_node.py", "PositionController"
    )
    if has_var_kw:
        return  # **kwargs면 임의 허용
    unexpected = passed - accepted
    assert unexpected == set(), (
        f"position_controller_node passes kwargs not accepted by "
        f"PositionController.__init__: {sorted(unexpected)}"
    )


def test_hybrid_controller_kwargs_subset_of_position_init():
    """HybridController가 만드는 PositionController kwarg 정합 (회귀 가드).

    hybrid는 integral_safety_factor를 쓰므로 이미 GREEN — fix가 hybrid를 깨지 않음을 보증.
    """
    accepted, has_var_kw = _init_params(
        _SRC / "controllers/position_controller.py", "PositionController"
    )
    passed = _call_kwargs(
        _SRC / "controllers/hybrid_controller.py", "PositionController"
    )
    if has_var_kw:
        return
    unexpected = passed - accepted
    assert unexpected == set(), (
        f"hybrid_controller passes kwargs not accepted by "
        f"PositionController.__init__: {sorted(unexpected)}"
    )
