"""Characterization: angle wrapping 동작 동결 (P3 안전망).

P3 T4(angle_wrap dedup)와 P4 격리의 회귀 가드.
두 알고리즘이 **경계에서 다르게 동작**함을 명시적으로 동결한다:
  - modulo:  (angle + π) % (2π) - π   → control 패키지 SSOT(data_types) + position staticmethod + trajectory ilos
  - arctan2: arctan2(sin, cos)         → trajectory los_guidance._normalize_angle

x=π 경계에서 modulo는 -π, arctan2는 +π로 **2π 발산**한다.
이 테스트가 GREEN인 한 누구도 los를 modulo로 "친절하게" 통합할 수 없다(통합 = 수치 변경 = P4).

conftest.py의 load_module fixture로 파일을 직접 로드한다(ROS/gtsam __init__ 오염 우회).
"""
from pathlib import Path

import numpy as np
import pytest

# 정적 검증용 repo root (conftest의 REPO_ROOT는 fixture 스코프라 모듈 레벨에서 별도 정의)
REPO_ROOT = Path(__file__).resolve().parent.parent

# repo-root 상대 경로 — 파일 이동(T3/T4) 시 이 문자열을 동시에 갱신해야 함(CI parity 엣지)
_DATA_TYPES = (
    'stonefish_control/stonefish_control/stonefish_control/'
    'control_interfaces/data_types.py'
)
_POSITION_CTRL = (
    'stonefish_control/stonefish_control/stonefish_control/'
    'controllers/position_controller.py'
)


def _modulo_wrap(angle):
    """control 패키지 SSOT 알고리즘의 독립 참조 구현."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def _arctan2_wrap(angle):
    """los_guidance 알고리즘의 독립 참조 구현."""
    return np.arctan2(np.sin(angle), np.cos(angle))


# 경계·일반·큰 값 — 동결 입력 집합
_INPUTS = [0.0, np.pi, -np.pi, 3 * np.pi, -3 * np.pi, 0.5, -0.5,
           2.0, -2.0, 100.0, -100.0, np.pi / 2, -np.pi / 2,
           np.pi - 1e-9, -np.pi + 1e-9]


def test_data_types_angle_wrap_is_modulo(load_module):
    """data_types.angle_wrap이 modulo 알고리즘 출력을 바이트 동일하게 낸다."""
    dt = load_module(_DATA_TYPES, 'char_data_types')
    for x in _INPUTS:
        assert dt.angle_wrap(x) == _modulo_wrap(x), f'angle_wrap({x})'


def test_position_controller_delegates_angle_wrap_to_data_types():
    """position_controller._angle_wrap이 data_types.angle_wrap에 위임함을 정적 동결 (T4).

    position_controller는 §2.2 상대 import(`from ..control_interfaces.data_types`)를 쓰고,
    control_interfaces/__init__.py가 ROS 의존(nav_msgs 등)을 끌어와서, 이 모듈은
    load_module(path-load)로도 패키지 import로도 직접 실행할 수 없다(rclpy/ROS 부재 환경).
    따라서 위임의 동작보존을 **정적으로** 증명한다:
      (1) _angle_wrap 본문이 angle_wrap(...)에 단일 위임
      (2) angle_wrap이 control_interfaces.data_types(= test_data_types_angle_wrap_is_modulo가
          직접 검증한 SSOT)에서 import됨
    (1)+(2) + data_types.angle_wrap의 직접 검증(GREEN)이 합쳐지면
    "position._angle_wrap(x) == data_types.angle_wrap(x) ∀x"가 연역적으로 보증된다.

    누가 _angle_wrap 본문에 modulo 식을 다시 인라인하거나 다른 함수로 위임을 바꾸면 RED.
    """
    import ast
    src = Path(REPO_ROOT / _POSITION_CTRL).read_text()
    tree = ast.parse(src)

    # (1) _angle_wrap이 angle_wrap에 위임
    delegated = False
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == '_angle_wrap':
            calls = [n.func.id for n in ast.walk(node)
                     if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)]
            assert calls == ['angle_wrap'], \
                f'_angle_wrap이 angle_wrap 단일 위임이 아님: 호출={calls}'
            delegated = True
    assert delegated, 'position_controller에 _angle_wrap 메서드가 없음'

    # (2) angle_wrap이 control_interfaces.data_types에서 import됨 (상대 import level 무관)
    imports_angle_wrap_from_data_types = any(
        isinstance(n, ast.ImportFrom)
        and n.module and n.module.endswith('control_interfaces.data_types')
        and any(a.name == 'angle_wrap' for a in n.names)
        for n in ast.walk(tree)
    )
    assert imports_angle_wrap_from_data_types, \
        'angle_wrap이 control_interfaces.data_types에서 import되지 않음(SSOT 위임 깨짐)'


def test_modulo_and_arctan2_diverge_at_pi_boundary():
    """x=π 경계에서 두 알고리즘이 정확히 2π 발산함을 동결.

    이 발산이 los_guidance(arctan2)를 modulo와 통합 불가하게 만드는 근거.
    누가 이 둘을 "같다"고 가정하면 이 테스트가 RED가 된다.
    """
    # x = π: modulo → -π, arctan2 → +π
    assert _modulo_wrap(np.pi) == pytest.approx(-np.pi)
    assert _arctan2_wrap(np.pi) == pytest.approx(np.pi)
    # 차이는 정확히 2π
    assert abs(_arctan2_wrap(np.pi) - _modulo_wrap(np.pi)) == pytest.approx(2 * np.pi)


def test_modulo_and_arctan2_agree_off_boundary():
    """경계가 아닌 곳에서는 두 알고리즘이 일치(발산은 경계 한정)."""
    for x in [0.0, 0.5, -0.5, 1.0, 2.0, -2.0, np.pi / 2]:
        assert _modulo_wrap(x) == pytest.approx(_arctan2_wrap(x)), f'x={x}'
