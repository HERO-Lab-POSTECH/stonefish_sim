"""control_interfaces 패키지 import-smoke 게이트 (T0.7, P4 Phase 0.7).

control_interfaces 패키지는 rclpy 비의존 leaf다 — LIVE 심볼 ``DynamicsLoader``
(``dynamics_loader.py``: numpy/typing만 의존)만 끌면 이 컨테이너(rclpy/nav_msgs 부재)
에서도 import 가능해야 한다. 그러나 현재(eager) ``__init__.py``는 dead 3클래스
(``vehicle``/``dp_controller_base``/``dp_pid_controller_base``)를 eager import해서
``vehicle.py:21 from nav_msgs.msg import Odometry``(hard) 및
``dp_controller_base.py:20 import rclpy``(hard)를 import-time에 끌어와 크래시한다.

이 테스트는 T0.7 lazy 전환의 GREEN observable이다:
  - ``import stonefish_control.control_interfaces`` 가 성공한다(현재는 ModuleNotFoundError).
  - import 후 ``sys.modules`` 에 ``nav_msgs``·``casadi``·``rclpy`` 가 **모두 부재**하다
    (= dead chain 미견인 *관측*, 단순 "import 됨"이 아님 — vacuous-pass 방지).
  - LIVE 심볼 ``DynamicsLoader`` 는 여전히 패키지에서 접근 가능하다.

⚠️ ``static_import_gate.py`` doc과의 reconcile: 거기서 "모듈 실제 로드 불가"라 한 건
*rclpy 의존 노드 모듈* 얘기다. control_interfaces 패키지는 rclpy 비의존이라 실제 로드
가능 — 영역이 분리됨(노드 전체 로드 = AST 정적, 이 leaf 패키지 = 실제 import-smoke).
"""
import importlib
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
_PKG_SRC = REPO_ROOT / "stonefish_control/stonefish_control"

# dead chain이 견인하는 모듈(부재해야 GREEN). casadi는 vehicle.py의 try-import라
# 부재 환경에선 어차피 안 잡히지만, dead chain 미견인 시 vehicle.py 자체가 로드 안 됨.
_DEAD_CHAIN_MODULES = ("nav_msgs", "casadi", "rclpy")


@pytest.fixture
def import_control_interfaces():
    """control_interfaces 패키지를 src 루트에서 실제 import. 테스트 후 정리.

    ★격리: 다른 characterization 테스트가 ``sys.modules``에 ``nav_msgs`` stub을
    주입하고 정리하지 않으므로(test_characterization_vehicle_params.py:38-43),
    이 fixture는 dead chain 모듈을 import 직전에 **제거**한 뒤 상태를 저장하고
    teardown에서 복원한다. 그러면 "import가 *새로* 견인한 모듈"만 관측되어
    테스트 순서·외부 stub 오염에 무관해진다.
    """
    added_path = str(_PKG_SRC)
    sys.path.insert(0, added_path)
    # 이전 테스트가 남긴 부분 로드 상태 제거
    for name in list(sys.modules):
        if name.startswith("stonefish_control"):
            sys.modules.pop(name, None)
    # dead chain 모듈을 저장 후 제거 — import-time에 *새로* 견인되는지만 측정
    saved = {}
    for m in _DEAD_CHAIN_MODULES:
        for name in list(sys.modules):
            if name == m or name.startswith(m + "."):
                saved[name] = sys.modules.pop(name)
    try:
        yield lambda: importlib.import_module("stonefish_control.control_interfaces")
    finally:
        for name in list(sys.modules):
            if name.startswith("stonefish_control"):
                sys.modules.pop(name, None)
        # 외부 stub 원상복구(다른 테스트가 기대하는 상태 보존)
        for name, mod in saved.items():
            sys.modules[name] = mod
        try:
            sys.path.remove(added_path)
        except ValueError:
            pass


def test_control_interfaces_imports_without_ros(import_control_interfaces):
    """패키지 import가 성공한다 (현재 eager __init__은 nav_msgs/rclpy hard-import로 크래시)."""
    mod = import_control_interfaces()
    assert mod is not None


def test_dead_chain_modules_not_pulled(import_control_interfaces):
    """import 후 dead chain(nav_msgs/casadi/rclpy)이 sys.modules에 부재함을 *관측*한다."""
    import_control_interfaces()
    pulled = [m for m in _DEAD_CHAIN_MODULES if m in sys.modules]
    assert pulled == [], f"dead chain modules were eagerly pulled: {pulled}"


def test_dynamics_loader_symbol_still_accessible(import_control_interfaces):
    """LIVE 심볼 DynamicsLoader는 lazy 전환 후에도 패키지에서 접근 가능해야 한다."""
    mod = import_control_interfaces()
    assert hasattr(mod, "DynamicsLoader")
