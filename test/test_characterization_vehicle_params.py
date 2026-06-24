"""Characterization: Vehicle.__init__ 파라미터 로딩 동작 동결 (P3 T5 안전망).

T5(VehicleParams god-class 추출)의 회귀 가드. 추출 전 현재 동작을 골든 마스터로 동결,
추출 후 동일함을 단언한다.

architect 적대 검증(2026-06-23)으로 확립한 방법 — **동적 fake-node characterization**:
이 환경엔 rclpy/nav_msgs가 없지만, vehicle.py는 rclpy를 전혀 안 쓰고(node를 받기만 함)
Odometry는 __init__ 밖에서만 type-ref로 쓰이므로, (1) nav_msgs.msg.Odometry stub +
(2) 합성 부모 패키지로 `from ._log` 해결 + (3) recording fake node를 주입하면 vehicle.py를
로드해 __init__를 실제 실행할 수 있다. fake node가 declare/get/has 호출 순서와 ValueError
raise 타이밍을 **직접 관찰**한다(수치 추정 없음 — T4와 달리 stub이 측정 도구 자체).

골든 마스터(현재 코드에서 측정):
  - 36-call ordered trace (declare/get/has 순서)
  - 8개 ValueError의 (메시지, raise 직전 호출 횟수)

추출 후 VehicleParamsLoader가 이 순서·타이밍을 바이트 동일하게 보존해야 GREEN.
"""
import sys
import types
import importlib.util
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
_VEHICLE = (REPO_ROOT / 'stonefish_control/stonefish_control/stonefish_control/'
            'control_interfaces/vehicle.py')


def _load_vehicle_module():
    """nav_msgs stub + 합성 부모 패키지로 vehicle.py를 로드해 Vehicle 클래스 반환.

    rclpy/nav_msgs 부재 환경에서 vehicle.py를 실제 import해 __init__를 실행 가능하게 한다.
    매번 깨끗한 모듈명으로 로드(테스트 격리).
    """
    # nav_msgs.msg.Odometry stub (vehicle.py:20, __init__ 밖에서만 type-ref)
    if 'nav_msgs' not in sys.modules:
        nav = types.ModuleType('nav_msgs')
        nav.msg = types.ModuleType('nav_msgs.msg')
        nav.msg.Odometry = type('Odometry', (), {})
        sys.modules['nav_msgs'] = nav
        sys.modules['nav_msgs.msg'] = nav.msg

    # 합성 부모 패키지 + stub _log (vehicle.py:24 `from ._log import get_logger`)
    pkg_name = '_char_ci_pkg'
    pkg = types.ModuleType(pkg_name)
    pkg.__path__ = [str(_VEHICLE.parent)]
    sys.modules[pkg_name] = pkg
    log = types.ModuleType(f'{pkg_name}._log')
    log.get_logger = lambda: None
    sys.modules[f'{pkg_name}._log'] = log

    spec = importlib.util.spec_from_file_location(f'{pkg_name}.vehicle', str(_VEHICLE))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[f'{pkg_name}.vehicle'] = mod
    spec.loader.exec_module(mod)
    return mod.Vehicle


class _FakeNode:
    """파라미터 접근 호출 순서를 기록하는 fake ROS2 Node.

    architect 권고대로 rclpy 의미 2개 모방:
      - 중복 declare → raise (ParameterAlreadyDeclaredException 모방)
      - 미선언 get → raise (ParameterNotDeclaredException 모방)
    이로써 추출이 실수로 중복 declare하면 silent GREEN이 아니라 RED로 잡힌다.
    """

    def __init__(self, params):
        self._declared = {}
        self._defaults = params
        self.trace = []

    def get_logger(self):
        class _L:
            def __getattr__(self, n):
                return lambda *a, **k: None
        return _L()

    def get_namespace(self):
        return '/test'

    def declare_parameter(self, name, default):
        self.trace.append(('declare', name))
        if name in self._declared:
            raise RuntimeError(f'Parameter already declared: {name}')
        self._declared[name] = self._defaults.get(name, default)

    def get_parameter(self, name):
        self.trace.append(('get', name))
        if name not in self._declared:
            raise RuntimeError(f'Parameter not declared: {name}')
        return types.SimpleNamespace(value=self._declared[name])

    def has_parameter(self, name):
        self.trace.append(('has', name))
        return name in self._declared


# 유효 설정 — 성공적인 full __init__
_VALID = dict(mass=10.0, cog=[0.0, 0.0, 0.0], cob=[0.0, 0.0, 0.0],
              volume=0.01, density=1028.0, height=1.0, length=1.0, width=1.0)

# 골든 마스터: 추출 전 현재 코드에서 측정한 36-call ordered trace
_GOLDEN_TRACE = [
    ('declare', 'mass'), ('get', 'mass'),
    ('declare', 'inertial.ixx'), ('declare', 'inertial.iyy'), ('declare', 'inertial.izz'),
    ('declare', 'inertial.ixy'), ('declare', 'inertial.ixz'), ('declare', 'inertial.iyz'),
    ('has', 'inertial.ixx'), ('get', 'inertial.ixx'),
    ('has', 'inertial.iyy'), ('get', 'inertial.iyy'),
    ('has', 'inertial.izz'), ('get', 'inertial.izz'),
    ('has', 'inertial.ixy'), ('get', 'inertial.ixy'),
    ('has', 'inertial.ixz'), ('get', 'inertial.ixz'),
    ('has', 'inertial.iyz'), ('get', 'inertial.iyz'),
    ('declare', 'cog'), ('get', 'cog'),
    ('declare', 'cob'), ('get', 'cob'),
    ('declare', 'base_link'), ('get', 'base_link'),
    ('declare', 'volume'), ('get', 'volume'),
    ('declare', 'density'), ('get', 'density'),
    ('declare', 'height'), ('declare', 'length'), ('declare', 'width'),
    ('get', 'height'), ('get', 'length'), ('get', 'width'),
]

# 골든 마스터: 8개 ValueError의 (메시지, raise 직전 호출 횟수)
_GOLDEN_RAISES = [
    ('mass<=0', 'Mass has to be positive', 2, dict(_VALID, mass=0.0)),
    ('cog!=3', 'Invalid center of gravity vector', 22, dict(_VALID, cog=[0.0, 0.0])),
    ('cob!=3', 'Invalid center of buoyancy vector', 24, dict(_VALID, cob=[0.0, 0.0])),
    ('volume<=0', 'Invalid volume', 28, dict(_VALID, volume=0.0)),
    ('density<=0', 'Invalid fluid density', 30, dict(_VALID, density=0.0)),
    ('height<=0', 'Invalid height', 34, dict(_VALID, height=0.0)),
    ('length<=0', 'Invalid length', 35, dict(_VALID, length=0.0)),
    ('width<=0', 'Invalid width', 36, dict(_VALID, width=0.0)),
]


def test_vehicle_init_param_call_order_frozen():
    """Vehicle.__init__의 36-call 파라미터 접근 순서를 골든 마스터로 동결.

    추출(T5) 후에도 VehicleParamsLoader가 이 순서를 바이트 동일하게 보존해야 GREEN.
    순서가 재배치되면(god-class 추출의 전형적 실수) RED.
    """
    Vehicle = _load_vehicle_module()
    node = _FakeNode(_VALID)
    Vehicle(node=node)
    assert node.trace == _GOLDEN_TRACE, (
        f'파라미터 접근 순서 변동:\n'
        f'  기대 {len(_GOLDEN_TRACE)}, 실제 {len(node.trace)}\n'
        f'  diff at: {next((i for i, (a, b) in enumerate(zip(_GOLDEN_TRACE, node.trace)) if a != b), None)}'
    )


def test_vehicle_init_valueerror_timing_frozen():
    """8개 invalid 입력 각각의 ValueError 메시지 + raise 타이밍을 동결.

    추출 후 raise가 param registry 변이 대비 같은 시점에 일어나야 한다.
    """
    Vehicle = _load_vehicle_module()
    for label, msg, expected_calls, cfg in _GOLDEN_RAISES:
        node = _FakeNode(cfg)
        with pytest.raises(ValueError) as exc:
            Vehicle(node=node)
        assert str(exc.value) == msg, f'{label}: 메시지 변동 {str(exc.value)!r} != {msg!r}'
        assert len(node.trace) == expected_calls, (
            f'{label}: raise 타이밍 변동 — {len(node.trace)} calls (기대 {expected_calls})')


def test_vehicle_init_attribute_value_mapping():
    """추출 후 Vehicle 인스턴스의 10개 속성이 올바른 파라미터 값으로 매핑됨을 동결.

    골든 trace는 호출 *순서*만 덮어 속성→값 매핑 오류(예: cog=cob 혼동)를 못 잡는다
    (code-reviewer 사각지대 #1). 여기서는 cog≠cob 등 **구별 가능한 값**을 주입해
    VehicleParamsLoader가 각 파라미터를 올바른 속성에 넣었는지 직접 단언한다.
    """
    Vehicle = _load_vehicle_module()
    # 모든 값을 서로 구별 가능하게 (혼동 시 RED)
    distinct = dict(mass=12.5, cog=[1.0, 2.0, 3.0], cob=[4.0, 5.0, 6.0],
                    volume=0.07, density=1025.0, height=1.1, length=2.2, width=3.3)
    node = _FakeNode(distinct)
    v = Vehicle(node=node)
    assert v._mass == 12.5
    assert v._cog == [1.0, 2.0, 3.0]
    assert v._cob == [4.0, 5.0, 6.0]
    assert v._volume == 0.07
    assert v._density == 1025.0
    assert v._height == 1.1
    assert v._length == 2.2
    assert v._width == 3.3
    # inertial: _FakeNode가 default를 주입 안 한 키는 declare default(0.0)로 채워짐
    assert v._inertial == dict(ixx=0.0, iyy=0.0, izz=0.0, ixy=0.0, ixz=0.0, iyz=0.0)


def test_vehicle_init_declare_defaults_flow_when_param_absent():
    """declare_parameter의 기본값이 실제로 흐르는지 동결 (code-reviewer 사각지대 #2).

    _FakeNode가 _defaults에 density를 주지 않으면, declare_parameter('density', 1028.0)의
    기본값 1028.0이 그대로 흘러야 한다. density default가 1028.0에서 바뀌면 RED.
    """
    Vehicle = _load_vehicle_module()
    # density를 일부러 빼서 declare default(1028.0)가 흐르도록
    cfg = {k: v for k, v in _VALID.items() if k != 'density'}
    node = _FakeNode(cfg)
    v = Vehicle(node=node)
    assert v._density == 1028.0, f'density declare default 변동: {v._density} (기대 1028.0)'


def test_vehicle_no_duplicate_declares():
    """declare_parameter 이름에 중복이 없음(추출이 실수로 중복 declare 도입 방지).

    fake node가 중복 declare에 raise하므로, 정상 run이 통과하면 중복 없음이 보장된다.
    이 테스트는 그 불변식을 명시적으로도 동결한다.
    """
    Vehicle = _load_vehicle_module()
    node = _FakeNode(_VALID)
    Vehicle(node=node)
    declared = [name for kind, name in node.trace if kind == 'declare']
    assert len(declared) == len(set(declared)), \
        f'중복 declare 발견: {[n for n in declared if declared.count(n) > 1]}'
    assert len(declared) == 15, f'declare 수 변동: {len(declared)} (기대 15)'
