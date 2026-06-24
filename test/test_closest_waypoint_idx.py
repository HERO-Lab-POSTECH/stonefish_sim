"""closest_waypoint_idx argmin 부호 버그 테스트 (T1.4, P4 Phase 2) [T1].

path_generator.py:159-160:
    v = np.array(self._s - self._cur_s)   # 부호 있는 차이
    idx = np.argmin(v)                     # 가장 *음수*(작은) 값

self._s는 단조증가 정규화 호장(0→1). 중간 _cur_s에 대해 _s - _cur_s는 _s[0](=0
근처)에서 가장 음수 → argmin이 항상 가장 이른 waypoint(idx 0 근처)를 반환한다.
"가장 가까운" waypoint는 argmin(abs(_s - _cur_s))여야 한다.

early-return(L155-158)이 _cur_s ∈ {0, 1}에서 버그를 가리므로, 테스트 입력은
반드시 내부 _cur_s(∉ {0,1})를 쓴다.

O13=LIVE 확정(docs/LIVENESS_AUDIT.md): path_following/path_generator 노드가 working
launch+config이므로 이 수정은 in-scope(frozen 아님).

path_generator.py는 ROS 메시지에 깊게 의존하므로 sys.modules stub으로 ROS를 막고
합성 부모 패키지로 PathGenerator만 로드, _s/_cur_s를 직접 세팅해 property를 호출한다.
"""
import importlib.util
import sys
import types
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
_TM = REPO_ROOT / "stonefish_control/stonefish_trajectory_manager/stonefish_trajectory_manager"
_PG_FILE = _TM / "path_generator/path_generator.py"

# path_generator.py가 import-time에 끌어오는 ROS/common 모듈 (stub 대상)
_ROS_STUB_MODULES = [
    "visualization_msgs", "visualization_msgs.msg",
    "stonefish_control_msgs", "stonefish_control_msgs.msg",
    "nav_msgs", "nav_msgs.msg",
    "geometry_msgs", "geometry_msgs.msg",
    "builtin_interfaces", "builtin_interfaces.msg",
]


@pytest.fixture
def PathGenerator():
    """ROS 메시지를 stub하고 합성 부모 패키지로 path_generator.py 로드."""
    pkg = "_t14_pkg"
    created = []

    saved_modules = {}
    for m in _ROS_STUB_MODULES:
        saved_modules[m] = sys.modules.get(m)
        mod = types.ModuleType(m)
        # 흔히 참조되는 메시지 타입을 가짜 클래스로
        for attr in ("MarkerArray", "Marker", "Waypoint", "WaypointSet",
                     "TrajectoryPoint", "Path", "PoseStamped", "Time"):
            setattr(mod, attr, type(attr, (), {}))
        sys.modules[m] = mod

    def _make_pkg(name, path=None):
        m = types.ModuleType(name)
        if path is not None:
            m.__path__ = [str(path)]
        sys.modules[name] = m
        created.append(name)
        return m

    _make_pkg(pkg)
    _make_pkg(f"{pkg}.common", _TM / "common")
    _make_pkg(f"{pkg}.path_generator", _TM / "path_generator")

    # common 서브모듈(waypoint/waypoint_set/trajectory_point)을 가벼운 stub으로
    # (path_generator.py는 클래스 이름만 import하지 closest_waypoint_idx에선 안 씀)
    for sub, cls in [("waypoint", "Waypoint"), ("waypoint_set", "WaypointSet"),
                     ("trajectory_point", "TrajectoryPoint")]:
        sm = types.ModuleType(f"{pkg}.common.{sub}")
        setattr(sm, cls, type(cls, (), {}))
        sys.modules[f"{pkg}.common.{sub}"] = sm
        created.append(f"{pkg}.common.{sub}")

    spec = importlib.util.spec_from_file_location(
        f"{pkg}.path_generator.path_generator", str(_PG_FILE))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[f"{pkg}.path_generator.path_generator"] = mod
    created.append(f"{pkg}.path_generator.path_generator")
    spec.loader.exec_module(mod)

    yield mod.PathGenerator

    for name in created:
        sys.modules.pop(name, None)
    for m, prev in saved_modules.items():
        if prev is None:
            sys.modules.pop(m, None)
        else:
            sys.modules[m] = prev


def test_closest_waypoint_idx_returns_nearest_not_earliest(PathGenerator):
    """중간 _cur_s에서 가장 *가까운* waypoint 인덱스를 반환해야 한다(가장 이른 것 아님).

    _s = [0, 0.25, 0.5, 0.75, 1.0], _cur_s = 0.72 → 가장 가까운 건 idx 3 (0.75).
    버그(argmin of signed diff): _s - 0.72 = [-0.72,-0.47,-0.22,0.03,0.28] →
    argmin = 0 (가장 음수). RED: 0을 반환. GREEN: argmin(abs) → 3.
    """
    pg = PathGenerator()
    pg._s = np.array([0.0, 0.25, 0.5, 0.75, 1.0])
    pg._cur_s = 0.72  # 내부값(∉ {0,1}) — early-return 우회
    assert pg.closest_waypoint_idx == 3


def test_closest_waypoint_idx_midpoint(PathGenerator):
    """_cur_s가 한 waypoint에 가까울 때 그 인덱스. _s=[0,0.5,1], _cur_s=0.55 → idx 1."""
    pg = PathGenerator()
    pg._s = np.array([0.0, 0.5, 1.0])
    pg._cur_s = 0.55
    assert pg.closest_waypoint_idx == 1
