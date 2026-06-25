#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""CascadeController 단위테스트 — outer 위치P → inner 속도PI 2단 보상기.

손계산 오라클(atol=1e-9). 합성 부모 패키지 fixture로 직접 로드(상대 import 충족).
설계 SSOT: .sp/specs/2026-06-24-path-following-position-cascade-design.md §8B.
"""
import importlib.util
import sys
import types
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
_CTRL_DIR = (REPO_ROOT
             / "stonefish_control/stonefish_control/stonefish_control/controllers")
_CI_DIR = (REPO_ROOT
           / "stonefish_control/stonefish_control/stonefish_control/control_interfaces")


@pytest.fixture
def CascadeController():
    """합성 부모 패키지로 cascade_controller.py 로드(상대 import 충족, ROS 비의존).

    test_feedforward_dimensional.py의 검증된 패턴. load_module(평면)은 상대 import
    (..control_interfaces.data_types)를 충족 못 하므로 쓰지 않는다.
    """
    pkg = "_cascade_pkg"
    created = [pkg, f"{pkg}.control_interfaces", f"{pkg}.control_interfaces.data_types",
               f"{pkg}.controllers", f"{pkg}.controllers.cascade_controller"]
    for name in created:
        sys.modules.pop(name, None)

    root = types.ModuleType(pkg)
    root.__path__ = []
    sys.modules[pkg] = root

    ci = types.ModuleType(f"{pkg}.control_interfaces")
    ci.__path__ = [str(_CI_DIR)]
    sys.modules[f"{pkg}.control_interfaces"] = ci

    spec_dt = importlib.util.spec_from_file_location(
        f"{pkg}.control_interfaces.data_types", str(_CI_DIR / "data_types.py"))
    dt = importlib.util.module_from_spec(spec_dt)
    sys.modules[f"{pkg}.control_interfaces.data_types"] = dt
    spec_dt.loader.exec_module(dt)

    ctrls = types.ModuleType(f"{pkg}.controllers")
    ctrls.__path__ = [str(_CTRL_DIR)]
    sys.modules[f"{pkg}.controllers"] = ctrls

    spec_cc = importlib.util.spec_from_file_location(
        f"{pkg}.controllers.cascade_controller", str(_CTRL_DIR / "cascade_controller.py"))
    cc = importlib.util.module_from_spec(spec_cc)
    sys.modules[f"{pkg}.controllers.cascade_controller"] = cc
    spec_cc.loader.exec_module(cc)

    yield cc.CascadeController

    for name in created:
        sys.modules.pop(name, None)


def _make_cascade(CascadeController, **overrides):
    """격리 테스트용 기본 인스턴스. 게인을 케이스별로 override."""
    params = dict(
        Kp_outer=np.array([1.0, 1.0, 1.0, 1.0]),
        Kp_inner=np.array([1.0, 1.0, 1.0, 1.0]),
        Ki_inner=np.array([0.0, 0.0, 0.0, 0.0]),
        Kb_inner=np.array([0.8, 0.8, 0.8, 0.8]),
        Kd_inner=np.array([0.0, 0.0, 0.0, 0.0]),
        mass=11.5,
        inertia_zz=0.16,
        v_sp_limit=np.array([100.0, 100.0, 100.0, 100.0]),
        max_force=10000.0,
        max_torque=10000.0,
        integral_safety_factor=0.5,
    )
    params.update(overrides)
    return CascadeController(**params)


def test_outer_inner_serial_chain(CascadeController):
    """B1: Kp_outer=Kp_inner=1, 나머지 0. e_pos_body=[1,0,0], v_body=0, vel_ff=0
    → v_sp=Kp_outer·e=[1,0,0,0], e_inner=v_sp, tau[0]=Kp_inner·v_sp[0]=1."""
    c = _make_cascade(CascadeController)
    pose_des = np.array([1.0, 0.0, 0.0, 0.0])      # 1m ahead in world (yaw=0 → body==world)
    pose_curr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    vel_curr = np.zeros(6)
    tau, info = c.compute_control(pose_des, pose_curr, vel_curr, dt=0.1, vel_ff=None)
    np.testing.assert_allclose(info['v_sp'], [1.0, 0.0, 0.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(info['e_inner'], [1.0, 0.0, 0.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(tau, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], atol=1e-9)


def test_vel_ff_adds_to_setpoint(CascadeController):
    """B2: vel_ff=[0.5,0,0,0], e_pos_body=0 → v_sp[0]=0.5. sway 슬롯 vel_ff[1]=0 명시."""
    c = _make_cascade(CascadeController)
    pose_des = np.array([0.0, 0.0, 0.0, 0.0])
    pose_curr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    vel_curr = np.zeros(6)
    vel_ff = np.array([0.5, 0.0, 0.0, 0.0])
    tau, info = c.compute_control(pose_des, pose_curr, vel_curr, dt=0.1, vel_ff=vel_ff)
    assert info['v_sp'][0] == pytest.approx(0.5, abs=1e-9)
    assert info['v_sp'][1] == pytest.approx(0.0, abs=1e-9), 'sway ff must be 0 (no double-correction)'


def test_v_sp_clamped_after_ff(CascadeController):
    """B3: 큰 e_pos_body → v_sp가 v_sp_limit로 포화 (ff 합산 후)."""
    c = _make_cascade(CascadeController, Kp_outer=np.array([1.0, 1.0, 1.0, 1.0]),
                      v_sp_limit=np.array([0.5, 0.3, 0.25, 0.6]))
    pose_des = np.array([10.0, 0.0, 0.0, 0.0])     # 10m ahead → v_sp would be 10
    pose_curr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    vel_curr = np.zeros(6)
    _, info = c.compute_control(pose_des, pose_curr, vel_curr, dt=0.1, vel_ff=None)
    assert info['v_sp'][0] == pytest.approx(0.5, abs=1e-9), 'v_sp[0] clamped to limit'


def test_y_error_single_channel_sway(CascadeController):
    """B4: e_pos_body=[0,1,0](des가 +Y 1m) → sway setpoint 양수(+0.5=Kp_outer[1]·1).
    ILOS sway=0과 합쳐 이중보정 없음을 회귀 고정. yaw=0이므로 body y == world y."""
    c = _make_cascade(CascadeController, Kp_outer=np.array([1.0, 0.5, 1.0, 1.0]))
    pose_des = np.array([0.0, 1.0, 0.0, 0.0])      # path point 1m to +Y (world)
    pose_curr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    vel_curr = np.zeros(6)
    _, info = c.compute_control(pose_des, pose_curr, vel_curr, dt=0.1, vel_ff=None)
    # e_pos_body[1] = +1 (des is +Y of current) → v_sp[1] = Kp_outer[1]*1 = 0.5
    assert info['v_sp'][1] == pytest.approx(0.5, abs=1e-9)


def test_inner_back_calc_anti_windup(CascadeController):
    """B5: 강제 포화 → inner 적분이 back-calc로 감소(excess>0 → integral 음수)."""
    c = _make_cascade(CascadeController,
                      Kp_inner=np.array([1000.0, 1.0, 1.0, 1.0]),
                      Ki_inner=np.array([10.0, 10.0, 10.0, 10.0]),
                      Kb_inner=np.array([0.5, 0.5, 0.5, 0.5]),
                      v_sp_limit=np.array([100.0, 100.0, 100.0, 100.0]),
                      max_force=5.0, max_torque=5.0)
    pose_des = np.array([1.0, 0.0, 0.0, 0.0])
    pose_curr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    vel_curr = np.zeros(6)
    _, info = c.compute_control(pose_des, pose_curr, vel_curr, dt=0.1, vel_ff=None)
    assert info['saturated'] is True
    # 사다리꼴 적분으로 integral_inner[0]은 먼저 +0.05(=0.5*(1+0)*0.1)로 누적된 뒤
    # back-calc로 (excess/Ki)*Kb 만큼 감산된다. excess가 매우 크므로 순효과는 음수.
    assert info['integral_inner'][0] < 0.0


def test_rotation_consistency_with_position_controller(CascadeController):
    """B6: yaw≠0에서 e_pos_world→e_pos_body가 R.T@e_pos_world와 동일(F2 공유 수학)."""
    from scipy.spatial.transform import Rotation
    c = _make_cascade(CascadeController)
    yaw = np.pi / 6
    pose_des = np.array([1.0, 2.0, 0.0, yaw])
    pose_curr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, yaw])
    vel_curr = np.zeros(6)
    _, info = c.compute_control(pose_des, pose_curr, vel_curr, dt=0.1, vel_ff=None)
    R = Rotation.from_euler('xyz', [0.0, 0.0, yaw]).as_matrix()
    e_pos_body_expected = R.T @ np.array([1.0, 2.0, 0.0])
    # v_sp[:3] = Kp_outer[:3] * e_pos_body (Kp_outer=1)
    np.testing.assert_allclose(info['v_sp'][:3], e_pos_body_expected, atol=1e-9)


def test_reset_zeros_integral(CascadeController):
    """B7: reset() 후 integral_inner=0."""
    c = _make_cascade(CascadeController, Ki_inner=np.array([10.0, 10.0, 10.0, 10.0]))
    pose_des = np.array([1.0, 0.0, 0.0, 0.0])
    pose_curr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    c.compute_control(pose_des, pose_curr, np.zeros(6), dt=0.1, vel_ff=None)
    c.reset()
    _, info = c.compute_control(np.zeros(4), pose_curr, np.zeros(6), dt=0.0, vel_ff=None)
    np.testing.assert_allclose(info['integral_inner'], [0.0, 0.0, 0.0, 0.0], atol=1e-9)


def test_yaw_error_angle_wrap(CascadeController):
    """B8: e_yaw는 angle_wrap 적용 — des_yaw=π-0.1, curr_yaw=-π+0.1 → e_yaw≈-0.2(wrap)."""
    c = _make_cascade(CascadeController)
    des_yaw = np.pi - 0.1
    curr_yaw = -np.pi + 0.1
    pose_des = np.array([0.0, 0.0, 0.0, des_yaw])
    pose_curr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, curr_yaw])
    _, info = c.compute_control(pose_des, pose_curr, np.zeros(6), dt=0.1, vel_ff=None)
    # wrapped error = -0.2 (not 2π-0.2). v_sp[3] = Kp_outer[3]*e_yaw = -0.2
    assert info['v_sp'][3] == pytest.approx(-0.2, abs=1e-9)


# ── HybridController cascade 라우팅 (Task 4) ──────────────────────────────────

_HYBRID_PATH = (
    'stonefish_control/stonefish_control/stonefish_control/'
    'controllers/hybrid_controller.py'
)
_POS_PATH = (
    'stonefish_control/stonefish_control/stonefish_control/'
    'controllers/position_controller.py'
)
_CASCADE_PATH = (
    'stonefish_control/stonefish_control/stonefish_control/'
    'controllers/cascade_controller.py'
)
_CI_PATH = (
    'stonefish_control/stonefish_control/stonefish_control/'
    'control_interfaces/data_types.py'
)


def _make_hybrid(tmp_pkg='_hybrid_test_pkg'):
    """HybridController를 합성 패키지로 로드 — 상대 import 충족.

    hybrid_controller.py는 .position_controller와 .cascade_controller를 상대 import하고,
    cascade_controller.py는 ..control_interfaces.data_types를 상대 import한다.
    모두 동일 합성 루트 패키지 아래 등록해 충족한다.
    """
    pkg = tmp_pkg
    ctrl_dir = str(REPO_ROOT / 'stonefish_control/stonefish_control/stonefish_control/controllers')
    ci_dir = str(REPO_ROOT / 'stonefish_control/stonefish_control/stonefish_control/control_interfaces')

    to_clean = [
        pkg,
        f'{pkg}.control_interfaces',
        f'{pkg}.control_interfaces.data_types',
        f'{pkg}.controllers',
        f'{pkg}.controllers.position_controller',
        f'{pkg}.controllers.cascade_controller',
        f'{pkg}.controllers.hybrid_controller',
    ]
    for name in to_clean:
        sys.modules.pop(name, None)

    # root package
    root = types.ModuleType(pkg)
    root.__path__ = []
    sys.modules[pkg] = root

    # control_interfaces sub-package
    ci_pkg = types.ModuleType(f'{pkg}.control_interfaces')
    ci_pkg.__path__ = [ci_dir]
    sys.modules[f'{pkg}.control_interfaces'] = ci_pkg

    spec_dt = importlib.util.spec_from_file_location(
        f'{pkg}.control_interfaces.data_types',
        str(REPO_ROOT / _CI_PATH))
    dt_mod = importlib.util.module_from_spec(spec_dt)
    sys.modules[f'{pkg}.control_interfaces.data_types'] = dt_mod
    spec_dt.loader.exec_module(dt_mod)

    # controllers sub-package
    ctrls_pkg = types.ModuleType(f'{pkg}.controllers')
    ctrls_pkg.__path__ = [ctrl_dir]
    sys.modules[f'{pkg}.controllers'] = ctrls_pkg

    # position_controller
    spec_pos = importlib.util.spec_from_file_location(
        f'{pkg}.controllers.position_controller',
        str(REPO_ROOT / _POS_PATH))
    pos_mod = importlib.util.module_from_spec(spec_pos)
    sys.modules[f'{pkg}.controllers.position_controller'] = pos_mod
    spec_pos.loader.exec_module(pos_mod)

    # cascade_controller
    spec_cc = importlib.util.spec_from_file_location(
        f'{pkg}.controllers.cascade_controller',
        str(REPO_ROOT / _CASCADE_PATH))
    cc_mod = importlib.util.module_from_spec(spec_cc)
    sys.modules[f'{pkg}.controllers.cascade_controller'] = cc_mod
    spec_cc.loader.exec_module(cc_mod)

    # hybrid_controller
    spec_hyb = importlib.util.spec_from_file_location(
        f'{pkg}.controllers.hybrid_controller',
        str(REPO_ROOT / _HYBRID_PATH))
    hyb_mod = importlib.util.module_from_spec(spec_hyb)
    sys.modules[f'{pkg}.controllers.hybrid_controller'] = hyb_mod
    spec_hyb.loader.exec_module(hyb_mod)

    return hyb_mod, to_clean


def test_hybrid_routes_cascade_mode():
    """B-route: control_mode='cascade'일 때 CascadeController가 호출되어 tau 6-vector 반환."""
    hyb_mod, to_clean = _make_hybrid()
    try:
        c = hyb_mod.HybridController(
            Kp_vel=np.array([200., 200., 250., 150.]), Kd_vel=np.array([0., 100., 100., 80.]),
            Ki_vel=np.array([50., 50., 60., 10.]), Kb_vel=np.array([0.8]*4),
            Kp_pos=np.array([300., 300., 400., 200.]), Kd_pos=np.array([150., 150., 200., 100.]),
            Ki_pos=np.array([10., 10., 20., 5.]), Kb_pos=np.array([0.8]*4),
            Kp_outer=np.array([0.4, 0.4, 0.3, 0.8]), Ki_outer=np.array([0.]*4),
            Kp_inner=np.array([200., 200., 250., 150.]), Ki_inner=np.array([50., 50., 60., 10.]),
            Kd_inner=np.array([0., 100., 100., 80.]), Kb_inner=np.array([0.8]*4),
            v_sp_limit=np.array([0.5, 0.3, 0.25, 0.6]),
            mass=11.5, inertia_zz=0.16,
            initial_mode='cascade',
        )
        pose_des = np.array([1.0, 0.0, 0.0, 0.0])
        pose_curr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        vel_curr = np.zeros(6)
        vel_des = np.array([0.5, 0.0, 0.0, 0.0])
        tau, info = c.compute_control(pose_des, pose_curr, vel_curr, 0.1, vel_des)
        assert tau.shape == (6,)
        assert info['active_mode'] == 'cascade'
    finally:
        for name in to_clean:
            sys.modules.pop(name, None)


def test_hybrid_set_mode_accepts_cascade():
    """set_mode('cascade')가 ValueError 없이 수용되고 cascade reset 호출."""
    hyb_mod, to_clean = _make_hybrid()
    try:
        c = hyb_mod.HybridController(
            Kp_vel=np.array([200., 200., 250., 150.]), Kd_vel=np.array([0., 100., 100., 80.]),
            Ki_vel=np.array([50., 50., 60., 10.]), Kb_vel=np.array([0.8]*4),
            Kp_pos=np.array([300., 300., 400., 200.]), Kd_pos=np.array([150., 150., 200., 100.]),
            Ki_pos=np.array([10., 10., 20., 5.]), Kb_pos=np.array([0.8]*4),
            Kp_outer=np.array([0.4, 0.4, 0.3, 0.8]), Ki_outer=np.array([0.]*4),
            Kp_inner=np.array([200., 200., 250., 150.]), Ki_inner=np.array([50., 50., 60., 10.]),
            Kd_inner=np.array([0., 100., 100., 80.]), Kb_inner=np.array([0.8]*4),
            v_sp_limit=np.array([0.5, 0.3, 0.25, 0.6]),
            mass=11.5, inertia_zz=0.16,
            initial_mode='velocity',
        )
        c.set_mode('cascade')   # must not raise
        assert c.control_mode == 'cascade'
    finally:
        for name in to_clean:
            sys.modules.pop(name, None)


# ── 결함 C: outer yaw 게이팅 (Task 2) ───────────────────────────────────────


def test_yaw_gate_full_when_aligned(CascadeController):
    """C1: e_yaw=0이면 yaw_gate=1 — sway 위치오차 기여가 게이트 전과 동일."""
    c = _make_cascade(CascadeController)
    # 차량 yaw=0(정렬), lookahead가 body-right(+y_body=+y_world)로 2m
    pose_des = np.array([0.0, 2.0, 0.0, 0.0])      # yaw_des=0 → e_yaw=0
    pose_curr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    vel_curr = np.zeros(6)
    _, dbg = c.compute_control(pose_des, pose_curr, vel_curr, dt=0.1)
    # e_outer[1] = sway 위치오차 = 2.0 (gate=cos(0)=1 적용 후에도 2.0)
    assert dbg['e_outer'][1] == pytest.approx(2.0, abs=1e-9), \
        'e_yaw=0 → yaw_gate=1 → sway 위치오차 2.0 무변'


def test_yaw_gate_zero_at_90deg(CascadeController):
    """C2: e_yaw=π/2이면 yaw_gate=0 — sway 위치오차 기여가 0으로 차단된다."""
    c = _make_cascade(CascadeController)
    # yaw_des - yaw_curr = π/2
    pose_des = np.array([0.0, 2.0, 0.0, np.pi / 2])
    pose_curr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    vel_curr = np.zeros(6)
    _, dbg = c.compute_control(pose_des, pose_curr, vel_curr, dt=0.1)
    assert dbg['e_outer'][1] == pytest.approx(0.0, abs=1e-9), \
        'e_yaw=π/2 → yaw_gate=cos(π/2)=0 → sway 위치오차 0'


def test_yaw_gate_clamped_nonneg_at_180deg(CascadeController):
    """C3: e_yaw=π이면 cos=-1이지만 max(.,0)으로 게이트=0 (역방향 명령 차단)."""
    c = _make_cascade(CascadeController)
    pose_des = np.array([0.0, 2.0, 0.0, np.pi])
    pose_curr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    vel_curr = np.zeros(6)
    _, dbg = c.compute_control(pose_des, pose_curr, vel_curr, dt=0.1)
    assert dbg['e_outer'][1] == pytest.approx(0.0, abs=1e-9), \
        'e_yaw=π → max(cos(π),0)=0 → sway 위치오차 0 (음수 차단)'


def test_yaw_gate_does_not_affect_surge_yaw_depth(CascadeController):
    """C4: yaw 게이트는 sway에만 작용 — surge·heave·yaw 채널 무영향."""
    c = _make_cascade(CascadeController)
    # surge·heave 위치오차 있고 e_yaw=π/2 (sway만 게이트돼야)
    # yaw_curr=0이므로 R=I → e_pos_body = e_pos_world = [3,2,1]
    pose_des = np.array([3.0, 2.0, 1.0, np.pi / 2])
    pose_curr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    vel_curr = np.zeros(6)
    _, dbg = c.compute_control(pose_des, pose_curr, vel_curr, dt=0.1)
    assert dbg['e_outer'][0] == pytest.approx(3.0, abs=1e-9), 'surge 무게이트'
    assert dbg['e_outer'][2] == pytest.approx(1.0, abs=1e-9), 'heave 무게이트'
    assert dbg['e_outer'][3] == pytest.approx(np.pi / 2, abs=1e-9), 'yaw 무게이트'
    assert dbg['e_outer'][1] == pytest.approx(0.0, abs=1e-9), 'sway만 게이트→0'


def test_hybrid_velocity_position_unchanged():
    """하위호환: velocity/position 라우팅이 여전히 동작(cascade 추가가 기존 경로 불변)."""
    hyb_mod, to_clean = _make_hybrid()
    try:
        c = hyb_mod.HybridController(
            Kp_vel=np.array([200., 200., 250., 150.]), Kd_vel=np.array([0., 100., 100., 80.]),
            Ki_vel=np.array([50., 50., 60., 10.]), Kb_vel=np.array([0.8]*4),
            Kp_pos=np.array([300., 300., 400., 200.]), Kd_pos=np.array([150., 150., 200., 100.]),
            Ki_pos=np.array([10., 10., 20., 5.]), Kb_pos=np.array([0.8]*4),
            Kp_outer=np.array([0.4, 0.4, 0.3, 0.8]), Ki_outer=np.array([0.]*4),
            Kp_inner=np.array([200., 200., 250., 150.]), Ki_inner=np.array([50., 50., 60., 10.]),
            Kd_inner=np.array([0., 100., 100., 80.]), Kb_inner=np.array([0.8]*4),
            v_sp_limit=np.array([0.5, 0.3, 0.25, 0.6]),
            mass=11.5, inertia_zz=0.16,
            initial_mode='velocity',
        )
        pose_des = np.array([0.0, 0.0, 0.0, 0.0])
        pose_curr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        vel_curr = np.zeros(6)
        vel_des = np.array([1.0, 0.0, 0.0, 0.0])
        tau, info = c.compute_control(pose_des, pose_curr, vel_curr, 0.1, vel_des)
        assert info['active_mode'] == 'velocity'
        assert tau.shape == (6,)
    finally:
        for name in to_clean:
            sys.modules.pop(name, None)
