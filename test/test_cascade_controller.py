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


def test_yaw_gate_partial_at_45deg(CascadeController):
    """C1: e_yaw=π/4이면 yaw_gate=cos(π/4)≈0.707 — sway 위치오차가 게이트로 스케일된다.

    게이트가 제거되면 e_outer[1]=2.0, 게이트 적용 시 2.0*cos(π/4)≈1.4142 → 두 값이
    달라 regression guard로 유효하다(e_yaw=0 케이스는 gate=1이라 차이 없음).
    """
    c = _make_cascade(CascadeController)
    # 차량 yaw=0, lookahead가 body-right(+y_body=+y_world)로 2m, yaw_des=π/4 → e_yaw=π/4
    pose_des = np.array([0.0, 2.0, 0.0, np.pi / 4])
    pose_curr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    vel_curr = np.zeros(6)
    _, dbg = c.compute_control(pose_des, pose_curr, vel_curr, dt=0.1)
    # e_outer[1] = 2.0 * cos(π/4) ≈ 1.4142 (게이트 제거 시 2.0이라 FAIL)
    assert dbg['e_outer'][1] == pytest.approx(2.0 * np.cos(np.pi / 4), abs=1e-9), \
        'e_yaw=π/4 → yaw_gate=cos(π/4)≈0.707 → sway 위치오차 2.0*0.707≈1.4142'


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


# ============================================================
# P2 모델 주입: accel ff = M_eff · d(v_sp)/dt (내부 미분, 리뷰 BLOCKER-1 반영)
# ============================================================

def _alpha(cutoff_hz, dt):
    """컨트롤러와 동일한 1차 LPF 계수 오라클."""
    return 1.0 - np.exp(-2.0 * np.pi * cutoff_hz * dt)


def test_accel_ff_from_v_sp_derivative(CascadeController):
    """P2-1: v_sp 변화 시 ff = M_eff·LPF(dv_sp/dt)가 tau에 합산된다.

    inner 게인 전부 0으로 격리 → tau == ff. 첫 틱은 미분기 초기화라 ff=0,
    둘째 틱에서 raw=(v_sp2-v_sp1)/dt, acc=alpha·raw.
    """
    M_eff = np.array([70.2, 62.0, 63.9, 0.24])
    c = _make_cascade(
        CascadeController, M_eff_diag=M_eff, accel_ff_cutoff_hz=2.0,
        Kp_inner=np.zeros(4), Ki_inner=np.zeros(4), Kd_inner=np.zeros(4))
    pose_curr = np.zeros(6)
    vel_curr = np.zeros(6)
    dt = 0.1
    tau1, _ = c.compute_control(np.array([0.5, 0.0, 0.0, 0.0]), pose_curr,
                                vel_curr, dt=dt)
    np.testing.assert_allclose(tau1, np.zeros(6), atol=1e-9)  # 첫 틱 ff=0
    tau2, info = c.compute_control(np.array([0.6, 0.0, 0.0, 0.0]), pose_curr,
                                   vel_curr, dt=dt)
    # v_sp: 0.5→0.6 (Kp_outer=1), raw=1.0 (clamp ±2.0 이내)
    expected_acc = _alpha(2.0, dt) * 1.0
    assert tau2[0] == pytest.approx(M_eff[0] * expected_acc, rel=1e-9)
    assert info['tau_ff'][0] == pytest.approx(M_eff[0] * expected_acc, rel=1e-9)


def test_accel_ff_zero_when_v_sp_constant(CascadeController):
    """P2-2: v_sp가 일정하면 ff=0 — 정상상태에서 유령 힘 없음(하위호환)."""
    c = _make_cascade(CascadeController,
                      M_eff_diag=np.array([70.2, 62.0, 63.9, 0.24]),
                      accel_ff_cutoff_hz=2.0,
                      Ki_inner=np.zeros(4))
    pose_des = np.array([1.0, 0.5, -0.3, 0.2])
    tau1, _ = c.compute_control(pose_des, np.zeros(6), np.zeros(6), dt=0.1)
    tau2, info = c.compute_control(pose_des, np.zeros(6), np.zeros(6), dt=0.1)
    np.testing.assert_allclose(tau2, tau1, atol=1e-9)
    np.testing.assert_allclose(info['tau_ff'], np.zeros(4), atol=1e-9)


def test_accel_ff_cutoff_zero_disables(CascadeController):
    """P2-3: accel_ff_cutoff_hz=0 → v_sp가 변해도 ff=0 (기존 동작)."""
    c = _make_cascade(CascadeController,
                      M_eff_diag=np.array([70.2, 62.0, 63.9, 0.24]),
                      accel_ff_cutoff_hz=0.0,
                      Kp_inner=np.zeros(4), Ki_inner=np.zeros(4))
    c.compute_control(np.array([0.5, 0.0, 0.0, 0.0]), np.zeros(6), np.zeros(6), dt=0.1)
    tau2, _ = c.compute_control(np.array([0.6, 0.0, 0.0, 0.0]), np.zeros(6),
                                np.zeros(6), dt=0.1)
    np.testing.assert_allclose(tau2, np.zeros(6), atol=1e-9)


def test_M_eff_fallback_rigid_mass(CascadeController):
    """P2-4: M_eff_diag 미공급 → 강체 질량 fallback (M = diag[m,m,m,Izz])."""
    c = _make_cascade(CascadeController)   # mass=11.5, inertia_zz=0.16
    np.testing.assert_allclose(np.diag(c.M), [11.5, 11.5, 11.5, 0.16], atol=1e-9)


def test_accel_ff_saturation_engages_backcalc(CascadeController):
    """P2-5 (리뷰 MINOR-7): ff 단독 포화 시 back-calculation이 적분기를 되돌린다.

    Kp=0·Ki=1로 두면 tau = i_in + ff. 큰 v_sp 점프(raw clamp 2.0)로 ff가
    max_force를 넘게 만들고, 적분기가 순수 사다리꼴 값보다 작아졌는지 확인
    — ff에서 온 excess가 적분기로 전파되는 경로가 살아 있고 발산하지 않는다.
    """
    M_eff = np.array([70.2, 62.0, 63.9, 0.24])
    c = _make_cascade(
        CascadeController, M_eff_diag=M_eff, accel_ff_cutoff_hz=2.0,
        Kp_inner=np.zeros(4), Ki_inner=np.array([1.0, 1.0, 1.0, 1.0]),
        Kd_inner=np.zeros(4), max_force=10.0, max_torque=10.0)
    dt = 0.1
    c.compute_control(np.array([0.0, 0.0, 0.0, 0.0]), np.zeros(6), np.zeros(6), dt=dt)
    tau, info = c.compute_control(np.array([5.0, 0.0, 0.0, 0.0]), np.zeros(6),
                                  np.zeros(6), dt=dt)
    # raw = 5.0/0.1 = 50 → clamp 2.0, acc = alpha·2.0, ff = 70.2·acc ≈ 100 N > 10
    assert info['tau_ff'][0] > 10.0
    assert tau[0] == pytest.approx(10.0, abs=1e-9)          # 포화 clip
    assert info['saturated'] is True or info['saturation_count'] >= 1
    # 순수 사다리꼴 적분(0.5·(e2+e1)·dt, e1=0, e2=5.0)보다 작아야 back-calc 발동
    pure_trapezoid = 0.5 * (5.0 + 0.0) * dt
    assert info['integral_inner'][0] < pure_trapezoid


def test_damping_static_ff_oracle(CascadeController):
    """P2-6 (리뷰 MINOR-2): ff = d1·v_sp + d2·v_sp|v_sp| + static (게인 0 격리).

    accel ff는 cutoff=0으로 끄고 v_sp=[0.5,0,-0.2,0.3] 강제(vel_ff 경유).
    """
    d1 = np.array([1.65, 11.30, 17.95, 0.31])
    d2 = np.array([80.28, 44.25, 157.51, 0.45])
    static = np.array([0.0, 0.0, 7.27, 0.0])
    c = _make_cascade(
        CascadeController,
        Kp_outer=np.zeros(4), Kp_inner=np.zeros(4), Ki_inner=np.zeros(4),
        accel_ff_cutoff_hz=0.0, d1_diag=d1, d2_diag=d2, static_ff=static)
    vel_ff = np.array([0.5, 0.0, -0.2, 0.3])
    tau, info = c.compute_control(np.zeros(4), np.zeros(6), np.zeros(6),
                                  dt=0.1, vel_ff=vel_ff)
    expected = d1 * vel_ff + d2 * vel_ff * np.abs(vel_ff) + static
    np.testing.assert_allclose(info['tau_ff'], expected, atol=1e-9)
    np.testing.assert_allclose(
        tau, [expected[0], expected[1], expected[2], 0.0, 0.0, expected[3]],
        atol=1e-9)


def test_damping_ff_default_off(CascadeController):
    """P2-7: d1/d2/static 미공급 → damping ff 0 (하위호환)."""
    c = _make_cascade(CascadeController,
                      Kp_outer=np.zeros(4), Kp_inner=np.zeros(4),
                      Ki_inner=np.zeros(4), accel_ff_cutoff_hz=0.0)
    tau, _ = c.compute_control(np.zeros(4), np.zeros(6), np.zeros(6),
                               dt=0.1, vel_ff=np.array([0.5, 0.0, 0.0, 0.0]))
    np.testing.assert_allclose(tau, np.zeros(6), atol=1e-9)
