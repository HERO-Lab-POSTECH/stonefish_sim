"""position 모드 feedforward 차원 정합 테스트 (T1.3, P4 Phase 1) [T1].

position_controller.py:239의 ``ff_term = self.M @ vel_ff``는 ``vel_ff``가 속도일 때
M(질량행렬)×속도 = 운동량(N·s)을 내며, 힘/토크(N·Nm)인 P/D/I 항과 더해진다 —
차원 오류(코드 주석 L238도 인정). 올바른 물리는 F_ff = M·a(힘 = 질량×가속도).

이 테스트는 position 모드 feedforward 입력이 **가속도**로 해석됨을 계약으로 고정한다:
손계산 — mass=10, accel=[2,0,0,0] m/s² → ff_term = diag([10,10,10,I])@[2,0,0,0]
= [20,0,0,0] N. 다른 항(P/D/I)을 0으로 격리(Kp=Kd=Ki=0, error=0, vel_curr=0)하면
tau[0] == 20.0 N이어야 한다.

⚠️ ①버그(차원교정)만 다룬다 — 가속도 end-to-end wiring(노드가 msg.acceleration 읽기)과
0.1 feedforward_gain 제거는 ④고도화로 분리(owner 결정, P4_FLAGS.md).

PositionController는 ROS 비의존(numpy/scipy + data_types.angle_wrap)이라 합성 부모
패키지로 직접 로드 가능.
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
def PositionController():
    """합성 부모 패키지로 position_controller.py 로드(ROS 비의존)."""
    pkg = "_t13_pkg"
    created = [pkg, f"{pkg}.control_interfaces", f"{pkg}.control_interfaces.data_types",
               f"{pkg}.controllers", f"{pkg}.controllers.position_controller"]
    for name in created:
        sys.modules.pop(name, None)

    root = types.ModuleType(pkg)
    root.__path__ = []
    sys.modules[pkg] = root

    ci = types.ModuleType(f"{pkg}.control_interfaces")
    ci.__path__ = [str(_CI_DIR)]
    sys.modules[f"{pkg}.control_interfaces"] = ci

    # data_types: numpy/dataclasses만 의존 → 실제 로드
    spec_dt = importlib.util.spec_from_file_location(
        f"{pkg}.control_interfaces.data_types", str(_CI_DIR / "data_types.py"))
    dt = importlib.util.module_from_spec(spec_dt)
    sys.modules[f"{pkg}.control_interfaces.data_types"] = dt
    spec_dt.loader.exec_module(dt)

    ctrls = types.ModuleType(f"{pkg}.controllers")
    ctrls.__path__ = [str(_CTRL_DIR)]
    sys.modules[f"{pkg}.controllers"] = ctrls

    spec_pc = importlib.util.spec_from_file_location(
        f"{pkg}.controllers.position_controller", str(_CTRL_DIR / "position_controller.py"))
    pc = importlib.util.module_from_spec(spec_pc)
    sys.modules[f"{pkg}.controllers.position_controller"] = pc
    spec_pc.loader.exec_module(pc)

    yield pc.PositionController

    for name in created:
        sys.modules.pop(name, None)


def _zero_gain_controller(PositionController, mass=10.0, inertia_zz=5.0):
    """P/D/I를 0으로 만든 컨트롤러 — feedforward 항만 출력에 남는다."""
    z = np.zeros(4)
    return PositionController(
        Kp=z, Kd=z, Ki=z, Kb=z,
        mass=mass, inertia_zz=inertia_zz,
        max_force=1e6, max_torque=1e6,  # 포화 회피
        integral_safety_factor=2.0,
        control_mode="position",
    )


def test_position_feedforward_is_mass_times_acceleration(PositionController):
    """position 모드 feedforward = M·a (힘). mass=10, a=[2,0,0,0] → F=[20,0,0,0]."""
    ctrl = _zero_gain_controller(PositionController, mass=10.0, inertia_zz=5.0)
    accel_ff = np.array([2.0, 0.0, 0.0, 0.0])  # m/s² (surge accel)
    pose = np.zeros(6)
    vel = np.zeros(6)
    tau, info = ctrl.compute_control(
        pose_des=np.zeros(4), pose_curr=pose, vel_curr=vel, dt=0.02,
        accel_ff=accel_ff,
    )
    # F_x = mass * a_x = 10 * 2 = 20 N
    assert np.isclose(tau[0], 20.0), f"expected 20 N surge feedforward, got {tau[0]}"
    np.testing.assert_allclose(info["ff_term"], [20.0, 0.0, 0.0, 0.0], atol=1e-9)


def test_position_feedforward_yaw_uses_inertia(PositionController):
    """yaw feedforward = I_zz·α. inertia_zz=5, α=[0,0,0,3] → M_z = 15 Nm."""
    ctrl = _zero_gain_controller(PositionController, mass=10.0, inertia_zz=5.0)
    accel_ff = np.array([0.0, 0.0, 0.0, 3.0])  # rad/s² (yaw accel)
    tau, info = ctrl.compute_control(
        pose_des=np.zeros(4), pose_curr=np.zeros(6), vel_curr=np.zeros(6), dt=0.02,
        accel_ff=accel_ff,
    )
    # M_z = inertia_zz * alpha = 5 * 3 = 15 Nm (tau index 5 = yaw moment)
    assert np.isclose(tau[5], 15.0), f"expected 15 Nm yaw feedforward, got {tau[5]}"
