import numpy as np
import pytest

REL = "stonefish_control/stonefish_thruster_manager/stonefish_thruster_manager/thruster_manager.py"


def _mgr(load_module):
    ThrusterManager = load_module(REL, "thruster_manager_under_test").ThrusterManager
    return ThrusterManager(tam_matrix=np.eye(6))  # 6 DOF, 6 thruster, fully actuated


def test_thrust_then_wrench_roundtrip_identity_tam(load_module):
    m = _mgr(load_module)
    wrench = np.array([1.0, -2.0, 3.0, 0.5, -0.5, 0.2])
    recovered = m.compute_wrench(m.compute_thrust_forces(wrench))
    np.testing.assert_allclose(recovered, wrench, atol=1e-10)  # 실측 OK


def test_thrust_formula_is_pinv_at_wrench(load_module):
    m = _mgr(load_module)
    wrench = np.array([2.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    expected = np.linalg.pinv(np.eye(6)) @ wrench
    np.testing.assert_allclose(m.compute_thrust_forces(wrench), expected, atol=1e-12)


def test_wrench_shape_validation(load_module):
    m = _mgr(load_module)
    with pytest.raises(ValueError):
        m.compute_thrust_forces(np.array([1.0, 2.0, 3.0]))  # not (6,)
