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


# ---------------------------------------------------------------------------
# force_to_pwm 역추력맵 (fix/thrust-map)
# ---------------------------------------------------------------------------

T_MAX = 20.62  # bluerov2.scn 유도 추진기당 물리 최대 추력 [N]


def _f2p(load_module):
    return load_module(REL, "thruster_manager_under_test").force_to_pwm


def test_force_to_pwm_zero(load_module):
    f2p = _f2p(load_module)
    np.testing.assert_array_equal(f2p(np.zeros(8), T_MAX), np.zeros(8))


def test_force_to_pwm_boundary_full_scale(load_module):
    """F = ±T_max → pwm = ±1 (경계 정합)."""
    f2p = _f2p(load_module)
    np.testing.assert_allclose(f2p(np.array([T_MAX, -T_MAX]), T_MAX),
                               [1.0, -1.0], atol=1e-12)


def test_force_to_pwm_clips_beyond_physical(load_module):
    """|F| > T_max는 ±1로 클립 — 물리적으로 낼 수 없는 힘."""
    f2p = _f2p(load_module)
    np.testing.assert_allclose(f2p(np.array([100.0, -3 * T_MAX]), T_MAX),
                               [1.0, -1.0], atol=1e-12)


def test_force_to_pwm_monotonic(load_module):
    f2p = _f2p(load_module)
    forces = np.linspace(-1.5 * T_MAX, 1.5 * T_MAX, 101)
    pwm = f2p(forces, T_MAX)
    assert np.all(np.diff(pwm) >= 0)


def test_force_to_pwm_odd_symmetry(load_module):
    f2p = _f2p(load_module)
    forces = np.linspace(0.0, T_MAX, 20)
    np.testing.assert_allclose(f2p(-forces, T_MAX), -f2p(forces, T_MAX),
                               atol=1e-12)


def test_force_to_pwm_roundtrip_static_thrust(load_module):
    """정적 추력 모델 T = T_max·pwm|pwm| 이 명령 힘을 복원해야 한다.

    이 성질이 이 수정의 본질이다 — 종전 선형맵(F/scale)은
    T_max·(F/scale)² 로 제곱 왜곡됐다(100 N 명령 → 7.3 N).
    """
    f2p = _f2p(load_module)
    forces = np.linspace(-T_MAX, T_MAX, 41)
    pwm = f2p(forces, T_MAX)
    np.testing.assert_allclose(T_MAX * pwm * np.abs(pwm), forces, atol=1e-9)
