import numpy as np

REL = "stonefish_control/stonefish_control/stonefish_control/control_interfaces/data_types.py"


def _m(load_module):
    return load_module(REL, "data_types_under_test")


def test_angle_wrap_keeps_in_range_value(load_module):
    assert np.isclose(_m(load_module).angle_wrap(0.5), 0.5)


def test_angle_wrap_wraps_above_pi(load_module):
    # 3.5 rad → 3.5 - 2π
    assert np.isclose(_m(load_module).angle_wrap(3.5), 3.5 - 2 * np.pi)


def test_angle_wrap_pi_maps_to_minus_pi(load_module):
    # (π+π) % 2π - π = 0 - π = -π  (수학적으로 유일하게 결정됨)
    assert np.isclose(_m(load_module).angle_wrap(np.pi), -np.pi)


def test_rotation_matrix_z_is_orthonormal(load_module):
    R = _m(load_module).rotation_matrix_z(0.7)
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(R), 1.0)


def test_rotation_matrix_z_quarter_turn(load_module):
    R = _m(load_module).rotation_matrix_z(np.pi / 2)
    expected = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
    np.testing.assert_allclose(R, expected, atol=1e-12)


def test_rotation_matrix_full_identity_at_zero(load_module):
    np.testing.assert_allclose(
        _m(load_module).rotation_matrix_full(0, 0, 0), np.eye(3), atol=1e-12)


def test_rotation_matrix_full_is_proper_rotation(load_module):
    R = _m(load_module).rotation_matrix_full(0.1, -0.2, 0.3)
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(R), 1.0)


def test_mass_matrix_4dof_is_diagonal(load_module):
    p = _m(load_module).VehicleParams(mass=20.0, inertia_zz=0.13)
    np.testing.assert_allclose(p.mass_matrix_4dof, np.diag([20.0, 20.0, 20.0, 0.13]))
