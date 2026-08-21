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
