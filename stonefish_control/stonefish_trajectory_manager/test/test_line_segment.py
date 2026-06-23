import numpy as np

REL = "stonefish_control/stonefish_trajectory_manager/stonefish_trajectory_manager/path_generator/line_segment.py"


def _seg(load_module):
    LineSegment = load_module(REL, "line_segment_under_test").LineSegment
    return LineSegment(np.array([0.0, 0.0, 0.0]), np.array([3.0, 4.0, 0.0]))


def test_interpolate_endpoints(load_module):
    s = _seg(load_module)
    np.testing.assert_allclose(s.interpolate(0.0), [0.0, 0.0, 0.0])
    np.testing.assert_allclose(s.interpolate(1.0), [3.0, 4.0, 0.0])


def test_interpolate_midpoint(load_module):
    np.testing.assert_allclose(_seg(load_module).interpolate(0.5), [1.5, 2.0, 0.0])


def test_get_length_is_euclidean(load_module):
    assert np.isclose(_seg(load_module).get_length(), 5.0)  # 3-4-5 triangle


def test_get_tangent_is_unit(load_module):
    t = _seg(load_module).get_tangent()
    assert np.isclose(np.linalg.norm(t), 1.0)
    np.testing.assert_allclose(t, [0.6, 0.8, 0.0])
