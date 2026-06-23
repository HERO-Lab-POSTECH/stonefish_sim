import numpy as np

REL = "stonefish_control/stonefish_trajectory_manager/stonefish_trajectory_manager/path_generator/bezier_curve.py"


def _cubic(load_module):
    BezierCurve = load_module(REL, "bezier_curve_under_test").BezierCurve
    # order=3: 2점 + tangents. 점·tangents 모두 np.array 필수(list면 concat 버그 — P4_FLAGS 참조)
    pnts = [np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])]
    tangents = [np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])]
    return BezierCurve(pnts, order=3, tangents=tangents)


def test_bernstein_partition_of_unity(load_module):
    c = _cubic(load_module)
    for u in (0.0, 0.25, 0.5, 0.75, 1.0):
        s = sum(c.compute_polynomial(3, i, u) for i in range(4))
        assert np.isclose(s, 1.0)  # 이항정리: 모든 u에서 Σ B_i(u) = 1


def test_bernstein_known_values(load_module):
    c = _cubic(load_module)
    assert np.isclose(c.compute_polynomial(3, 0, 0.5), 0.125)  # (1-u)^3, 실측 0.125
    assert np.isclose(c.compute_polynomial(3, 3, 0.5), 0.125)  # u^3, 실측 0.125


def test_interpolate_endpoints_match_control_points(load_module):
    c = _cubic(load_module)
    np.testing.assert_allclose(c.interpolate(0.0), c._control_pnts[0], atol=1e-12)
    np.testing.assert_allclose(c.interpolate(1.0), c._control_pnts[-1], atol=1e-12)
