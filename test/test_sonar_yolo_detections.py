# SPDX-FileCopyrightText: 2025 HERO Lab, POSTECH
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""`stonefish_sonar_yolo.detections` — xyxy → 중심+크기 변환의 특성 테스트.

노드 모듈(`sonar_yolo_node.py`)은 상단에서 `rclpy`·`cv_bridge`·`ultralytics` 를
끌어오므로 여기서 열 수 없다. 변환 로직만 ROS 없는 모듈로 떼어 두고 그것을
`load_module` 로 연다 — CI 러너에는 ROS 가 없다.
"""

import numpy as np
import pytest

REL = "stonefish_sonar_yolo/stonefish_sonar_yolo/detections.py"


@pytest.fixture
def det(load_module):
    return load_module(REL, "sonar_yolo_detections")


def test_center_and_size_from_corners(det):
    out = det.boxes_to_detections(
        np.array([[10.0, 20.0, 30.0, 50.0]]), [0.9], [1], {1: "sofa"}
    )
    assert len(out) == 1
    d = out[0]
    assert (d.center_x, d.center_y) == (20.0, 35.0)
    assert (d.size_x, d.size_y) == (20.0, 30.0)


def test_class_id_is_decimal_string_of_the_index(det):
    """vision_msgs 4.1.1 의 ObjectHypothesis.class_id 는 string 이다.

    소비자(stonefish_slam)가 라벨을 정수 인덱스로 쓰므로 이름이 아니라 인덱스를
    싣는다 — 이름은 `Detection2D.id` 로 따로 간다.
    """
    d = det.boxes_to_detections(
        np.array([[0.0, 0.0, 1.0, 1.0]]), [0.5], [7], {7: "sofa"}
    )[0]
    assert d.class_id == "7"
    assert d.class_name == "sofa"
    assert isinstance(d.score, float) and d.score == pytest.approx(0.5)


def test_empty_input_gives_empty_list(det):
    assert det.boxes_to_detections(np.zeros((0, 4)), [], [], {}) == []


def test_size_stays_positive_when_corners_are_reversed(det):
    d = det.boxes_to_detections(
        np.array([[30.0, 50.0, 10.0, 20.0]]), [0.1], [0], {0: "x"}
    )[0]
    assert d.size_x == 20.0 and d.size_y == 30.0
    assert (d.center_x, d.center_y) == (20.0, 35.0)


def test_order_is_preserved(det):
    out = det.boxes_to_detections(
        np.array([[0.0, 0.0, 2.0, 2.0], [10.0, 10.0, 14.0, 14.0]]),
        [0.3, 0.8],
        [0, 1],
        {0: "a", 1: "b"},
    )
    assert [d.class_name for d in out] == ["a", "b"]
    assert [d.center_x for d in out] == [1.0, 12.0]
