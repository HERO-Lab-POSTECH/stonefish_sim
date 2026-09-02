# SPDX-FileCopyrightText: 2025 HERO Lab, POSTECH
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""`sonar_yolo_node.cb_image` 가 실제로 조립·발행하는 메시지를 검사한다.

`boxes_to_detections` 만 보는 테스트는 조립 단계를 통째로 놓친다 —
`bbox.center.position.x` 에 오타가 나면 콜백의 `except` 가 삼켜서 그 프레임만
조용히 무발행되고, 노드는 멀쩡히 살아 있다. 그래서 여기서는 콜백을 직접 돌린다.

ROS 없는 CI 러너에서도 돌아야 하므로 `rclpy`·`cv_bridge`·`ultralytics`·`cv2`·
`vision_msgs` 를 stub 으로 채운다. 메시지 stub 은 `__slots__` 라 규격에 없는
필드에 쓰면 `AttributeError` 가 난다 — 그게 이 테스트가 오타를 잡는 방식이다.
stub 이 진짜 규격과 같은지는 `test_message_stubs_match_the_real_vision_msgs` 가
(vision_msgs 가 있을 때만) 따로 확인한다.
"""

import sys
import types

import numpy as np
import pytest

NODE_REL = "stonefish_sonar_yolo/stonefish_sonar_yolo/sonar_yolo_node.py"
DET_REL = "stonefish_sonar_yolo/stonefish_sonar_yolo/detections.py"


# ------------------------------------------------------- vision_msgs stubs


class _Point2D:
    __slots__ = ("x", "y")

    def __init__(self):
        self.x = self.y = 0.0


class _Pose2D:
    __slots__ = ("position", "theta")

    def __init__(self):
        self.position = _Point2D()
        self.theta = 0.0


class _BoundingBox2D:
    __slots__ = ("center", "size_x", "size_y")

    def __init__(self):
        self.center = _Pose2D()
        self.size_x = self.size_y = 0.0


class _ObjectHypothesis:
    __slots__ = ("class_id", "score")

    def __init__(self):
        self.class_id = ""
        self.score = 0.0


class _ObjectHypothesisWithPose:
    __slots__ = ("hypothesis", "pose")

    def __init__(self):
        self.hypothesis = _ObjectHypothesis()
        self.pose = None


class _Detection2D:
    __slots__ = ("header", "results", "bbox", "id")

    def __init__(self):
        self.header = None
        self.results = []
        self.bbox = _BoundingBox2D()
        self.id = ""


class _Detection2DArray:
    __slots__ = ("header", "detections")

    def __init__(self):
        self.header = None
        self.detections = []


STUB_MSGS = {
    "Detection2D": _Detection2D,
    "Detection2DArray": _Detection2DArray,
    "ObjectHypothesisWithPose": _ObjectHypothesisWithPose,
    "BoundingBox2D": _BoundingBox2D,
    "ObjectHypothesis": _ObjectHypothesis,
    "Pose2D": _Pose2D,
    "Point2D": _Point2D,
}


# ------------------------------------------------------------- test doubles


class _Pub:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)

    def get_subscription_count(self):
        return 0


class _Logger:
    def __init__(self):
        self.errors = []

    def info(self, *a, **k):
        pass

    def warning(self, *a, **k):
        pass

    def error(self, msg, *a, **k):
        self.errors.append(msg)


class _Boxes:
    """ultralytics `r.boxes` 의 `.cpu().numpy()` 인터페이스만 흉내낸다."""

    class _T:
        def __init__(self, arr):
            self._a = arr

        def cpu(self):
            return self

        def numpy(self):
            return self._a

    def __init__(self, xyxy, conf, cls):
        self.xyxy = self._T(np.asarray(xyxy, np.float32))
        self.conf = self._T(np.asarray(conf, np.float32))
        self.cls = self._T(np.asarray(cls, np.float32))

    def __len__(self):
        return len(self.xyxy.numpy())


class _Result:
    def __init__(self, boxes):
        self.boxes = boxes


class _Model:
    def __init__(self, result=None, raises=None):
        self._result, self._raises = result, raises
        self.names = {0: "sofa"}

    def predict(self, *a, **k):
        if self._raises:
            raise self._raises
        return [self._result]


class _Header:
    def __init__(self, sec, nanosec):
        self.stamp = types.SimpleNamespace(sec=sec, nanosec=nanosec)
        self.frame_id = "bluerov2/fls"


@pytest.fixture
def node_cls(load_module, monkeypatch):
    """stub 을 심고 `SonarYoloNode` 클래스를 돌려준다."""
    detections = load_module(DET_REL, "stonefish_sonar_yolo_detections")

    pkg = types.ModuleType("stonefish_sonar_yolo")
    pkg.detections = detections
    stubs = {
        "stonefish_sonar_yolo": pkg,
        "stonefish_sonar_yolo.detections": detections,
        "cv2": types.ModuleType("cv2"),
        "rclpy": types.ModuleType("rclpy"),
        "rclpy.node": types.ModuleType("rclpy.node"),
        "rclpy.qos": types.ModuleType("rclpy.qos"),
        "sensor_msgs": types.ModuleType("sensor_msgs"),
        "sensor_msgs.msg": types.ModuleType("sensor_msgs.msg"),
        "cv_bridge": types.ModuleType("cv_bridge"),
        "ultralytics": types.ModuleType("ultralytics"),
        "vision_msgs": types.ModuleType("vision_msgs"),
        "vision_msgs.msg": types.ModuleType("vision_msgs.msg"),
    }
    stubs["rclpy.node"].Node = type("Node", (), {})
    stubs["rclpy.qos"].qos_profile_sensor_data = object()
    stubs["sensor_msgs.msg"].Image = type("Image", (), {})
    stubs["cv_bridge"].CvBridge = type("CvBridge", (), {})
    stubs["ultralytics"].YOLO = type("YOLO", (), {})
    for name, cls in STUB_MSGS.items():
        setattr(stubs["vision_msgs.msg"], name, cls)
    for name, mod in stubs.items():
        monkeypatch.setitem(sys.modules, name, mod)

    return load_module(NODE_REL, "stonefish_sonar_yolo_node").SonarYoloNode


def _node(node_cls, model):
    """`__init__` 을 건너뛰고 `cb_image` 가 읽는 속성만 채운 인스턴스."""
    n = object.__new__(node_cls)
    n.busy = False
    n.model = model
    n.imgsz, n.conf, n.iou, n.device = 640, 0.25, 0.45, "cpu"
    n.publish_annotated = False
    n.pub_det, n.pub_ann = _Pub(), _Pub()
    n.logger = _Logger()
    n.get_logger = lambda: n.logger
    # bridge 는 이미 3채널 uint8 을 돌려줘 to_uint8_bgr 이 cv2 를 안 타게 한다.
    n.bridge = types.SimpleNamespace(
        imgmsg_to_cv2=lambda msg, desired_encoding=None: np.zeros((4, 4, 3), np.uint8))
    return n


def _msg(sec=12, nanosec=345):
    return types.SimpleNamespace(header=_Header(sec, nanosec))


# ------------------------------------------------------------------- tests


def test_one_box_becomes_one_detection_with_the_image_header(node_cls):
    model = _Model(_Result(_Boxes([[10.0, 20.0, 30.0, 50.0]], [0.9], [0])))
    n = _node(node_cls, model)
    msg = _msg()

    n.cb_image(msg)

    assert n.logger.errors == [], n.logger.errors
    assert len(n.pub_det.published) == 1
    arr = n.pub_det.published[0]
    assert arr.header is msg.header
    assert len(arr.detections) == 1
    det = arr.detections[0]
    assert det.header is msg.header
    assert det.id == "", "Detection2D.id 는 tracking identity — 비워 둔다"
    assert (det.bbox.center.position.x, det.bbox.center.position.y) == (20.0, 35.0)
    assert (det.bbox.size_x, det.bbox.size_y) == (20.0, 30.0)
    assert det.results[0].hypothesis.class_id == "0"
    assert det.results[0].hypothesis.score == pytest.approx(0.9)


def test_zero_boxes_still_publishes_exactly_one_empty_array(node_cls):
    n = _node(node_cls, _Model(_Result(_Boxes(np.zeros((0, 4)), [], []))))
    n.cb_image(_msg())

    assert len(n.pub_det.published) == 1
    assert n.pub_det.published[0].detections == []


def test_no_boxes_attribute_is_treated_as_zero_detections(node_cls):
    n = _node(node_cls, _Model(_Result(None)))
    n.cb_image(_msg())

    assert len(n.pub_det.published) == 1
    assert n.pub_det.published[0].detections == []


def test_busy_frame_publishes_nothing(node_cls):
    n = _node(node_cls, _Model(_Result(_Boxes([[0.0, 0.0, 1.0, 1.0]], [0.9], [0]))))
    n.busy = True

    n.cb_image(_msg())

    assert n.pub_det.published == []
    assert n.busy is True, "재진입 방어 플래그를 남의 처리 도중에 풀면 안 된다"


def test_inference_failure_publishes_nothing_and_clears_busy(node_cls):
    n = _node(node_cls, _Model(raises=RuntimeError("CUDA out of memory")))

    n.cb_image(_msg())

    assert n.pub_det.published == []
    assert n.busy is False, "예외 뒤 busy 가 남으면 노드가 영구히 먹통이 된다"
    assert n.logger.errors, "실패가 로그에도 안 남으면 무발행 원인을 못 찾는다"


def test_message_stubs_match_the_real_vision_msgs():
    """stub 이 진짜 규격과 같은 필드를 갖는지 — vision_msgs 가 있을 때만."""
    msgs = pytest.importorskip("vision_msgs.msg")
    for name, stub in STUB_MSGS.items():
        real = getattr(msgs, name)
        real_fields = set(real.get_fields_and_field_types())
        assert set(stub.__slots__) == real_fields, f"{name} 필드가 규격과 다르다"
