# SPDX-FileCopyrightText: 2025 Minjong Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""YOLO 박스를 ``vision_msgs/Detection2D`` 가 쓰는 표현으로 옮기는 순수 함수.

ROS·ultralytics 를 import 하지 않는다 — 노드 모듈은 `rclpy`·`cv_bridge`·
`ultralytics` 를 상단에서 끌어오므로 그 안에 두면 CI(ROS 없는 python3.10 러너)
에서 테스트가 수집되지 않는다.
"""

from typing import NamedTuple, Sequence

import numpy as np


class Detection(NamedTuple):
    """``vision_msgs/Detection2D`` 한 건에 실릴 값.

    Attributes:
        class_id (str): 클래스 인덱스의 십진 문자열. `vision_msgs` 4.1.1 의
            `ObjectHypothesis.class_id` 는 문자열이라 정수를 그대로 못 싣는다.
            소비자(stonefish_slam)는 라벨을 정수 인덱스로 쓰므로 인덱스를 싣는다.
            사람이 읽는 이름은 **어느 필드에도 싣지 않는다** — `Detection2D.id`
            는 메시지 사이에서 같은 물체를 잇는 tracking identity 이지 클래스
            이름이 아니고(msg 주석), 거기에 이름을 넣으면 한 프레임에 같은
            클래스가 둘일 때 소비자가 둘을 한 물체로 병합한다.
        score (float): 신뢰도.
        center_x (float): bbox 중심 x(픽셀, col = bearing 축).
        center_y (float): bbox 중심 y(픽셀, row = range bin 축).
        size_x (float): bbox 폭(픽셀).
        size_y (float): bbox 높이(픽셀).
    """

    class_id: str
    score: float
    center_x: float
    center_y: float
    size_x: float
    size_y: float


def boxes_to_detections(
    xyxy: np.ndarray,
    confs: Sequence[float],
    clss: Sequence[int],
) -> "list[Detection]":
    """ultralytics 의 xyxy 박스를 중심+크기 표현으로 바꾼다.

    Args:
        xyxy (np.ndarray): (K, 4) `[x1, y1, x2, y2]` 픽셀 좌표.
        confs (Sequence[float]): (K,) 신뢰도.
        clss (Sequence[int]): (K,) 클래스 인덱스.

    Returns:
        list[Detection]: 입력 순서를 보존한 탐지 목록. 입력이 비면 빈 목록.
    """
    out = []
    for (x1, y1, x2, y2), conf, cls in zip(xyxy, confs, clss):
        out.append(
            Detection(
                class_id=str(int(cls)),
                score=float(conf),
                center_x=0.5 * (float(x1) + float(x2)),
                center_y=0.5 * (float(y1) + float(y2)),
                size_x=abs(float(x2) - float(x1)),
                size_y=abs(float(y2) - float(y1)),
            )
        )
    return out
