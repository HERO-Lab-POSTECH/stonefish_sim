#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Minjong Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later
import json
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from ultralytics import YOLO

# ros2 run sonar_yolo_ros2 sonar_yolo_node \
#   --ros-args \
#   -p model:=/home/colcon_ws2/src/stonefish_sim/sonar_yolo_ros2/stonefish_yolo_sofa.pt \
#   -p image_topic:=/bluerov2/fls/image \
#   -p device:=cuda:0 \
#   -p conf:=0.25 \
#   -p iou:=0.7 \
#   -p imgsz:=640


def to_uint8_bgr(img: np.ndarray) -> np.ndarray:
    """
    img: cv_bridge로 받은 numpy (H,W), (H,W,1), (H,W,3) / dtype uint8, uint16, float32 등 가능
    반환: uint8 BGR (H,W,3)
    """
    if img.ndim == 3 and img.shape[2] == 3:
        bgr = img
    else:
        gray = img[..., 0] if (img.ndim == 3 and img.shape[2] == 1) else img

        # float/uint16 등 -> 0~255로 정규화해서 uint8로
        if gray.dtype != np.uint8:
            g = gray.astype(np.float32)
            # 값 범위가 이상할 때도 견고하게
            g = cv2.normalize(g, None, 0, 255, cv2.NORM_MINMAX)
            gray_u8 = g.astype(np.uint8)
        else:
            gray_u8 = gray

        bgr = cv2.cvtColor(gray_u8, cv2.COLOR_GRAY2BGR)

    if bgr.dtype != np.uint8:
        bgr = cv2.normalize(bgr.astype(np.float32), None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    return bgr


class SonarYoloNode(Node):
    def __init__(self):
        super().__init__("sonar_yolo_node")

        # ---- params ----
        self.declare_parameter("model", "stonefish_yolo_sofa.pt")
        self.declare_parameter("image_topic", "/bluerov2/fls/image")
        self.declare_parameter("device", "cuda:0")   # "cpu" or "cuda:0" or "0"
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("publish_annotated", True)

        self.model_path = self.get_parameter("model").value
        self.image_topic = self.get_parameter("image_topic").value
        self.device = self.get_parameter("device").value
        self.conf = float(self.get_parameter("conf").value)
        self.iou = float(self.get_parameter("iou").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.publish_annotated = bool(self.get_parameter("publish_annotated").value)

        # ---- load model ----
        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()
        self.busy = False  # 프레임 드랍(실시간성)용

        # ---- pubs ----
        self.pub_det = self.create_publisher(String, "sonar_yolo/detections", 10)
        self.pub_ann = self.create_publisher(Image, "sonar_yolo/annotated", 10)

        # ---- sub ----
        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.cb_image,
            qos_profile_sensor_data  # 이미지에는 보통 sensor QoS가 안전
        )

        self.get_logger().info(
            f"Loaded model: {self.model_path}\n"
            f"Sub: {self.image_topic}\n"
            f"Pub: sonar_yolo/detections, sonar_yolo/annotated\n"
            f"device={self.device}, conf={self.conf}, iou={self.iou}, imgsz={self.imgsz}"
        )

    def cb_image(self, msg: Image):
        if self.busy:
            return
        self.busy = True

        try:
            # 1) ROS Image -> cv numpy
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            bgr = to_uint8_bgr(cv_img)

            # 2) YOLO inference
            device = int(self.device) if str(self.device).isdigit() else self.device
            r = self.model.predict(
                bgr,
                imgsz=self.imgsz,
                conf=self.conf,
                iou=self.iou,
                device=device,
                verbose=False
            )[0]

            # 3) detections -> JSON publish
            dets = []
            if r.boxes is not None and len(r.boxes) > 0:
                xyxy = r.boxes.xyxy.cpu().numpy()
                confs = r.boxes.conf.cpu().numpy()
                clss = r.boxes.cls.cpu().numpy().astype(int)
                names = self.model.names
                for (x1, y1, x2, y2), cf, c in zip(xyxy, confs, clss):
                    dets.append({
                        "class_id": int(c),
                        "class_name": str(names[int(c)]),
                        "conf": float(cf),
                        "xyxy": [float(x1), float(y1), float(x2), float(y2)],
                    })

            self.pub_det.publish(String(data=json.dumps(dets, ensure_ascii=False)))

            # 4) annotated image publish
            if self.publish_annotated and self.pub_ann.get_subscription_count() > 0:
                ann = r.plot()  # 보통 BGR numpy
                ann_msg = self.bridge.cv2_to_imgmsg(ann, encoding="bgr8")
                ann_msg.header = msg.header
                self.pub_ann.publish(ann_msg)

        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
        finally:
            self.busy = False


def main():
    rclpy.init()
    node = SonarYoloNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
