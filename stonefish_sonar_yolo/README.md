# stonefish_sonar_yolo

FLS 소나 이미지에 YOLO 추론을 수행해 탐지 결과를 발행하는 노드.

- 구독: `image_topic` (기본 `/bluerov2/fls/image`, `sensor_msgs/Image`)
- 발행: `sonar_yolo/detections` (`std_msgs/String`, JSON 배열: class_id/class_name/conf/xyxy),
  `sonar_yolo/annotated` (`sensor_msgs/Image`, 구독자가 있을 때만)

## 요구 사항

- pip 전용 의존성: `ultralytics` (rosdep 키 없음 — `pip install ultralytics`)
- 가중치 파일은 repo에 포함되지 않음. 학습된 `stonefish_yolo_sofa.pt`를 별도로 받아
  `model` 파라미터로 경로를 지정한다.

## 실행

```bash
ros2 run stonefish_sonar_yolo sonar_yolo_node --ros-args \
  -p model:=/path/to/stonefish_yolo_sofa.pt \
  -p image_topic:=/bluerov2/fls/image \
  -p device:=cuda:0 -p conf:=0.25 -p iou:=0.7 -p imgsz:=640
```

`device`는 `cpu`, `cuda:0`, 또는 GPU 인덱스 숫자를 받는다. `busy` 플래그로 추론 중
들어온 프레임은 드랍해 실시간성을 유지한다.
