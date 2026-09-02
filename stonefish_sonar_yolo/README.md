# stonefish_sonar_yolo

FLS 소나 이미지에 YOLO 추론을 수행해 탐지 결과를 발행하는 노드.

- 구독: `image_topic` (기본 `/bluerov2/fls/image`, `sensor_msgs/Image`)
- 발행: `sonar_yolo/detections` (`vision_msgs/Detection2DArray`),
  `sonar_yolo/annotated` (`sensor_msgs/Image`, 구독자가 있을 때만)

`Detection2DArray.header` 와 각 `Detection2D.header` 는 입력 이미지의 header 를
그대로 복사한다 — 소비자(`stonefish_slam`)가 **어느 소나 프레임의 탐지인지 stamp 로
매칭**하기 때문이다. 탐지가 0건인 프레임도 빈 배열로 발행한다: "아무것도 없었다"와
"`busy` 드랍으로 추론을 건너뛰었다"가 소비자 쪽에서 갈려야 한다.

필드 배치:

| 필드 | 내용 |
|:--|:--|
| `bbox.center.position.x/y` | bbox 중심 픽셀 (x=col=bearing 축, y=row=range bin 축) |
| `bbox.size_x/size_y` | bbox 폭·높이(픽셀) |
| `results[0].hypothesis.class_id` | 클래스 **인덱스의 십진 문자열**(`"0"`) — `vision_msgs` 4.1.1 의 `class_id` 가 string 이라 정수를 그대로 못 싣는다 |
| `results[0].hypothesis.score` | 신뢰도 |
| `id` | 사람이 읽는 클래스 이름(`"sofa"`) |

## 요구 사항

- apt: `ros-humble-vision-msgs` (4.1.1). `ros-humble-desktop` 에 안 들어 있다 —
  `package.xml` 에 선언돼 있으므로 `rosdep install --from-paths src` 로도 잡힌다.
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
