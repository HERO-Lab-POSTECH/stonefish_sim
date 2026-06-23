## P4 후보 (P2 발견)
- `bezier_curve.py::BezierCurve.__init__`: tangents를 list로 받으면 `tangents[0]+tangents[1]`이 list concat(길이 6)이 되어 order=3/4 경로에서 np.dot shape 오류. assert는 len==3 list를 허용하나 내부 연산은 np.array만 정상. 수정안: 생성자에서 tangents/pnts를 np.asarray로 정규화. (동작 변경이라 P4에서 처리)

## P4 후보 (P3.0 컨벤션 조사 발견)
- **노드명 중복**: `controllers/velocity_controller_node.py:55`와 `nodes/position_controller_node.py:55`가 둘 다 `super().__init__('pid_4dof_controller')`. 동시 실행 시 ROS2 고유 노드명 요구(RMW 강제)를 위반해 노드 등록 충돌. 근거: [rmw validate_node_name.c](https://github.com/ros2/rmw/blob/master/rmw/src/validate_node_name.c). 수정안: 각자 고유 이름(예: `velocity_controller`·`position_controller`)으로 초기화. 동작 변경(노드명 의존 토픽 네임스페이스 영향 가능)이라 P4에서 처리.
