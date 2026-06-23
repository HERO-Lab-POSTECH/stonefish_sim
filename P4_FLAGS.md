## P4 후보 (P2 발견)
- `bezier_curve.py::BezierCurve.__init__`: tangents를 list로 받으면 `tangents[0]+tangents[1]`이 list concat(길이 6)이 되어 order=3/4 경로에서 np.dot shape 오류. assert는 len==3 list를 허용하나 내부 연산은 np.array만 정상. 수정안: 생성자에서 tangents/pnts를 np.asarray로 정규화. (동작 변경이라 P4에서 처리)
