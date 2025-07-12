# launch file about lane detection application

### lane_detection.launch
- 차선 인식 기능 동작을 위한 다수의 Node 실행하는 런치 파일
- LIMO 내부에서 차선 인식을 단독으로 동작 시키기 위해 사용
- 아래의 Node들을 포함
  - 차선 인식을 위한 `lane_detect.py`
  - 횡단 보도 인식을 위한 `crosswalk_detect.py`
  - 장애물 인식을 위한 `e_stop.py`
  - 제어를 위한 `control.py`
- 각각의 Node 실행 시, 이에 해당하는 Parameter Load
  - `lane_detect.py` - `lane_detection.yaml`
  - `crosswalk_detect.py` - `crosswalk.yaml`
  - `e_stop.py` - `e_stop.yaml`
  - `control.py` - `control.yaml`
- 이 외의 동작에 필요한 `Topic Name` 및 시각화를 위한 `Visualization` Parameter 변경 기능

### lane_detection_with_yolo.launch
- 차선 인식 기능 및 Object Detection (yolo)를 함께 동작 시키기 위한 런치 파일
- LIMO 외부에서 두 가지를 동시에 실행 시키기 위해 사용(권장하 지 않음)
- 아래의 두 가지 launch 파일 포함
  - 차선 인식 기능을 위한 `lane_detection.launch`
  - 표지판 및 신호등 인식을 위한 `yolov3-tiny-traffic.launch`