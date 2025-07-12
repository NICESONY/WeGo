# python scripts about lane detection application

### lane_detect.py
- `ROS` 및 `Camera` 기반 차선 인식 알고리즘
- 인식 프로세스
  1. `CompressedImage` type 이미지 Subscribe
  2. `CvBridge`를 이용하여 `OpenCV` 이미지로 변경
  3. 이미지 ROI 지정 (`imageCrop`)
  4. 특정 색 영역 검출 (`colorDetect`)
  5. 검출된 색 영역의 모멘트를 통해 중심 픽셀 계산 (`calcLaneDistance`)
  6. 결과로 얻어진 거리 값 Publish
- Subscibe Topic
  - `~image_topic_name` (`sensor_msgs`/`CompressedImage`) - 실제 입력으로 들어가는 Topic 이름
- Publish Topic
  - `/limo/lane_x` (`std_msgs`/`Int32`) - 최종 검출된 차선 중심의 x 픽셀 위치
- Parameters
  - `~image_topic_name` (default : `/camera/rgb/image_raw/compressed`) - Subscribe할 CompressedImage type의 Topic 이름
  - `visualization` (default : `False`) - 검출 중간 과정의 결과 출력 여부

### crosswalk_detect.py
- `ROS` 및 `Camera` 기반 횡단 보도 인식 알고리즘
- 인식 프로세스
  1. `CompressedImage` type 이미지 Subscribe
  2. `CvBridge`를 이용하여 `OpenCV` 이미지로 변경
  3. 이미지 ROI 지정 (`imageCrop`)
  4. 특정 색 영역 검출 (`colorDetect`)
  5. 검출된 색 영역의 Edge 검출 (`edgeDetect`)
  6. `houghTransform`을 이용하여, 검출된 Edge에서 확인되는 직선 검출 (`houghLineDetect`)
  7. 검출된 직선의 개수를 확인하여, 일정 이상이면 검출된 색 영역의 모멘트를 계산하여 중심 좌표 계산 (`calcCrossWalkDistance`)
  8. 결과로 얻어진 거리 값 Publish
- Subscibe Topic
  - `~image_topic_name` (`sensor_msgs`/`CompressedImage`) - 실제 입력으로 들어가는 Topic 이름
- Publish Topic
  - `/limo/crosswalk_y` (`std_msgs`/`Int32`) - 최종 검출된 횡단 보도 중심의 y 픽셀 위치
- Parameters
  - `~image_topic_name` (default : `/camera/rgb/image_raw/compressed`) - Subscribe할 CompressedImage type의 Topic 이름
  - `visualization` (default : `False`) - 검출 중간 과정의 결과 출력 여부

### e_stop.py
- `ROS` 및 `LiDAR` 기반 장애물 인식 알고리즘
- 인식 프로세스
  1. `LaserScan` type 이미지 Subscribe
  2. `CvBridge`를 이용하여 `OpenCV` 이미지로 변경
  3. 이미지 ROI 지정 (`imageCrop`)
  4. 특정 색 영역 검출 (`colorDetect`)
  5. 검출된 색 영역의 Edge 검출 (`edgeDetect`)
  6. `houghTransform`을 이용하여, 검출된 Edge에서 확인되는 직선 검출 (`houghLineDetect`)
  7. 검출된 직선의 개수를 확인하여, 일정 이상이면 검출된 색 영역의 모멘트를 계산하여 중심 좌표 계산 (`calcCrossWalkDistance`)
  8. 결과로 얻어진 거리 값 Publish
- Subscibe Topic
  - `~lidar_topic_name` (`sensor_msgs`/`LaserScan`) - 실제 입력으로 들어가는 Topic의 이름
- Publish Topic
  - `/limo/lidar_warning` (`std_msgs`/`String`) - 장애물이 있어서 위험한지("Warning"), 없어서 안전한지의 결과를 출력 ("Safe")
- Parameters
  - `~lidar_topic_name` (default : `/scan`) - Subscribe할 LaserScan type의 Topic 이름

### control.py
- `ROS` 기반의 제어 코드
- 차선 인식, 횡단 보도 인식, YOLO 기반 객체 검출, 장애물 검출 결과를 이용하여 전체 주행
- 진행 프로세스
  - 기본적으로 차선을 따라서 주행
  - 정지선 및 신호등(빨강 or 노랑) 신호등 인식 시 정지
  - 천천히 표지판 인식 시 속도를 기준 속도의 절반으로 설정
  - 일정 크기 이상의 보행자 표지판 인식 시 5초간 정지
  - 차량 정면에 장애물이 있을 경우 없어질 때까지 대기
  - 위 과정을 경우에 따라 나누어서 선속도, 회전속도, 조향각도 제어
- Subscibe Topic
  - `limo_status` (`limo_base`/`LimoStatus`) - LIMO의 모션 모델에 대한 정보 확인에 사용
  - `/limo/lane_x` (`std_msgs`/`Int32`) - 최종 검출된 차선의 x 픽셀 위치
  - `/limo/lidar_warning` (`std_msgs`/`String`) - 장애물이 있어서 위험한지("Warning"), 없어서 안전한지의 결과를 출력 ("Safe")
  - `/limo/crosswalk_y` (`std_msgs`/`Int32`) - 최종 검출된 횡단 보도 중심의 y 픽셀 위치
  - `/limo/traffic_light` (`object_msgs`/`ObjectArray`) - YOLO 기반 객체 검출 결과 (객체의 종류 및 좌상단, 우하단 픽셀 좌표 포함)
- Publish Topic
  - `cmd_vel` (`geometry_msgs`/`Twist`) - LIMO 제어를 위한 Topic
- Parameters
  - `~lidar_topic_name` (default : `/scan`) - Subscribe할 LaserScan type의 Topic 이름


