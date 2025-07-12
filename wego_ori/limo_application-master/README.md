# limo_application
LIMO Application by WeGo-Robotics

### Outline
* LIMO에 사용 가능한 Application
* Ubuntu 18.04, Python 2.7, OpenCV 4.1.1 사용
* /cmd_vel, /scan Topic 사용
* 실시간 파라미터 변경을 위한 Dynamic Reconfigure 포함
* 동작을 위한 launch파일 포함

### How to use
* IP주소 확인하는 방법
  * Terminal에서 아래 명령어 입력
  * `$ hostname -I`
  * LIMO와 연결된 PC의 IP주소를 모두 확인
  * LIMO에서 출력되는 값 중 첫 번째 값을 기록하였다가, 아래 `<<LIMO_IP_ADDRESS>>` 부분에 입력
  * 연결된 PC에서 출력되는 값 중 첫 번째 값을 기록하였다가, 아래 `<<CONNECTED_PC_IP_ADDRESS>>` 부분에 입력
* 차선 인식
  * LIMO (Jetson Nano)
    * 모든 터미널 공통 사항
      * `$ export ROS_MASTER_URI="http://<<LIMO_IP_ADDRESS>>:11311`
      * `$ export ROS_IP=<<LIMO_IP_ADDRESS>>`
    * Terminal 1
      * `$ roslaunch limo_bringup limo_start.launch`
    * Terminal 2
      * `$ roslaunch astra_camera dabai_u3.launch`
    * Terminal 3
      1. `$ cd catkin_ws/src`
      2. `$ git clone https://github.com/WeGo-Robotics/yolov3-pytorch-ros.git`
      3. `$ git clone https://github.com/WeGo-Robotics/limo_application.git`
      4. `$ cd .. && catkin_make`
      5. `$ source devel/setup.bash`
      6. `$ roslaunch limo_application lane_detection.launch`
  * 연결된 PC
    * Terminal 1
      * `$ export ROS_MASTER_URI="http://<<LIMO_IP_ADDRESS>>:11311`
      * `$ export ROS_IP=<<CONNECTED_PC_IP_ADDRESS>>`
      * `$ roslaunch yolov3-ros yolov3-tiny-traffic.launch`



