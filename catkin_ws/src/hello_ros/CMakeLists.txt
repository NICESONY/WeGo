cmake_minimum_required(VERSION 3.0.2)
project(hello_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  dynamic_reconfigure   # ← 반드시 포함
  std_msgs
  message_generation      # 메시지 • 서비스 • 액션 생성용
  actionlib_msgs          # 액션 메시지 타입
  actionlib               # (C++ 액션 클라이언트/서버 사용 시)
)

###################################
## Python packaging 설정        ##
###################################
# setup.py가 있는 경우 반드시 generate_messages()보다 먼저 호출해야 합니다.
catkin_python_setup()



###################################
## Dynamic-reconfigure (.cfg)    ##
###################################
generate_dynamic_reconfigure_options(
  cfg/SimpleParameter.cfg        # cfg/ 디렉터리에 여러 개면 줄로 추가
)


###################################
## Declare ROS messages          ##
###################################
add_message_files(
  FILES
  MyTopic.msg
)


###################################
## Declare ROS service          ##
###################################
add_service_files(
  FILES
  AddTwoInts.srv
)



###################################
## Declare ROS action         ##
###################################
add_action_files(
  FILES
  TimeWait.action
)


## Generate added messages with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    roscpp
    rospy
    std_msgs
    dynamic_reconfigure       # ← 추가
    actionlib_msgs            # ← 액션을 쓰면 함께 추가
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

###################################
## Install Python scripts & launch##
###################################
catkin_install_python(PROGRAMS
  scripts/hello
  scripts/move_Turtle
  scripts/mtpub
  scripts/mtsub
  scripts/msub2
  scripts/msub
  scripts/mpub
  scripts/simpleServiceServer
  scripts/simpleServiceClient
  scripts/simpleServiceClient2
  scripts/simpleParameter
  scripts/simpleParameter2
  scripts/myTopicPub
  scripts/myTopicSub
  scripts/addServer
  scripts/addClient
  scripts/timeActionServer
  scripts/timeActionClient
  scripts/timeActionClient_unsyn
  scripts/circle_limo
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
  launch/
  param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
  PATTERN ".svn" EXCLUDE
)
