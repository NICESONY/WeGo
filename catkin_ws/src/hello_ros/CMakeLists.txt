cmake_minimum_required(VERSION 3.0.2)
project(hello_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)

###################################
## Python packaging 설정        ##
###################################
# setup.py가 있는 경우 반드시 generate_messages()보다 먼저 호출해야 합니다.
catkin_python_setup()



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


## Generate added messages with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
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
  scripts/myTopicPub
  scripts/myTopicSub
  scripts/addServer
  scripts/addClient
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
  launch/
  param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
  PATTERN ".svn" EXCLUDE
)
