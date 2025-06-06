from launch import LaunchDescription
from launch_ros.actions import Node # 그 노드안에 구성된 노드랑 다른 것임 다른데 그것은 로스 클라이언트에서 선언된것
from launch.actions import IncludeLaunchDescription , DeclareLaunchArgument#
from launch_ros.substitutions import  FindPackageShare
from launch.substitutions import PathJoinSubstitution  ,LaunchConfiguration

# sudo apt install ros-humble-urdf-launch 설치 필수 

## 실행 코드 및 경로 
# son@samson:~/WeGo_LIMO/colcon_ws/src/ku_description/urdf$ ros2 launch ku_description display.launch.py 

## 실행 코드 및 경로 
# son@samson:~/WeGo_LIMO/colcon_ws/src/ku_description/urdf$ ros2 launch ku_description display.launch.py model:=urdf/origin.urdf


def generate_launch_description():
    default_model_path = PathJoinSubstitution(["urdf", "myfirst.urdf"])
    model = DeclareLaunchArgument(
        name = "model",
         default_value= default_model_path,
          description= "myfirst urdf file" )
    return LaunchDescription([
        model,
        # Node(package = 'hello_ros2', executable = "simple_pub", output='screen'), 
        # Node(package ='hello_ros2', executable = "simple_sub", output='screen'),
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("urdf_launch"), 
                                 "launch", 
                                 "display.launch.py"]
            ),
             launch_arguments= {
                    "urdf_package" : "ku_description",
                    'urdf_package_path' : LaunchConfiguration("model")
            }.items() 

        )]) 
## excutable == 노드의 이름 임