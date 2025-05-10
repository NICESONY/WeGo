from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node # 그 노드안에 구성된 노드랑 다른 것임 다른데 그것은 로스 클라이언트에서 선언된것
import os
import glob
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('hello_ros2')

    # 파라미터 파일용 LaunchConfiguration
    # 2개 이상의 yaml파일 사용 가능
    turtlesim_yaml = LaunchConfiguration(
        'turtlesim_yaml',
        default=os.path.join(pkg_share, 'param', 'turtlesim.yaml')
    )
    move_yaml = LaunchConfiguration(
        'move_yaml',
        default=os.path.join(pkg_share, 'param', 'move_turtle_param.yaml')
    )



    return LaunchDescription([
        DeclareLaunchArgument("param_dir",
                              default_value = turtlesim_yaml,
                              description = 'turtlesim parameter dump file'),

        Node(package = 'turtlesim', 
             executable = "turtlesim_node", 
             output='screen', 
             parameters= [turtlesim_yaml]), 



        Node(package ='hello_ros2', 
             executable = "move_turtle_param", 
             output='screen',
             parameters =[move_yaml]),
             # < 런치 파일을 실행하는 도중에 값을 변경하고 싶으면 해당 코드 실행하면 된다>
             
             # Ros2 launch hello_ros2 moveTurtle.launch.py \
             # --ros-args \
             # -p move_turtle_param.lin_inc:=0.01 \
             # -p move_turtle_param.ang_vel:=1.5
        
             


        Node(package ='hello_ros2', 
             executable = "change_color_client", 
             output='screen')
             
             
             
             
             
             ])


## 여기에 무엇을 넣을지가 중요하다. 여기서 리턴되는 값이 이것으로 정해져 있다. 가장 기본적인 구조
## excutable == 노드의 이름 임