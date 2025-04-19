from launch import LaunchDescription
from launch_ros.actions import Node # 그 노드안에 구성된 노드랑 다른 것임 다른데 그것은 로스 클라이언트에서 선언된것

def generate_launch_description():

    return LaunchDescription([
        Node(package = 'hello_ros2', executable = "simple_pub", output='screen'), 
        Node(package ='hello_ros2', executable = "simple_sub", output='screen')]) 
## 여기에 무엇을 넣을지가 중요하다. 여기서 리턴되는 값이 이것으로 정해져 있다. 가장 기본적인 구조
## excutable == 노드의 이름 임