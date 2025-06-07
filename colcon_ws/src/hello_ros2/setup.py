from setuptools import find_packages, setup
import os
from glob import glob  # 특정한 규칙의 파일을 추출할 때 필요

package_name = 'hello_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament index에 패키지를 등록하기 위한 설정
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # 패키지의 package.xml을 설치하기 위한 설정
        ('share/' + package_name,
         ['package.xml']),
        # launch 디렉터리 안의 모든 .launch.py 파일을 설치하기 위한 설정
        ('share/' + package_name + '/launch',
         glob(os.path.join('launch', '*.launch.py'))),
        # param folder make 
        ('share/' + package_name + '/param',
         glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'user_interface'
                       ],    
    zip_safe=True,
    maintainer='son',
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_ros = hello_ros2.hello_ros:main',
            'move_turtle = hello_ros2.move_turtle:main',
            'simple_sub = hello_ros2.simple_sub:main',
            'simple_pub = hello_ros2.simple_pub:main',
            'simpleServiceServer = hello_ros2.simpleServiceServer:main',
            'simpleServiceClient = hello_ros2.simpleServiceClient:main',
            'simpleServiceServer2 = hello_ros2.simpleServiceServer2:main', # 명령어, 패키지, 파일 이름, main
            'simple_parameter = hello_ros2.simple_parameter:main', # 명령어, 패키지, 파일 이름, main
            'mutil_parameter = hello_ros2.mutil_parameter:main', # 명령어, 패키지, 파일 이름, main
            'change_color_client = hello_ros2.change_color_client:main', # 명령어, 패키지, 파일 이름, main
            'move_turtle_param = hello_ros2.move_turtle_param:main',
            'my_Topic_Pub = hello_ros2.my_Topic_Pub:main',
            "add_server = hello_ros2.add_server:main",
            "action_server = hello_ros2.action_server:main",
            "action_client = hello_ros2.action_client:main",
            "move_limo = hello_ros2.move_limo:main",
            "publish_map = hello_ros2.publish_map:main",
            "patrol = hello_ros2.patrol:main",
            "dynamic_tf = hello_ros2.dynamic_tf:main",
            "tf_listener = hello_ros2.tf_listener:main",
            "follow_listener = hello_ros2.follow_listener:main",
            "dynamic_tf_turtlesim = hello_ros2.dynamic_tf_turtlesim:main",
            
        ],
    },
)
