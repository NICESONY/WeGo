import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ku_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 디렉터리 안의 모든 .launch.py 파일을 설치하기 위한 설정
        ('share/' + package_name + '/launch',
         glob(os.path.join('launch', '*.launch.py'))),
        # urdf 디렉터리 안의 모든 .urdf파일을 올바른 urdf 경로로 설치
        ('share/' + package_name + '/urdf',
        glob(os.path.join('urdf', '*.urdf'))),
        # urdf 디렉터리 안의 모든 .xacro파일을 올바른 urdf 경로로 설치
        ('share/' + package_name + '/urdf',
        glob(os.path.join('urdf', '*.xacro'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='son',
    maintainer_email='songunhee5426@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
