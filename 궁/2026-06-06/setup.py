import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'p2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    package_data={
        package_name: ['*.pt'],
    },

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],

    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='pbg',
    maintainer_email='pbg@todo.todo',
    description='라즈베리파이 4 하드웨어 최적화 및 모터·라이다 통합형 ROS 2 자율주행 패키지',
    license='TODO: License declaration',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'bridge_node = p2_pkg.bridge_node:main',
            'brain_node = p2_pkg.brain_node:main',
            'cam_line_node = p2_pkg.cam_line_node:main',
            'cam_yolo_node = p2_pkg.cam_yolo_node:main',
            'cam_image_node = p2_pkg.cam_image_node:main',
            'lidar_node = p2_pkg.lidar_node:main',
            'dashboard_node = p2_pkg.dashboard_node:main',
        ],
    },
)
