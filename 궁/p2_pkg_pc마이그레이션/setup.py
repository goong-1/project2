from setuptools import find_packages, setup

package_name = 'p2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pbg',
    maintainer_email='pbg@todo.todo',
    description='라즈베리파이 4 + PC 분산형 ROS 2 자율주행 패키지',
    license='TODO: License declaration',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            # [라즈베리파이] 카메라 발행
            'cam_image_node     = p2_pkg.cam_image_node:main',

            # [라즈베리파이] ESP32 브릿지
            'bridge_node        = p2_pkg.bridge_node:main',

            # [PC] 차선 감지
            'cam_line_node      = p2_pkg.cam_line_node:main',

            # [PC] YOLO 추론
            'cam_yolo_node      = p2_pkg.cam_yolo_node:main',

            # [PC] FSM 제어
            'brain_node         = p2_pkg.brain_node:main',

            # [PC] 웹 대시보드
            'dashboard_node     = p2_pkg.dashboard_node:main',
        ],
    },
)
