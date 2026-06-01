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
    description='라즈베리파이 4 + PC 분산형 ROS 2 자율주행 패키지 '
                '(Pi: 카메라/ESP32 브릿지 / PC: YOLO/차선/FSM/대시보드)',
    license='TODO: License declaration',
    tests_require=['pytest'],

    # =========================================================================
    # Entry Points
    # =========================================================================
    entry_points={
        'console_scripts': [
            # ─────────────────────────────────────────────────────────────
            # [라즈베리파이 측] 카메라 + ESP32 브릿지
            # ─────────────────────────────────────────────────────────────
            'cam_image_node     = p2_pkg.cam_image_node:main',
            'bridge_node        = p2_pkg.bridge_node:main',

            # ─────────────────────────────────────────────────────────────
            # [PC 측] 비전 처리 + FSM + 대시보드
            # ─────────────────────────────────────────────────────────────
            'cam_yolo_node      = p2_pkg.cam_yolo_node:main',
            'cam_line_node      = p2_pkg.cam_line_node:main',
            'brain_node         = p2_pkg.brain_node:main',
            'dashboard_node     = p2_pkg.dashboard_node:main',

            # ─────────────────────────────────────────────────────────────
            # [선택] 디버깅용 단일 뷰어
            # ─────────────────────────────────────────────────────────────
            'cam_subscriber_node = p2_pkg.cam_subscriber_node:main',
        ],
    },
)
