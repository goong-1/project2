from setuptools import find_packages, setup

# 패키지 이름을 정의합니다. 상위 폴더 및 워크스페이스 명세와 일치해야 합니다.
package_name = 'p2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS 2 자원을 시스템 환경에 등록하기 위한 인덱스 설정
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml 파일을 share 디렉토리로 복사하여 ROS 2 패키지로 인식하게 만듭니다.
        ('share/' + package_name, ['package.xml']),
        
        ('lib/python3.12/site-packages/' + package_name, ['p2_pkg/cam_yolo.pt']),
        ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pbg',
    maintainer_email='pbg@todo.todo',
    description='라즈베리파이 4 하드웨어 최적화 및 모터·라이다 통합형 ROS 2 자율주행 패키지',
    license='TODO: License declaration',
    tests_require=['pytest'],
    
    # =========================================================================
    # [핵심] Entry Points (콘솔 스크립트 진입점 설정)
    # =========================================================================
    # 터미널에서 'ros2 run p2_pkg 노드이름'으로 실행할 때 매칭되는 진입점 명세입니다.
    # 사용하지 않게 된 구 motor_driver_node 및 lidar_parser_node는 제외되었습니다.
    entry_points={
        'console_scripts': [
            'bridge_node = p2_pkg.bridge_node:main',
            'brain_node = p2_pkg.brain_node:main',
            'cam_line_node = p2_pkg.cam_line_node:main',
            'cam_yolo_node = p2_pkg.cam_yolo_node:main',
            'cam_image_node = p2_pkg.cam_image_node:main',
            'dashboard_node = p2_pkg.dashboard_node:main',
            
            
        ],
    },
)