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
            # 1. 통합 메인 두뇌 노드 (라이다 파싱 및 모터 직접 제어)
            'p2_proj_node = p2_pkg.bridge_node:main',
            
            # 2. 비전 인지 파서 노드 (YOLO 추론 엔진 및 정답 토픽 발행)
            'cam_yolo_node = p2_pkg.cam_yolo_node:main',
            
            # 3. 카메라 하드웨어 송신 노드 (V4L2 320x240 해상도 다이어트 버전)
            'cam_image_node = p2_pkg.cam_image_node:main',
            
            #'cam_yolo_node2 = p2_pkg.cam_yolo_node2:main',
            'dashboard_node = p2_pkg.dashboard_node:main',
            # 4. 카메라 영상 수신 및 디버그 뷰어 노드 (YOLO 디버그 화면 포함)
            'cam_subscriber_node = p2_pkg.cam_subscriber_node:main',
        ],
    },
)