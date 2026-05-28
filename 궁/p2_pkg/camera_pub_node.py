#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
from cv_bridge import CvBridge
import time

class CameraPublisherNode(Node):
    def __init__(self):
        # ROS 2 노드 이름을 'camera_publisher_node'로 초기화합니다.
        super().__init__('camera_publisher_node')

        # [QoS 설정] 라즈베리파이 가상 네트워크 환경에서 데이터 유실을 방지하기 위한 신뢰성 규칙 정의
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 'video_frames'라는 이름의 토픽으로 이미지 메시지를 발행하는 Publisher 생성
        self.publisher_ = self.create_publisher(Image, 'video_frames', qos_profile)
        
        # OpenCV 이미지와 ROS 2 메시지 포맷 간 변환을 담당하는 CvBridge 객체 생성
        self.br = CvBridge()

        # 카메라 디바이스 ID 설정 (기본 0번)
        DEVICE_ID = 0
        
        # 1. 카메라 하드웨어 오픈 (V4L2 백엔드 명시)
        self.cap = cv2.VideoCapture(DEVICE_ID, cv2.CAP_V4L2)
        
        # 2. 드라이버 세팅 안정화를 위해 0.5초 대기
        time.sleep(0.5)

        # [추가 최적화] 내부 하드웨어 버퍼 크기를 1로 고정 (프레임 누적 방지)
        # 이 설정을 해야 연산이 밀려도 버퍼에 영상이 쌓이지 않고 즉시 최신 프레임을 가져옵니다.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # [라즈베리파이 4 최적화 필수] 해상도를 320x240으로 낮춰 이미지 처리 버퍼 오버헤드 최소화
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 15)       # 초당 15프레임 제한으로 CPU 과부하 방지
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)  # 카메라 하드웨어 입력 버퍼 크기 최소화하여 레이턴시 차단

        if not self.cap.isOpened():
            self.get_logger().error('Ubuntu V4L2: 하드웨어 카메라 장치를 열 수 없습니다!')
            return

        # 15 FPS 주기(약 0.066초 단위)로 타이머 콜백 함수를 호출하도록 타이머 등록
        self.timer = self.create_timer(1.0 / 15.0, self.timer_callback)
        self.get_logger().info('카메라 하드웨어 발행 노드가 정상 가동되었습니다. (320x240 15FPS MJPG)')

    def timer_callback(self):
        # 카메라로부터 프레임 1장 획득
        ret, frame = self.cap.release_or_read() if hasattr(self.cap, 'release_or_read') else self.cap.read()
        
        if ret and frame is not None:
            # OpenCV BGR8 포맷 이미지를 ROS 2 표준 Image 메시지 데이터로 변환 후 토픽 발행
            msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('카메라 장치로부터 비디오 프레임을 가져오지 못했습니다.')

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap.isOpened():
            node.cap.release() # 프로그램 종료 시 하드웨어 리소스 해제
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()