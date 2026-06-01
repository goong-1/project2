#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
from cv_bridge import CvBridge
import time
import os

# OpenCV 내부 에러 로그 숨김 처리
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.publisher_ = self.create_publisher(Image, 'image_raw', qos_profile)
        
        # 💡 문제 구간 1: qos_profile이 누락되어 대시보드 및 YOLO 노드와 QoS 불일치 발생
        self.compressed_publisher_ = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)
        
        self.br = CvBridge()
        DEVICE_ID = 0
        
        self.cap = cv2.VideoCapture(DEVICE_ID, cv2.CAP_V4L2)
        time.sleep(0.5)

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # 💡 문제 구간 2: MJPG 포맷 사용으로 리눅스 드라이버 단에서 "Corrupt JPEG data" 경고 폭주
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 15)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)

        if not self.cap.isOpened():
            self.get_logger().error('Ubuntu V4L2: 하드웨어 카메라 장치를 열 수 없습니다!')
            return

        self.timer = self.create_timer(1.0 / 15.0, self.timer_callback)
        self.get_logger().info('카메라 하드웨어 리퍼블리셔 노드가 정상 가동되었습니다. (Raw & Compressed 동시 발행)')

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret and frame is not None:
            msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
            
            comp_msg = CompressedImage()
            comp_msg.header = msg.header
            comp_msg.format = "jpeg"
            
            _, compressed_img = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            comp_msg.data = compressed_img.tobytes()
            
            self.compressed_publisher_.publish(comp_msg)
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
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()