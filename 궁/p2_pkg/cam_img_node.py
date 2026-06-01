#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
from cv_bridge import CvBridge
import time


class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')

        self.ENABLE_RAW_COMPRESSED = True   # 원본과 압축 이미지 동시 발행 여부 (웹 대시보드용 압축 이미지 추가)    
                
        # =========================
        # QoS 설정 - BEST_EFFORT
        # =========================
        image_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # =========================
        # Publisher
        # =========================
        # YOLO 노드용 원본 이미지 토픽
        self.publisher_ = self.create_publisher(
            Image,
            'image_raw',
            image_qos_profile
        )

        # 웹 대시보드용 압축 이미지 토픽
        if self.ENABLE_RAW_COMPRESSED:
            self.compressed_publisher_ = self.create_publisher(
                CompressedImage,
                'image_raw/compressed',
                image_qos_profile
            )

        # =========================
        # 웹 대시보드 발행 최적화
        # =========================
        self.frame_count = 0
        self.web_publish_interval = 3 # 15FPS 기준 3프레임마다 1번 → 약 5FPS
        self.jpeg_quality = 55

        self.br = CvBridge()

        # =========================
        # 카메라 설정
        # =========================
        DEVICE_ID = 0

        self.cap = cv2.VideoCapture(DEVICE_ID, cv2.CAP_V4L2)
        time.sleep(0.5)

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 15)

        if not self.cap.isOpened():
            self.get_logger().error('Ubuntu V4L2: 하드웨어 카메라 장치를 열 수 없습니다!')
            return

        self.timer = self.create_timer(1.0 / 15.0, self.timer_callback)
        
        if self.ENABLE_RAW_COMPRESSED:
            self.get_logger().info(
                '카메라 하드웨어 발행 노드가 정상 가동되었습니다. '
                'BEST_EFFORT QoS '
                '(/image_raw: 15FPS, /image_raw/compressed: 약 5FPS)'
            )
        else:
            self.get_logger().info(
                '카메라 하드웨어 발행 노드가 정상 가동되었습니다. '
                'BEST_EFFORT QoS '
                '(/image_raw: 15FPS, raw compressed OFF)'
            )
            
    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.get_logger().warn('카메라 장치로부터 비디오 프레임을 가져오지 못했습니다.')
            return

        now = self.get_clock().now().to_msg()

        # =========================
        # 1. YOLO 노드용 raw Image는 매 프레임 발행
        # =========================
        img_msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = now
        self.publisher_.publish(img_msg)

        # =========================
        # 2. 웹 대시보드용 compressed Image는 일부 프레임만 발행
        # =========================
        if not self.ENABLE_RAW_COMPRESSED:
            return
        
        self.frame_count += 1
        if self.frame_count % self.web_publish_interval != 0:
            return

        success, encoded_image = cv2.imencode(
            '.jpg',
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )

        if not success:
            self.get_logger().warn('JPEG 인코딩 실패')
            return

        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = now
        compressed_msg.format = "jpeg"
        compressed_msg.data = encoded_image.tobytes()

        self.compressed_publisher_.publish(compressed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'cap') and node.cap.isOpened():
            node.cap.release()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()