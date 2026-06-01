#!/usr/bin/env python3
"""
[라즈베리파이4 전용] cam_image_node.py

원본 아키텍처에서 변경된 점:
  - /image_raw (uncompressed) 발행 제거 → 네트워크 부담 감소
  - /image_raw/compressed 만 발행 (15 FPS)
  - JPEG 품질 70 (YOLO 추론 정확도 보존 + 적당한 압축)
  - PC에서 YOLO / 차선 / FSM 처리하므로 Pi는 카메라만 책임짐
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import time


class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')

        # ── QoS: BEST_EFFORT + depth 1 (오래된 프레임 버림 → 실시간성 ↑) ──
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.compressed_pub = self.create_publisher(
            CompressedImage,
            'image_raw/compressed',
            qos
        )

        # ── JPEG 품질: YOLO 추론용이라 너무 낮추면 정확도 떨어짐 ──
        self.jpeg_quality = 70

        # ── 카메라 ──
        DEVICE_ID = 0
        self.cap = cv2.VideoCapture(DEVICE_ID, cv2.CAP_V4L2)
        time.sleep(0.5)

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 15)

        if not self.cap.isOpened():
            self.get_logger().error('카메라 장치를 열 수 없습니다!')
            return

        # 15 FPS 로 동작
        self.timer = self.create_timer(1.0 / 15.0, self.timer_callback)

        self.get_logger().info('==========================================')
        self.get_logger().info(' 카메라 발행 노드 시작 (Pi → PC)         ')
        self.get_logger().info('   /image_raw/compressed  15FPS  Q=70    ')
        self.get_logger().info('==========================================')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('카메라 프레임 가져오기 실패')
            return

        success, encoded = cv2.imencode(
            '.jpg', frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not success:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = encoded.tobytes()

        self.compressed_pub.publish(msg)


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
