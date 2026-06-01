#!/usr/bin/env python3
"""
[PC 전용] cam_yolo_node.py

원본 대비 변경점:
  - /image_raw (uncompressed) 구독 → /image_raw/compressed 로 변경
  - JPEG 디코딩 한 번 추가 (네트워크 절약 트레이드오프)
  - 발행 토픽은 동일: /image_yolo/compressed, /traffic_sign_topic

PC는 라즈베리파이보다 강력하므로 YOLO 추론 부담을 흡수.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from pathlib import Path

from p2_pkg.sign_detector import TrafficSignDetector


class CameraParserNode(Node):
    def __init__(self):
        super().__init__('cam_yolo_node')

        # ── 모델 ──
        BASE_DIR   = Path(__file__).resolve().parent
        MODEL_PATH = BASE_DIR / "best_n_model_v4_640p_2.pt"

        self.detector = TrafficSignDetector(
            model_path=str(MODEL_PATH),
            conf_threshold=0.40
        )

        # ── QoS ──
        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── 구독: 라즈베리파이가 발행하는 압축 이미지 (DDS over LAN) ──
        self.image_sub = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.image_callback,
            be_qos
        )

        # ── 발행 ──
        self.result_pub = self.create_publisher(String, 'traffic_sign_topic', 10)
        self.yolo_image_pub = self.create_publisher(
            CompressedImage, 'image_yolo/compressed', be_qos
        )

        # ── 발행 빈도 (대시보드용 디버그 영상) ──
        self.yolo_frame_count    = 0
        self.yolo_publish_interval = 2
        self.yolo_jpeg_quality   = 60

        # ── 신호 필터링 설정 (원본과 동일) ──
        self.valid_actions = {"STOP", "GO", "SPEED_LIMIT", "TURN_LEFT"}
        self.class_thresholds = {
            "STOP": 0.80, "GO": 0.80, "TURN_LEFT": 0.80, "SPEED_LIMIT": 0.92,
        }
        self.default_conf_threshold = 0.50
        self.roi_ratio = (0.10, 0.01, 0.90, 0.50)
        self.min_area_ratio = 0.010

        self.last_sent_action = None
        self.no_detection_count = 0
        self.reset_after_missing_frames = 10

        self.last_print_time = self.get_clock().now()

        self.get_logger().info('==========================================')
        self.get_logger().info(' YOLO 노드 시작 (PC 측)                   ')
        self.get_logger().info('   구독: /image_raw/compressed (from Pi)  ')
        self.get_logger().info('   발행: /traffic_sign_topic              ')
        self.get_logger().info('   발행: /image_yolo/compressed           ')
        self.get_logger().info('==========================================')

    def image_callback(self, msg):
        # ── JPEG 디코드 (압축 이미지를 BGR ndarray 로) ──
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return
        except Exception as e:
            self.get_logger().error(f'JPEG 디코드 실패: {e}')
            return

        # ── 객체 탐지 ──
        all_detections = self.detector.detect_all(frame)
        detections = self.filter_detections_by_roi(all_detections, frame)

        if not detections:
            self.no_detection_count += 1
            if self.no_detection_count >= self.reset_after_missing_frames:
                self.last_sent_action = None
            debug_frame = self.make_debug_frame(frame, detections, "NO VALID DETECTION")
            self.publish_yolo_debug_image(debug_frame)
            return

        self.no_detection_count = 0

        main_action = self.detector.get_main_action(detections)
        action_hint = main_action.get("action_hint", None)

        if action_hint not in self.valid_actions:
            debug_frame = self.make_debug_frame(frame, detections, "INVALID ACTION")
            self.publish_yolo_debug_image(debug_frame)
            return

        if action_hint == self.last_sent_action:
            debug_frame = self.make_debug_frame(frame, detections, f"ALREADY SENT: {action_hint}")
            self.publish_yolo_debug_image(debug_frame)
            return

        # 새 액션만 발행
        out = String()
        out.data = action_hint
        self.result_pub.publish(out)
        self.last_sent_action = action_hint

        debug_frame = self.make_debug_frame(frame, detections, f"SENT: {action_hint}")
        self.publish_yolo_debug_image(debug_frame)

        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_print_time).nanoseconds / 1e9
        if elapsed > 0.5:
            self.get_logger().info(f'[YOLO] action -> {action_hint}')
            self.last_print_time = current_time

    # ── 이하 원본과 동일 ────────────────────────────────────────
    def make_debug_frame(self, frame, detections, status_text):
        debug_frame = frame.copy()
        debug_frame = self.detector.draw_all(debug_frame, detections)
        x1, y1, x2, y2 = self.get_roi_box(frame)
        cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(debug_frame, status_text, (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        return debug_frame

    def publish_yolo_debug_image(self, debug_frame):
        self.yolo_frame_count += 1
        if self.yolo_frame_count % self.yolo_publish_interval != 0:
            return
        success, encoded = cv2.imencode(
            '.jpg', debug_frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.yolo_jpeg_quality]
        )
        if not success:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self.yolo_image_pub.publish(msg)

    def get_roi_box(self, frame):
        h, w = frame.shape[:2]
        rx1, ry1, rx2, ry2 = self.roi_ratio
        return int(w*rx1), int(h*ry1), int(w*rx2), int(h*ry2)

    def get_detection_confidence(self, d):
        for attr in ["confidence", "conf", "score"]:
            if hasattr(d, attr):
                return float(getattr(d, attr))
        if isinstance(d, dict):
            for k in ["confidence", "conf", "score"]:
                if k in d:
                    return float(d[k])
        return 1.0

    def filter_detections_by_roi(self, detections, frame):
        rx1, ry1, rx2, ry2 = self.get_roi_box(frame)
        out = []
        for d in detections:
            if d.action_hint not in self.valid_actions:
                continue
            conf = self.get_detection_confidence(d)
            thr  = self.class_thresholds.get(d.action_hint, self.default_conf_threshold)
            if conf < thr:
                continue
            cx, cy = d.center["x"], d.center["y"]
            if not (rx1 <= cx <= rx2 and ry1 <= cy <= ry2):
                continue
            if d.area_ratio < self.min_area_ratio:
                continue
            out.append(d)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = CameraParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
