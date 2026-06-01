#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge

import cv2
from pathlib import Path

from p2_pkg.sign_detector import TrafficSignDetector


class CameraParserNode(Node):
    def __init__(self):
        super().__init__('cam_yolo_node')

        self.br = CvBridge()

        # =========================
        # 모델 경로
        # =========================
        BASE_DIR = Path(__file__).resolve().parent
        MODEL_PATH = BASE_DIR / "best_n_model_v3_320p.pt"

        self.detector = TrafficSignDetector(
            model_path=str(MODEL_PATH),
            conf_threshold=0.40
        )

        # =========================
        # QoS 설정
        # =========================
        image_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # =========================
        # Subscriber
        # =========================
        self.image_sub = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            image_qos_profile
        )

        # =========================
        # Publisher
        # =========================
        self.result_pub = self.create_publisher(
            String,
            'traffic_sign_topic',
            10
        )

        self.yolo_image_pub = self.create_publisher(
            CompressedImage,
            'image_yolo/compressed',
            image_qos_profile
        )

        # =========================
        # YOLO 디버그 이미지 발행 설정
        # =========================
        self.yolo_frame_count = 0
        self.yolo_publish_interval = 2  # 2프레임마다 1번 발행
        self.yolo_jpeg_quality = 60

        # =========================
        # ROI / one-shot publish 설정
        # =========================
        self.valid_actions = {"STOP", "GO", "SPEED_LIMIT", "TURN_LEFT"}

        self.class_thresholds = {
            "STOP": 0.80,
            "GO": 0.80,
            "TURN_LEFT": 0.80,
            "SPEED_LIMIT": 0.92,
        }

        self.default_conf_threshold = 0.50

        # 화면 비율 기준 ROI: x1, y1, x2, y2
        self.roi_ratio = (0.10, 0.01, 0.90, 0.50)

        # 너무 작은 객체는 무시
        self.min_area_ratio = 0.010

        # 같은 액션 중복 발행 방지
        self.last_sent_action = None

        # 일정 시간 안 보이면 같은 액션 다시 보낼 수 있게 reset
        self.no_detection_count = 0
        self.reset_after_missing_frames = 10

        self.last_print_time = self.get_clock().now()

        self.get_logger().info('==================================================')
        self.get_logger().info(' YOLO 노드 시작: /image_raw 구독, /traffic_sign_topic 발행, /image_yolo/compressed 발행 ')
        self.get_logger().info('==================================================')

    def image_callback(self, msg):
        try:
            frame = self.br.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패: {str(e)}')
            return

        # 1. 전체 객체 탐지
        all_detections = self.detector.detect_all(frame)

        # 2. ROI / 크기 / confidence 기준 필터링
        detections = self.filter_detections_by_roi(all_detections, frame)

        # 3. 유효 감지 없음
        if not detections:
            self.no_detection_count += 1

            if self.no_detection_count >= self.reset_after_missing_frames:
                self.last_sent_action = None

            debug_frame = self.make_debug_frame(
                frame,
                detections,
                "NO VALID DETECTION"
            )

            self.publish_yolo_debug_image(debug_frame)
            return

        self.no_detection_count = 0

        # 4. 대표 액션 선택
        main_action = self.detector.get_main_action(detections)
        action_hint = main_action.get("action_hint", None)

        if action_hint not in self.valid_actions:
            debug_frame = self.make_debug_frame(
                frame,
                detections,
                "INVALID ACTION"
            )
            self.publish_yolo_debug_image(debug_frame)
            return

        # 5. 같은 액션이면 명령은 재발행하지 않지만, 디버그 화면은 계속 발행
        if action_hint == self.last_sent_action:
            debug_frame = self.make_debug_frame(
                frame,
                detections,
                f"ALREADY SENT: {action_hint}"
            )

            self.publish_yolo_debug_image(debug_frame)
            return

        # 6. 처음 본 액션만 발행
        string_msg = String()
        string_msg.data = action_hint
        self.result_pub.publish(string_msg)
        self.last_sent_action = action_hint

        debug_frame = self.make_debug_frame(
            frame,
            detections,
            f"SENT: {action_hint}"
        )

        self.publish_yolo_debug_image(debug_frame)

        # 로그 출력 제한
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.last_print_time).nanoseconds / 1e9

        if elapsed_time > 0.5:
            self.get_logger().info(
                f'[YOLO AI Result] FINAL ACTION HINT -> {action_hint}'
            )
            self.last_print_time = current_time

    def make_debug_frame(self, frame, detections, status_text):
        debug_frame = frame.copy()

        # YOLO bbox 그리기
        debug_frame = self.detector.draw_all(debug_frame, detections)

        # ROI 그리기
        x1, y1, x2, y2 = self.get_roi_box(frame)
        cv2.rectangle(
            debug_frame,
            (x1, y1),
            (x2, y2),
            (255, 0, 0),
            2
        )

        # 상태 텍스트 표시
        cv2.putText(
            debug_frame,
            status_text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )

        return debug_frame

    def publish_yolo_debug_image(self, debug_frame):
        self.yolo_frame_count += 1

        if self.yolo_frame_count % self.yolo_publish_interval != 0:
            return

        success, encoded_image = cv2.imencode(
            '.jpg',
            debug_frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.yolo_jpeg_quality]
        )

        if not success:
            self.get_logger().warn('YOLO 디버그 이미지 JPEG 인코딩 실패')
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = encoded_image.tobytes()

        self.yolo_image_pub.publish(msg)

    def get_roi_box(self, frame):
        h, w = frame.shape[:2]
        rx1, ry1, rx2, ry2 = self.roi_ratio

        x1 = int(w * rx1)
        y1 = int(h * ry1)
        x2 = int(w * rx2)
        y2 = int(h * ry2)

        return x1, y1, x2, y2

    def get_detection_confidence(self, d):
        for attr_name in ["confidence", "conf", "score"]:
            if hasattr(d, attr_name):
                return float(getattr(d, attr_name))

        if isinstance(d, dict):
            for key in ["confidence", "conf", "score"]:
                if key in d:
                    return float(d[key])

        return 1.0

    def filter_detections_by_roi(self, detections, frame):
        roi_x1, roi_y1, roi_x2, roi_y2 = self.get_roi_box(frame)

        filtered = []

        for d in detections:
            if d.action_hint not in self.valid_actions:
                continue

            conf = self.get_detection_confidence(d)
            threshold = self.class_thresholds.get(
                d.action_hint,
                self.default_conf_threshold
            )

            if conf < threshold:
                continue

            cx = d.center["x"]
            cy = d.center["y"]

            if not (roi_x1 <= cx <= roi_x2 and roi_y1 <= cy <= roi_y2):
                continue

            if d.area_ratio < self.min_area_ratio:
                continue

            filtered.append(d)

        return filtered


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