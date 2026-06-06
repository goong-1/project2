#!/usr/bin/env python3
"""
[PC 전용] cam_yolo_node.py

원본 대비 변경점:
  - /image_raw (uncompressed) 구독 → /image_raw/compressed 로 변경
  - JPEG 디코딩 한 번 추가 (네트워크 절약 트레이드오프)
  - 발행 토픽은 동일: /image_yolo/compressed, /traffic_sign_topic
  - 복수 감지 시 콤마 합산 → 라벨별 개별 발행으로 변경

PC는 라즈베리파이보다 강력하므로 YOLO 추론 부담을 흡수.
"""

import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from pathlib import Path

try:
    from p2_pkg.cam_yolo_detector import TrafficSignDetector
except ImportError:
    from cam_yolo_detector import TrafficSignDetector


class CameraParserNode(Node):
    def __init__(self):
        super().__init__('cam_yolo_node')

        # ── 모델 ──
        BASE_DIR   = Path(__file__).resolve().parent
        MODEL_PATH = BASE_DIR / "cam_yolo_7.pt"

        self.detector = TrafficSignDetector(
            model_path=str(MODEL_PATH),
            conf_threshold=0.25
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
        # ── 추론 시간(ms) 발행 ──
        self.perf_pub = self.create_publisher(Float32, 'yolo_perf', 10)

        # 추론 시간 평활화용 (최근 N개 이동평균)
        self._infer_ema = None
        self._ema_alpha = 0.2

        # ── 발행 빈도 (대시보드용 디버그 영상) ──
        self.yolo_frame_count    = 0
        self.yolo_publish_interval = 2
        self.yolo_jpeg_quality   = 60

        # ── 신호 필터링 설정 ──
        self.valid_actions = {"STOP", "GO", "SPEED_LIMIT", "TURN_LEFT"}
        self.class_thresholds = {
            "STOP": 0.70, "GO": 0.70, "TURN_LEFT": 0.70, "SPEED_LIMIT": 0.70,
        }
        self.default_conf_threshold = 0.50
        self.roi_ratio = (0.001, 0.200, 0.999, 0.850)

        # 라벨별 최소 면적 비율 (클수록 더 가까이 와야 인식)
        self.min_area_by_label = {
            "redlight":                 0.01,
            "traffic_light_red":        0.0029,
            "greenlight":               0.01,
            "traffic_left_light_green": 0.0029,
            "stop_sign":                0.0047,
            "limit_sign":               0.0047,
            "construction":             0.001,
        }
        self.default_min_area = 0.001

        # ── 직전 프레임 발행 라벨 세트 (개별 발행용) ──
        self.last_sent_labels = set()
        self.last_sent_time   = {}   # 라벨별 마지막 발행 시각
        self.resend_cooldown  = 3.0  # 동일 라벨 재발행 금지 시간(초)
        self.no_detection_count = 0
        self.reset_after_missing_frames = 10

        # ── greenlight 발행 전/후 허용 라벨 제어 ──
        self.greenlight_sent = False  # greenlight 발행 여부
        # greenlight 발행 전: 4개만 허용
        self.allowed_before_green = {"redlight", "greenlight", "stop_sign", "limit_sign"}
        # greenlight 발행 후: 위 4개 + traffic_light_red, traffic_left_light_green 추가
        self.allowed_after_green  = {"redlight", "greenlight", "stop_sign", "limit_sign",
                                     "traffic_light_red", "traffic_left_light_green"}

        self.last_print_time = self.get_clock().now()

        self.get_logger().info('==========================================')
        self.get_logger().info(' YOLO 노드 시작 (PC 측)                   ')
        self.get_logger().info('   구독: /image_raw/compressed (from Pi)  ')
        self.get_logger().info('   발행: /traffic_sign_topic              ')
        self.get_logger().info('   발행: /image_yolo/compressed           ')
        self.get_logger().info('   발행: /yolo_perf (추론시간 ms)          ')
        self.get_logger().info('==========================================')

    def image_callback(self, msg):
        # ── JPEG 디코드 ──
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return
        except Exception as e:
            self.get_logger().error(f'JPEG 디코드 실패: {e}')
            return

        # ── 객체 탐지 ──
        t0 = time.perf_counter()
        all_detections = self.detector.detect_all(frame)
        infer_ms = (time.perf_counter() - t0) * 1000.0

        if self._infer_ema is None:
            self._infer_ema = infer_ms
        else:
            self._infer_ema = (self._ema_alpha * infer_ms
                               + (1 - self._ema_alpha) * self._infer_ema)

        perf_msg = Float32()
        perf_msg.data = float(self._infer_ema)
        self.perf_pub.publish(perf_msg)

        detections = self.filter_detections_by_roi(all_detections, frame)

        if not detections:
            self.no_detection_count += 1
            if self.no_detection_count >= self.reset_after_missing_frames:
                self.last_sent_labels = set()
            debug_frame = self.make_debug_frame(frame, detections, "NO VALID DETECTION")
            self.publish_yolo_debug_image(debug_frame)
            return

        self.no_detection_count = 0

        # 현재 허용 라벨 목록 결정 (원본 라벨 기준)
        allowed = self.allowed_after_green if self.greenlight_sent else self.allowed_before_green

        # 원본 라벨 기준으로 먼저 필터 → 그 다음 merge
        merge = {"traffic_light_red": "redlight"}
        current_labels = set(
            merge.get(d.label, d.label)
            for d in detections
            if d.label in allowed
        )

        # 쿨다운이 지난 라벨만 개별 발행
        now = time.time()
        new_labels = set()
        for label in sorted(current_labels):
            last_t = self.last_sent_time.get(label, 0.0)
            if now - last_t >= self.resend_cooldown:
                out = String()
                out.data = label
                self.result_pub.publish(out)
                self.last_sent_time[label] = now
                new_labels.add(label)
                self.get_logger().info(f'[YOLO] 발행 → {label}  (추론 {self._infer_ema:.1f}ms)')
                if label == "greenlight":
                    self.greenlight_sent = True
                    self.get_logger().info('[YOLO] greenlight 발행 완료 → 이후 traffic_left_light_green 허용')

        self.last_sent_labels = current_labels

        status_text = ("SENT: " + ",".join(sorted(new_labels))) if new_labels else ("SEEN: " + ",".join(sorted(current_labels)))
        debug_frame = self.make_debug_frame(frame, detections, status_text)
        self.publish_yolo_debug_image(debug_frame)

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
            self.get_logger().warn('인코딩 실패!')
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
            print(f"{d.label}: area_ratio={d.area_ratio:.5f}")
            if d.action_hint not in self.valid_actions:
                continue
            conf = self.get_detection_confidence(d)
            thr  = self.class_thresholds.get(d.action_hint, self.default_conf_threshold)

            # ── 라벨별 min_area랑 실시간 area 비교 출력 ──
            min_area = self.min_area_by_label.get(d.label, self.default_min_area)
            passed_conf = conf >= thr
            cx, cy = d.center["x"], d.center["y"]
            passed_roi = (rx1 <= cx <= rx2 and ry1 <= cy <= ry2)
            passed_area = d.area_ratio >= min_area
            print(f"    {d.label:26s} conf={conf:.2f}(thr {thr:.2f}) "
                  f"area={d.area_ratio:.5f}(min {min_area:.5f}) "
                  f"{'✓통과' if (passed_conf and passed_roi and passed_area) else '✗걸림'}")

            if conf < thr:
                continue
            cx, cy = d.center["x"], d.center["y"]
            if not (rx1 <= cx <= rx2 and ry1 <= cy <= ry2):
                continue
            min_area = self.min_area_by_label.get(d.label, self.default_min_area)
            if d.area_ratio < min_area:
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
