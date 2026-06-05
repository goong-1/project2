#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

os.environ['ROS_DOMAIN_ID'] = '30'

TARGET_IP_1 = "192.168.0.125"
TARGET_IP_2 = "192.168.0.110"
TARGET_IP_3 = "192.168.0.155"


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        self.image_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, qos_profile_sensor_data)
        self.state_sub = self.create_subscription(String, '/control_state', self.state_callback, 10)

        self.vision_pub = self.create_publisher(String, '/vision_status', 10)

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.image_pub = self.create_publisher(CompressedImage, '/image_line/compressed', best_effort_qos)

        self.bridge = CvBridge()

        self.lane_width            = 450
        self.last_valid_target_x   = None
        self.current_control_state = "CRUISE"
        self.last_sent_cmd         = ""

        self.prev_left_top  = None
        self.prev_left_bot  = None
        self.prev_right_top = None
        self.prev_right_bot = None

        self.prev_avoid_direction = -1

    def state_callback(self, msg):
        try:
            self.current_control_state, self.last_sent_cmd = msg.data.split('|')
        except ValueError:
            pass

    def filter_by_angle(self, mask, min_area=300, max_area=8000,
                        min_angle=25.0, max_angle=65.0):
        result = np.zeros_like(mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area or area > max_area:
                continue
            if len(cnt) < 5:
                x, y, w, h = cv2.boundingRect(cnt)
                angle = np.degrees(np.arctan2(h, w + 1e-5))
            else:
                pts = cnt.reshape(-1, 2).astype(np.float32)
                _, eigenvectors = cv2.PCACompute(pts, mean=None)
                vx, vy = eigenvectors[0]
                angle = np.degrees(np.arctan2(abs(vy), abs(vx)))
            if min_angle <= angle <= max_angle:
                cv2.drawContours(result, [cnt], -1, 255, thickness=cv2.FILLED)
        return result

    def extract_black_lane(self, roi):
        """
        CLAHE로 대비 향상 → adaptiveThreshold로 어두운 영역 추출
        단순 HSV 마스킹 대신 조명 변화에 강인한 방식
        """
        # 1. 그레이스케일 변환
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # 2. CLAHE로 대비 향상 (조명 불균일 보정)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray_clahe = clahe.apply(gray)

        # 3. adaptiveThreshold로 어두운 영역 추출
        # ADAPTIVE_THRESH_GAUSSIAN_C: 주변 픽셀 가중 평균 기준
        # THRESH_BINARY_INV: 어두운 부분 = 흰색(255)
        adaptive = cv2.adaptiveThreshold(
            gray_clahe,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            blockSize=31,   # 주변 참조 영역 크기 (홀수) — 클수록 넓은 영역 기준
            C=10            # 임계값 보정값 — 클수록 더 어두운 것만 추출
        )

        # 4. 노이즈 제거 (작은 점 제거)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        adaptive = cv2.morphologyEx(adaptive, cv2.MORPH_OPEN, kernel)

        return adaptive, gray_clahe

    def fit_line(self, points, roi_h):
        if len(points) < 2:
            return -1, -1
        pts = np.array(points)
        try:
            a, b = np.polyfit(pts[:, 1], pts[:, 0], 1)
            return int(b), int(a * roi_h + b)
        except Exception:
            return -1, -1

    def detect_single_lane(self, mask, roi_w, roi_h, side):
        edges = cv2.Canny(mask, 50, 150)
        lines = cv2.HoughLinesP(
            edges, rho=1, theta=np.pi/180,
            threshold=30, minLineLength=30, maxLineGap=20
        )

        candidates = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 == x1:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                if abs(slope) < 0.3:
                    continue

                if side == 'left':
                    if slope < 0 and max(x1, x2) < roi_w * 0.75:
                        pts = [(x1, y1), (x2, y2)]
                        x_top, x_bot_fitted = self.fit_line(pts, roi_h)
                        if x_bot_fitted != -1:
                            candidates.append((x_bot_fitted, pts))
                else:
                    if slope > 0 and min(x1, x2) > roi_w * 0.25:
                        pts = [(x1, y1), (x2, y2)]
                        x_top, x_bot_fitted = self.fit_line(pts, roi_h)
                        if x_bot_fitted != -1:
                            candidates.append((x_bot_fitted, pts))

        if not candidates:
            return -1, -1

        if side == 'left':
            candidates.sort(key=lambda c: c[0])
            best_points = candidates[0][1]
        else:
            candidates.sort(key=lambda c: c[0], reverse=True)
            best_points = candidates[0][1]

        return self.fit_line(best_points, roi_h)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            roi_h, roi_w = cv_image.shape[:2]

            # ── 1. ROI 다각형 1채널 마스크 생성 ──
            roi_vertices = np.array([[
                (0,   500),
                (100, 200),
                (540, 200),
                (640, 500)
            ]], dtype=np.int32)
            
            # 흑백(1채널) 마스크 생성 (AND 연산용)
            poly_mask_1ch = np.zeros((roi_h, roi_w), dtype=np.uint8)
            cv2.fillPoly(poly_mask_1ch, roi_vertices, 255)

            # 디버그 시각화 및 기존 변수 호환을 위한 roi 이미지 생성
            roi = cv2.bitwise_and(cv_image, cv_image, mask=poly_mask_1ch)

            # ── 2. 색공간 변환 (인공적인 검은 여백이 없는 원본 사용!) ──
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            if self.last_valid_target_x is None:
                self.last_valid_target_x = roi_w // 2

            # ── 3. 색상 추출 후 ROI 마스크 적용 ──
            # 노란선
            mask_yellow_raw = cv2.inRange(hsv, np.array([20,  65, 120]), np.array([41, 255, 255]))
            mask_yellow = cv2.bitwise_and(mask_yellow_raw, poly_mask_1ch)

            # 빨간선
            mask_red_raw = cv2.bitwise_or(
                cv2.inRange(hsv, np.array([0,   30,  49]), np.array([12,  255, 255])),
                cv2.inRange(hsv, np.array([170, 120,  70]), np.array([180, 255, 255]))
            )
            mask_red = cv2.bitwise_and(mask_red_raw, poly_mask_1ch)

            # 검은선: 원본(cv_image)을 넘겨서 외곽선 왜곡을 방지한 뒤, 마지막에 ROI 적용
            mask_black_raw_unmasked, gray_clahe = self.extract_black_lane(cv_image)
            mask_black_raw = cv2.bitwise_and(mask_black_raw_unmasked, poly_mask_1ch)

            # ── 4. 각도 필터로 그림자 제거 (이후 코드는 기존과 동일) ──
            mask_black = self.filter_by_angle(
                mask_black_raw, min_area=300, max_area=8000,
                min_angle=25.0, max_angle=65.0
            )
            
            # ... (이하 avoid_direction 계산 로직 동일) ...

            # ── avoid_direction 계산 ──
            yellow_left  = cv2.countNonZero(mask_yellow[:, :roi_w//2])
            yellow_right = cv2.countNonZero(mask_yellow[:, roi_w//2:])
            yellow_total = yellow_left + yellow_right

            if yellow_total == 0:
                avoid_direction = self.prev_avoid_direction
                mask_left_lane  = mask_black
                mask_right_lane = mask_black
            elif yellow_left > yellow_right:
                avoid_direction = 1
                self.prev_avoid_direction = avoid_direction
                mask_black[:, :roi_w//2] = 0   # 왼쪽 검은선 제거
                mask_left_lane  = mask_yellow
                mask_right_lane = mask_black
            else:
                avoid_direction = -1
                self.prev_avoid_direction = avoid_direction
                mask_black[:, roi_w//2:] = 0   # 오른쪽 검은선 제거
                mask_left_lane  = mask_black
                mask_right_lane = mask_yellow

            # ── 차선 검출 ──
            left_top,  left_bot  = self.detect_single_lane(mask_left_lane,  roi_w, roi_h, 'left')
            right_top, right_bot = self.detect_single_lane(mask_right_lane, roi_w, roi_h, 'right')

            if left_bot != -1:
                self.prev_left_top  = left_top
                self.prev_left_bot  = left_bot
            elif self.prev_left_bot is not None:
                left_top = self.prev_left_top
                left_bot = self.prev_left_bot

            if right_bot != -1:
                self.prev_right_top = right_top
                self.prev_right_bot = right_bot
            elif self.prev_right_bot is not None:
                right_top = self.prev_right_top
                right_bot = self.prev_right_bot

            if left_bot != -1 and right_bot != -1:
                measured = right_bot - left_bot
                if 100 < measured < roi_w:
                    self.lane_width = measured

            if left_bot != -1 and right_bot == -1:
                right_bot = left_bot  + self.lane_width
                right_top = left_top  + self.lane_width
            elif right_bot != -1 and left_bot == -1:
                left_bot  = right_bot - self.lane_width
                left_top  = right_top - self.lane_width

            # ── 차선 중앙 계산 ──
            if left_bot != -1 and right_bot != -1:
                center_bot = (left_bot  + right_bot) // 2
                center_top = (left_top  + right_top) // 2
            else:
                center_bot = self.last_valid_target_x
                center_top = self.last_valid_target_x

            self.last_valid_target_x = center_bot
            error = center_bot - (roi_w // 2)

            red_line_detected    = cv2.countNonZero(mask_red)    > 5000
            yellow_line_detected = yellow_total > 5000

            # ── vision_status 퍼블리시 ──
            status_msg = String()
            status_msg.data = (
                f"{error}|{1 if red_line_detected else 0}|"
                f"0|"
                f"{1 if yellow_line_detected else 0}|"
                f"{avoid_direction}"
            )
            self.vision_pub.publish(status_msg)

            # ── 디버그 시각화 ──
            overlay = roi.copy()
            cv2.polylines(overlay, [roi_vertices], isClosed=True, color=(255, 0, 0), thickness=2)

            overlay[mask_yellow > 0] = (0, 220, 220)
            overlay[mask_black  > 0] = (0, 255, 0)
            overlay[mask_red    > 0] = (0, 0, 255)

            if left_top != -1 and left_bot != -1:
                cv2.line(overlay, (left_top, 0), (left_bot, roi_h), (255, 80, 0), 2)
                cv2.putText(overlay, "L", (left_bot - 10, roi_h - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 80, 0), 1)

            if right_top != -1 and right_bot != -1:
                cv2.line(overlay, (right_top, 0), (right_bot, roi_h), (0, 80, 255), 2)
                cv2.putText(overlay, "R", (right_bot + 5, roi_h - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 80, 255), 1)

            cv2.line(overlay, (center_top, 0), (center_bot, roi_h), (0, 255, 0), 2)

            for y in range(0, roi_h, 12):
                cv2.line(overlay, (roi_w//2, y), (roi_w//2, min(y+6, roi_h)), (255, 255, 255), 1)

            mid_y = roi_h // 2
            cv2.arrowedLine(overlay, (roi_w//2, mid_y), (center_bot, mid_y),
                            (0, 200, 255), 2, tipLength=0.2)
            cv2.putText(overlay, f"err:{error:+.1f}",
                        (min(roi_w//2, center_bot) + 4, mid_y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)

            state_color = {"CRUISE": (255,255,255), "AVOID": (0,165,255), "STOP_TIMER": (0,0,255)}
            cv2.putText(overlay, f"State:{self.current_control_state}", (10, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        state_color.get(self.current_control_state, (255,255,255)), 2)
            cv2.putText(overlay, f"Cmd:{self.last_sent_cmd}", (10, 44),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.putText(overlay, f"RED:{'ON' if red_line_detected else 'off'}", (10, 64),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 255) if red_line_detected else (100, 100, 100), 1)
            avoid_text  = "AVOID:RIGHT(+1)" if avoid_direction == 1 else "AVOID:LEFT(-1)"
            avoid_color = (0, 220, 220) if avoid_direction == 1 else (255, 100, 100)
            cv2.putText(overlay, avoid_text, (10, 84),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, avoid_color, 1)
            cv2.putText(overlay,
                        f"L:{'YELLOW' if avoid_direction==1 else 'BLACK'}  "
                        f"R:{'BLACK' if avoid_direction==1 else 'YELLOW'}",
                        (10, 104), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 0), 1)

            # 하단 디버그 — CLAHE 결과도 표시
            def make_mask_vis(m, color, label):
                vis = np.zeros((m.shape[0], m.shape[1], 3), dtype=np.uint8)
                vis[m > 0] = color
                cv2.putText(vis, label, (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                return vis

            cell_w, cell_h = roi_w // 4, int(roi_h * 0.4)
            m_y  = cv2.resize(make_mask_vis(mask_yellow,    (0, 220, 220), "YELLOW"),   (cell_w, cell_h))
            m_b  = cv2.resize(make_mask_vis(mask_black,     (0, 255, 0),   "BLACK"),    (cell_w, cell_h))
            m_cl = cv2.resize(cv2.cvtColor(gray_clahe, cv2.COLOR_GRAY2BGR),             (cell_w, cell_h))
            cv2.putText(m_cl, "CLAHE", (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            m_r  = cv2.resize(make_mask_vis(mask_red,       (0, 0, 255),   "RED"),      (roi_w - cell_w*3, cell_h))

            mask_row    = cv2.resize(np.hstack([m_y, m_b, m_cl, m_r]), (roi_w, cell_h))
            debug_final = np.vstack([overlay, mask_row])

            compressed_msg = CompressedImage()
            compressed_msg.header.stamp    = self.get_clock().now().to_msg()
            compressed_msg.header.frame_id = "camera_frame"
            compressed_msg.format          = "jpeg"
            compressed_msg.data            = cv2.imencode('.jpg', debug_final)[1].tobytes()
            self.image_pub.publish(compressed_msg)

        except Exception as e:
            self.get_logger().error(f'image_callback 오류: {e}')
            import traceback
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
