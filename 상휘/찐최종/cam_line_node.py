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

        # ── 차선 상태 관리 ──
        # 0: 미결정, 1: 1차선(노란=왼쪽, 검은=오른쪽), 2: 2차선(노란=오른쪽, 검은=왼쪽)
        self.current_lane = 0

    def state_callback(self, msg):
        try:
            state_part, self.last_sent_cmd = msg.data.split('|')
            self.current_control_state = state_part

            # control_handler에서 장애물 회피 완료 시 "LANE_CHANGE:1" 또는 "LANE_CHANGE:2" 전송
            if "LANE_CHANGE:" in state_part:
                try:
                    new_lane = int(state_part.split("LANE_CHANGE:")[1].strip())
                    self.current_lane = new_lane
                    # 차선 변경 시 이전 좌표 초기화
                    self.prev_left_top  = None
                    self.prev_left_bot  = None
                    self.prev_right_top = None
                    self.prev_right_bot = None
                    self.get_logger().info(f"차선 변경 수신 → current_lane = {self.current_lane}")
                except Exception:
                    pass
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
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray_clahe = clahe.apply(gray)
        adaptive = cv2.adaptiveThreshold(
            gray_clahe, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            blockSize=31, C=8
        )
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

            # ── 1. ROI 다각형 마스크 ──
            roi_vertices = np.array([[
                (0,   500),
                (100, 200),
                (540, 200),
                (640, 500)
            ]], dtype=np.int32)

            poly_mask_1ch = np.zeros((roi_h, roi_w), dtype=np.uint8)
            cv2.fillPoly(poly_mask_1ch, roi_vertices, 255)

            roi = cv2.bitwise_and(cv_image, cv_image, mask=poly_mask_1ch)

            # ── 2. 색공간 변환 ──
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            if self.last_valid_target_x is None:
                self.last_valid_target_x = roi_w // 2

            # ── 3. 색상 마스킹 + ROI 적용 ──
            mask_yellow_raw = cv2.inRange(hsv, np.array([20,  65, 120]), np.array([41, 255, 255]))
            mask_yellow     = cv2.bitwise_and(mask_yellow_raw, poly_mask_1ch)

            mask_red_raw = cv2.bitwise_or(
                cv2.inRange(hsv, np.array([0,   30,  49]), np.array([12,  255, 255])),
                cv2.inRange(hsv, np.array([170, 120,  70]), np.array([180, 255, 255]))
            )
            mask_red = cv2.bitwise_and(mask_red_raw, poly_mask_1ch)

            mask_black_raw_unmasked, gray_clahe = self.extract_black_lane(cv_image)
            mask_black_raw = cv2.bitwise_and(mask_black_raw_unmasked, poly_mask_1ch)

            # ── 4. 각도 필터 (그림자 제거) ──
            mask_black = self.filter_by_angle(
                mask_black_raw, min_area=300, max_area=8000,
                min_angle=25.0, max_angle=65.0
            )

            # ── 5. 횡단보도 감지 ──
            contours_black, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid_black_lines  = sum(1 for cnt in contours_black if cv2.contourArea(cnt) > 400)
            is_crosswalk = valid_black_lines >= 3
            if is_crosswalk:
                mask_black = np.zeros_like(mask_black)

            # ── 6. 노란선 위치 파악 ──
            yellow_left  = cv2.countNonZero(mask_yellow[:, :roi_w//2])
            yellow_right = cv2.countNonZero(mask_yellow[:, roi_w//2:])
            yellow_total = yellow_left + yellow_right

            # ── 7. 처음 출발 시 차선 결정 ──
            if self.current_lane == 0 and yellow_total > 500:
                if yellow_left > yellow_right:
                    self.current_lane = 1
                else:
                    self.current_lane = 2
                self.get_logger().info(f"초기 차선 결정: {self.current_lane}차선")

            # ── 8. 차선에 따라 마스크 영역 제한 및 역할 배정 ──
            if self.current_lane == 1:
                # 1차선: 노란선=왼쪽, 검은선=오른쪽
                # 오른쪽 노란선 / 왼쪽 검은선 무시
                mask_yellow_lane        = mask_yellow.copy()
                mask_yellow_lane[:, roi_w//2:] = 0
                mask_black_lane         = mask_black.copy()
                mask_black_lane[:, :roi_w//2]  = 0
                avoid_direction = 1             # 1차선 → 오른쪽으로 회피
                mask_left_lane  = mask_yellow_lane
                mask_right_lane = mask_black_lane

            elif self.current_lane == 2:
                # 2차선: 노란선=오른쪽, 검은선=왼쪽
                # 왼쪽 노란선 / 오른쪽 검은선 무시
                mask_yellow_lane        = mask_yellow.copy()
                mask_yellow_lane[:, :roi_w//2] = 0
                mask_black_lane         = mask_black.copy()
                mask_black_lane[:, roi_w//2:]  = 0
                avoid_direction = -1            # 2차선 → 왼쪽으로 회피
                mask_left_lane  = mask_black_lane
                mask_right_lane = mask_yellow_lane

            else:
                # 차선 미결정 → 노란선 위치로 임시 판단
                mask_yellow_lane = mask_yellow.copy()
                mask_black_lane  = mask_black.copy()
                if yellow_left > yellow_right:
                    avoid_direction = 1
                    mask_black_lane[:, :roi_w//2] = 0
                    mask_left_lane  = mask_yellow_lane
                    mask_right_lane = mask_black_lane
                else:
                    avoid_direction = -1
                    mask_black_lane[:, roi_w//2:] = 0
                    mask_left_lane  = mask_black_lane
                    mask_right_lane = mask_yellow_lane

            # ── 9. 차선 검출 ──
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

            # ── 10. 차선 중앙 계산 ──
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

            # ── 11. vision_status 퍼블리시 ──
            status_msg = String()
            status_msg.data = (
                f"{error}|{1 if red_line_detected else 0}|"
                f"0|"
                f"{1 if yellow_line_detected else 0}|"
                f"{avoid_direction}"
            )
            self.vision_pub.publish(status_msg)

            # ── 12. 디버그 시각화 ──
            overlay = roi.copy()
            cv2.polylines(overlay, [roi_vertices], isClosed=True, color=(255, 0, 0), thickness=2)

            if is_crosswalk:
                cv2.putText(overlay, "CROSSWALK DETECTED!", (roi_w//2 - 120, 140),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)

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

            # 현재 차선 + 회피 방향 표시
            lane_label = f"LANE:{self.current_lane if self.current_lane != 0 else '?'}"
            lane_color = (0, 220, 220) if self.current_lane == 1 else (255, 100, 100)
            cv2.putText(overlay, lane_label, (10, 84),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, lane_color, 2)
            avoid_text  = "AVOID:RIGHT(+1)" if avoid_direction == 1 else "AVOID:LEFT(-1)"
            cv2.putText(overlay, avoid_text, (10, 104),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, lane_color, 1)
            cv2.putText(overlay,
                        f"L:{'YELLOW' if self.current_lane==1 else 'BLACK'}  "
                        f"R:{'BLACK'  if self.current_lane==1 else 'YELLOW'}",
                        (10, 124), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 0), 1)

            def make_mask_vis(m, color, label):
                vis = np.zeros((m.shape[0], m.shape[1], 3), dtype=np.uint8)
                vis[m > 0] = color
                cv2.putText(vis, label, (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                return vis

            cell_w, cell_h = roi_w // 4, int(roi_h * 0.4)
            m_y  = cv2.resize(make_mask_vis(mask_yellow, (0, 220, 220), "YELLOW"),   (cell_w, cell_h))
            m_b  = cv2.resize(make_mask_vis(mask_black,  (0, 255, 0),   "BLACK"),    (cell_w, cell_h))
            m_cl = cv2.resize(cv2.cvtColor(gray_clahe, cv2.COLOR_GRAY2BGR),          (cell_w, cell_h))
            cv2.putText(m_cl, "CLAHE", (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            m_r  = cv2.resize(make_mask_vis(mask_red,    (0, 0, 255),   "RED"),      (roi_w - cell_w*3, cell_h))

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
