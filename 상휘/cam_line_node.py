#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
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

        self.vision_pub = self.create_publisher(String, '/vision_status', 10)
        self.state_sub  = self.create_subscription(String, '/control_state', self.state_callback, 10)

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.image_pub = self.create_publisher(CompressedImage, '/image_line/compressed', best_effort_qos)

        self.bridge    = CvBridge()
        self.lane_width = 450
        self.last_valid_target_x   = None
        self.crosswalk_detected    = False
        self.current_control_state = "CRUISE"
        self.last_sent_cmd         = ""

    def state_callback(self, msg):
        try:
            self.current_control_state, self.last_sent_cmd = msg.data.split('|')
        except ValueError:
            pass

    def remove_crosswalk(self, mask_black):
        result = mask_black.copy()
        removed_rows = 0
        for y in range(mask_black.shape[0]):
            row = mask_black[y, :]
            transitions = np.diff(row.astype(int))
            runs = int(np.sum(transitions > 0))
            if runs >= 3:
                result[y, :] = 0
                removed_rows += 1
        return result, removed_rows

    def filter_by_angle(self, mask, min_area=300, max_area=8000,
                         min_angle=25.0, max_angle=65.0):
        """
        PCA 주축 기울기 + 면적 범위 기반 그림자 필터
        차선: 소실점 방향 대각선 → 25~65도 통과
        그림자: 수직(|) → 70~90도 제거
        """
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

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            roi      = cv_image.copy()
            roi_h, roi_w = roi.shape[:2]

            roi_vertices = np.array([[
                (0,   400),
                (240, 200),
                (400, 200),
                (640, 400)
            ]], dtype=np.int32)

            poly_mask = np.zeros_like(roi)
            cv2.fillPoly(poly_mask, roi_vertices, (255, 255, 255))
            roi = cv2.bitwise_and(roi, poly_mask)

            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            if self.last_valid_target_x is None:
                self.last_valid_target_x = roi_w // 2

            # ── 마스킹 ──
            mask_yellow    = cv2.inRange(hsv, np.array([20,  65, 120]), np.array([41,  255, 255]))
            mask_black_raw = cv2.inRange(hsv, np.array([47,   0,  96]), np.array([180, 255, 132]))
            mask_red = cv2.bitwise_or(
                cv2.inRange(hsv, np.array([0,   30,  49]), np.array([12,  255, 255])),
                cv2.inRange(hsv, np.array([170, 120,  70]), np.array([180, 255, 255]))
            )

            # ── 기울기 + 면적 필터로 그림자 제거 ──
            mask_black_filtered = self.filter_by_angle(
                mask_black_raw, min_area=300, max_area=8000,
                min_angle=25.0, max_angle=65.0
            )

            # ── 횡단보도 제거 ──
            mask_black, removed_rows = self.remove_crosswalk(mask_black_filtered)
            self.crosswalk_detected  = (removed_rows > roi_h * 0.3)
            combined_mask = cv2.bitwise_or(mask_yellow, mask_black)

            # ── 차선 중심 검출 ──
            m_left  = cv2.moments(combined_mask[:, :roi_w//2])
            m_right = cv2.moments(combined_mask[:, roi_w//2:])
            cx_left  = int(m_left['m10']  / m_left['m00'])             if m_left['m00']  > 0 else -1
            cx_right = int(m_right['m10'] / m_right['m00']) + roi_w//2 if m_right['m00'] > 0 else -1
            if cx_left != -1 and cx_right != -1:
                self.lane_width = cx_right - cx_left

            red_line_detected    = cv2.countNonZero(mask_red)    > 5000
            yellow_line_detected = cv2.countNonZero(mask_yellow) > 5000

            # ── 조향 목표 계산 ──
            if self.crosswalk_detected:
                target_x = self.last_valid_target_x
            else:
                if cx_left != -1 and cx_right != -1:
                    target_x = (cx_left + cx_right) // 2
                elif cx_left != -1:
                    target_x = cx_left + (self.lane_width // 2)
                elif cx_right != -1:
                    target_x = cx_right - (self.lane_width // 2)
                else:
                    target_x = roi_w // 2
                self.last_valid_target_x = target_x

            error = (roi_w / 2) - target_x

            # ── 제어 노드로 전송 ──
            # 포맷: error | red_line | crosswalk | yellow_line
            # 장애물 관련 필드 완전 제거
            status_msg = String()
            status_msg.data = (
                f"{error}|{1 if red_line_detected else 0}|"
                f"{1 if self.crosswalk_detected else 0}|"
                f"{1 if yellow_line_detected else 0}"
            )
            self.vision_pub.publish(status_msg)

            # ── 디버그 시각화 ──
            overlay = roi.copy()
            cv2.polylines(overlay, [roi_vertices], isClosed=True, color=(255, 0, 0), thickness=2)

            overlay[mask_yellow > 0] = (0, 220, 220)   # 노랑 차선
            overlay[mask_black  > 0] = (0, 255, 0)     # 인정된 검은 차선
            overlay[mask_red    > 0] = (0, 0, 255)     # 적색 라인

            # 그림자로 제거된 영역 → 주황
            shadow_removed = cv2.bitwise_and(
                mask_black_raw, cv2.bitwise_not(mask_black_filtered))
            overlay[shadow_removed > 0] = (0, 128, 255)

            # 횡단보도로 제거된 영역 → 보라
            crosswalk_removed = cv2.bitwise_and(
                mask_black_filtered, cv2.bitwise_not(mask_black))
            overlay[crosswalk_removed > 0] = (180, 80, 255)

            draw_y = int(roi_h * 0.8)
            if cx_left  != -1: cv2.circle(overlay, (cx_left,  draw_y), 8,  (255, 0, 255), -1)
            if cx_right != -1: cv2.circle(overlay, (cx_right, draw_y), 8,  (255, 0, 255), -1)
            cv2.circle(overlay, (target_x, draw_y), 12, (0, 0, 255), -1)
            cv2.line(overlay, (roi_w//2, 0), (roi_w//2, roi_h), (180, 180, 180), 1)

            state_color = {"CRUISE": (255,255,255), "AVOID": (0,165,255), "STOP_TIMER": (0,0,255)}
            status_text = f"State:{self.current_control_state}"
            if self.crosswalk_detected:
                status_text += " [CROSSWALK]"

            cv2.putText(overlay, status_text, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        state_color.get(self.current_control_state, (255,255,255)), 2)
            cv2.putText(overlay, f"Cmd:{self.last_sent_cmd}", (10, 44),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.putText(overlay, f"RED:{'ON' if red_line_detected else 'off'}", (10, 64),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 255) if red_line_detected else (100, 100, 100), 1)

            def make_mask_vis(m, color, label):
                vis = np.zeros((m.shape[0], m.shape[1], 3), dtype=np.uint8)
                vis[m > 0] = color
                cv2.putText(vis, label, (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                return vis

            cell_w, cell_h = roi_w // 4, int(roi_h * 0.4)
            m_y = cv2.resize(make_mask_vis(mask_yellow,    (0, 220, 220), "YELLOW"), (cell_w, cell_h))
            m_b = cv2.resize(make_mask_vis(mask_black,     (0, 255, 0),   "BLACK"),  (cell_w, cell_h))
            m_s = cv2.resize(make_mask_vis(shadow_removed, (0, 128, 255), "SHADOW"), (cell_w, cell_h))
            m_r = cv2.resize(make_mask_vis(mask_red,       (0, 0, 255),   "RED"),    (roi_w - cell_w*3, cell_h))

            mask_row    = cv2.resize(np.hstack([m_y, m_b, m_s, m_r]), (roi_w, cell_h))
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
