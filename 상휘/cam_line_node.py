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

# 통신 설정
os.environ['ROS_DOMAIN_ID'] = '30'

TARGET_IP_1 = "192.168.0.125"
TARGET_IP_2 = "192.168.0.110"
TARGET_IP_3 = "192.168.0.155"


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        # Subscriber
        self.image_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, qos_profile_sensor_data)
        self.state_sub = self.create_subscription(String, '/control_state', self.state_callback, 10)

        # Publisher
        self.vision_pub = self.create_publisher(String, '/vision_status', 10)
        
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.image_pub = self.create_publisher(CompressedImage, '/image_line/compressed', best_effort_qos)

        self.bridge = CvBridge()
        
        # 주행 상태 및 차선 관련 변수
        self.lane_width = 450
        self.last_valid_target_x = None
        self.crosswalk_detected = False
        self.current_control_state = "CRUISE"
        self.last_sent_cmd = ""

        # 차선 피팅 데이터 유지를 위한 이전 값 (흔들림 방지)
        self.prev_left_top = None
        self.prev_left_bot = None
        self.prev_right_top = None
        self.prev_right_bot = None

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
        """면적과 각도를 기반으로 그림자나 불필요한 노이즈를 제거"""
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

    def fit_line(self, points, roi_h):
        """점들을 polyfit으로 직선 피팅하여 화면 상단/하단 X좌표 반환"""
        if len(points) < 2:
            return -1, -1
        pts = np.array(points)
        try:
            a, b = np.polyfit(pts[:, 1], pts[:, 0], 1)
            x_top = int(a * 0     + b)
            x_bot = int(a * roi_h + b)
            return x_top, x_bot
        except Exception:
            return -1, -1

    def detect_lanes(self, mask_black, mask_yellow, roi_w, roi_h):
        """Hough 직선 검출 후 기울기로 좌/우를 나누어 Polyfit 피팅 수행"""
        combined = cv2.bitwise_or(mask_black, mask_yellow)
        edges = cv2.Canny(combined, 50, 150)

        lines = cv2.HoughLinesP(
            edges, rho=1, theta=np.pi / 180, threshold=30, minLineLength=30, maxLineGap=20
        )

        left_points, right_points = [], []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 == x1: continue
                slope = (y2 - y1) / (x2 - x1)
                
                if abs(slope) < 0.3: continue # 수평선에 가까운 노이즈 무시

                # 기울기에 따라 좌/우 차선 분류
                if slope < 0:
                    if max(x1, x2) < roi_w * 0.75:
                        left_points += [(x1, y1), (x2, y2)]
                else:
                    if min(x1, x2) > roi_w * 0.25:
                        right_points += [(x1, y1), (x2, y2)]

        left_top, left_bot = self.fit_line(left_points, roi_h)
        right_top, right_bot = self.fit_line(right_points, roi_h)

        return left_top, left_bot, right_top, right_bot, lines

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            roi = cv_image.copy()
            roi_h, roi_w = roi.shape[:2]

            # ── 1. 사다리꼴 관심 영역(ROI) ──
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

            # ── 2. 색상 기반 마스킹 ──
            mask_yellow    = cv2.inRange(hsv, np.array([20,  65, 120]), np.array([41,  255, 255]))
            mask_black_raw = cv2.inRange(hsv, np.array([45,   0,  9]), np.array([180, 255, 132]))
            mask_red = cv2.bitwise_or(
                cv2.inRange(hsv, np.array([0,   30,  49]), np.array([12,  255, 255])),
                cv2.inRange(hsv, np.array([170, 120,  70]), np.array([180, 255, 255]))
            )

            # ── 3. 그림자 및 횡단보도 필터링 ──
            mask_black_filtered = self.filter_by_angle(
                mask_black_raw, min_area=300, max_area=8000,
                min_angle=25.0, max_angle=65.0
            )
            mask_black, removed_rows = self.remove_crosswalk(mask_black_filtered)
            self.crosswalk_detected  = (removed_rows > roi_h * 0.3)

            # ── [수정] 노란선(오른쪽 차선) 위치 기반 그림자 오인식 차단 ──
            yellow_left  = cv2.countNonZero(mask_yellow[:, :roi_w//2])
            yellow_right = cv2.countNonZero(mask_yellow[:, roi_w//2:])
            avoid_direction = 1 if yellow_left > yellow_right else -1

            # 노란색 차선이 주로 감지되는 영역의 검은색 마스크를 완전히 지움
            if yellow_right > yellow_left:
                # 노란선이 화면 오른쪽에 주로 보임 -> 화면 오른쪽의 검은색(그림자) 무시
                mask_black[:, roi_w//2:] = 0
            elif yellow_left > yellow_right:
                # 노란선이 화면 왼쪽에 주로 보임 -> 화면 왼쪽의 검은색(그림자) 무시
                mask_black[:, :roi_w//2] = 0

            # ── 4. 차선 피팅 (Hough + Polyfit) ──
            left_top, left_bot, right_top, right_bot, raw_lines = self.detect_lanes(
                mask_black, mask_yellow, roi_w, roi_h)

            # 감지 실패 시 이전 상태 유지
            if left_bot != -1:
                self.prev_left_top, self.prev_left_bot = left_top, left_bot
            elif self.prev_left_bot is not None:
                left_top, left_bot = self.prev_left_top, self.prev_left_bot

            if right_bot != -1:
                self.prev_right_top, self.prev_right_bot = right_top, right_bot
            elif self.prev_right_bot is not None:
                right_top, right_bot = self.prev_right_top, self.prev_right_bot

            # 차선 폭(lane_width) 갱신
            if left_bot != -1 and right_bot != -1:
                measured = right_bot - left_bot
                if 100 < measured < roi_w:
                    self.lane_width = measured

            # 한쪽만 보일 때 반대쪽 차선 추정
            if left_bot != -1 and right_bot == -1:
                right_bot = left_bot + self.lane_width
                right_top = left_top + self.lane_width
            elif right_bot != -1 and left_bot == -1:
                left_bot  = right_bot - self.lane_width
                left_top  = right_top - self.lane_width

            # ── 5. 목표 주행점(Target X) 계산 ──
            if left_bot != -1 and right_bot != -1:
                target_x = (left_bot + right_bot) // 2
                target_x_top = (left_top + right_top) // 2
            else:
                target_x = self.last_valid_target_x
                target_x_top = self.last_valid_target_x

            # 횡단보도가 아닐 때만 유효한 값으로 저장
            if not self.crosswalk_detected:
                self.last_valid_target_x = target_x

            # 오차 계산 (이전과 동일하게 유지)
            error = (roi_w / 2) - target_x

            # ── 6. 상태 판별 (회피 및 붉은 선) ──
            red_line_detected    = cv2.countNonZero(mask_red) > 5000
            yellow_line_detected = cv2.countNonZero(mask_yellow) > 5000

            yellow_left  = cv2.countNonZero(mask_yellow[:, :roi_w//2])
            yellow_right = cv2.countNonZero(mask_yellow[:, roi_w//2:])
            avoid_direction = 1 if yellow_left > yellow_right else -1

            # ── 7. 제어 명령 퍼블리시 ──
            status_msg = String()
            status_msg.data = (
                f"{error}|{1 if red_line_detected else 0}|"
                f"{1 if self.crosswalk_detected else 0}|"
                f"{1 if yellow_line_detected else 0}|"
                f"{avoid_direction}"
            )
            self.vision_pub.publish(status_msg)

            # ── 8. 디버그 시각화 ──
            overlay = roi.copy()
            cv2.polylines(overlay, [roi_vertices], isClosed=True, color=(255, 0, 0), thickness=2)

            overlay[mask_yellow > 0] = (0, 220, 220)
            overlay[mask_black  > 0] = (0, 255, 0)
            overlay[mask_red    > 0] = (0, 0, 255)

            # 왼쪽 선(파랑), 오른쪽 선(주황), 중앙 선(초록) 그리기
            if left_top != -1 and left_bot != -1:
                cv2.line(overlay, (left_top, 0), (left_bot, roi_h), (0, 80, 255), 2)
            if right_top != -1 and right_bot != -1:
                cv2.line(overlay, (right_top, 0), (right_bot, roi_h), (255, 80, 0), 2)
            
            cv2.line(overlay, (target_x_top, 0), (target_x, roi_h), (0, 255, 0), 2)
            
            # 목표점(타겟) 및 화면 중심 표시
            draw_y = int(roi_h * 0.8)
            cv2.circle(overlay, (target_x, draw_y), 12, (0, 0, 255), -1)
            cv2.line(overlay, (roi_w//2, 0), (roi_w//2, roi_h), (180, 180, 180), 1)

            # 텍스트 정보 표시
            state_color = {"CRUISE": (255,255,255), "AVOID": (0,165,255), "STOP_TIMER": (0,0,255)}
            status_text = f"State:{self.current_control_state}"
            if self.crosswalk_detected: status_text += " [CROSSWALK]"

            cv2.putText(overlay, status_text, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, state_color.get(self.current_control_state, (255,255,255)), 2)
            cv2.putText(overlay, f"Cmd:{self.last_sent_cmd}", (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.putText(overlay, f"RED:{'ON' if red_line_detected else 'off'}", (10, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255) if red_line_detected else (100, 100, 100), 1)
            
            avoid_text  = "AVOID: RIGHT(+1)" if avoid_direction == 1 else "AVOID: LEFT(-1)"
            cv2.putText(overlay, avoid_text, (10, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 220) if avoid_direction == 1 else (255, 100, 100), 1)

            # 하단 마스크 이미지 생성
            def make_mask_vis(m, color, label):
                vis = np.zeros((m.shape[0], m.shape[1], 3), dtype=np.uint8)
                vis[m > 0] = color
                cv2.putText(vis, label, (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                return vis

            cell_w, cell_h = roi_w // 4, int(roi_h * 0.4)
            shadow_removed = cv2.bitwise_and(mask_black_raw, cv2.bitwise_not(mask_black_filtered))
            
            m_y = cv2.resize(make_mask_vis(mask_yellow,    (0, 220, 220), "YELLOW"), (cell_w, cell_h))
            m_b = cv2.resize(make_mask_vis(mask_black,     (0, 255, 0),   "BLACK"),  (cell_w, cell_h))
            m_s = cv2.resize(make_mask_vis(shadow_removed, (0, 128, 255), "SHADOW"), (cell_w, cell_h))
            m_r = cv2.resize(make_mask_vis(mask_red,       (0, 0, 255),   "RED"),    (roi_w - cell_w*3, cell_h))

            mask_row = cv2.resize(np.hstack([m_y, m_b, m_s, m_r]), (roi_w, cell_h))
            debug_final = np.vstack([overlay, mask_row])

            # Compressed 이미지 퍼블리시
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.header.frame_id = "camera_frame"
            compressed_msg.format = "jpeg"
            compressed_msg.data = cv2.imencode('.jpg', debug_final)[1].tobytes()
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
