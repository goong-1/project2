#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.image_sub = self.create_subscription(
            Image, 'image_raw', self.image_callback, qos_profile_sensor_data)
        
        # 제어 노드로 비전 분석 결과를 송신할 퍼블리셔
        self.vision_pub = self.create_publisher(String, '/vision_status', 10)
        
        # 제어 노드의 현재 상태를 화면에 그려주기 위한 서브스크라이버
        self.state_sub = self.create_subscription(String, '/control_state', self.state_callback, 10)
        
        self.line_image_pub = self.create_publisher(
            CompressedImage, 
            'image_line/compressed', 
            qos_profile_sensor_data
        )
        
        self.bridge = CvBridge()

        # 원본 변수 유지
        self.lane_width = 350
        self.last_valid_target_x = None
        self.crosswalk_detected = False
        
        # 디버그 표시용 변수
        self.current_control_state = "CRUISE"
        self.last_sent_cmd = ""
        
        # 터미널 시작 알림
        self.get_logger().info('=== VisionNode 가 성공적으로 시작되었습니다 ===')

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

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        height, width, _ = cv_image.shape
        roi = cv_image[int(height/2):height, 0:width].copy()
        roi_h, roi_w = roi.shape[:2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        if self.last_valid_target_x is None:
            self.last_valid_target_x = width // 2

        # ── [색상 마스킹 교정] PC 디버깅 가이드값 100% 반영 ──
        # 1. 노란색 라인 외곽선 범위 최적화
        mask_yellow = cv2.inRange(hsv, np.array([15, 80, 100]), np.array([35, 255, 255]))
        
        # 2. 검은색 가이드선 및 횡단보도 명도 기준 최적화 (바닥면 분리)
        mask_black_raw = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 60]))
        
        # 3. 빨간색 정지선
        mask_red = cv2.bitwise_or(
            cv2.inRange(hsv, np.array([0, 100, 70]), np.array([10, 255, 255])),
            cv2.inRange(hsv, np.array([160, 100, 70]), np.array([180, 255, 255]))
        )

        # 4. 흰색 장애물 탐지 (밝은 회색 바닥 및 조명 반사광 노이즈 완전 차단형 고명도 문턱치 세팅)
        blurred_gray = cv2.GaussianBlur(gray_roi, (5, 5), 0)
        _, mask_white_obstacle = cv2.threshold(blurred_gray, 235, 255, cv2.THRESH_BINARY)

        # ── 횡단보도 제거 ──
        mask_black, removed_rows = self.remove_crosswalk(mask_black_raw)
        self.crosswalk_detected = (removed_rows > roi_h * 0.3)
        combined_mask = cv2.bitwise_or(mask_yellow, mask_black)

        # ── 차선 중심 검출 ──
        m_left  = cv2.moments(combined_mask[:, :width//2])
        m_right = cv2.moments(combined_mask[:, width//2:])
        cx_left  = int(m_left['m10']  / m_left['m00'])   if m_left['m00']  > 0 else -1
        cx_right = int(m_right['m10'] / m_right['m00']) + width//2    if m_right['m00'] > 0 else -1
        if cx_left != -1 and cx_right != -1:
            self.lane_width = cx_right - cx_left

        # ── [장애물 검출 교정] 회색 바닥 분리 및 면적 필터 강화 ──
        contours, _ = cv2.findContours(mask_white_obstacle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 자잘한 조명 잔상 무시를 위해 면적 기준 800으로 상향 조정
        valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 800]
        obstacle_detected = len(valid_contours) > 0
        obstacle_in_front = False
        largest_cnt = None
        
        black_left_val = cv2.countNonZero(mask_black[:, :width//2])
        black_right_val = cv2.countNonZero(mask_black[:, width//2:])
        avoid_direction = 1 if black_left_val > black_right_val else -1

        if obstacle_detected:
            largest_cnt = max(valid_contours, key=cv2.contourArea)
            M = cv2.moments(largest_cnt)
            if M['m00'] > 0:
                obj_cx = int(M['m10'] / M['m00'])
                # 정면 안전 구역 비율 필터링 (가로축 기준 30% ~ 70% 영역 지정)
                if int(width * 0.3) <= obj_cx <= int(width * 0.7):
                    obstacle_in_front = True

        # 정지선 판단 임계 픽셀 보정
        red_line_detected = cv2.countNonZero(mask_red) > 1500

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
                target_x = width // 2
            self.last_valid_target_x = target_x

        error = (width / 2) - target_x

        # ── 제어 노드로 데이터 패킹 전송 ──
        status_msg = String()
        status_msg.data = f"{error}|{1 if red_line_detected else 0}|{1 if obstacle_in_front else 0}|{1 if self.crosswalk_detected else 0}|{avoid_direction}|{1 if obstacle_detected else 0}"
        self.vision_pub.publish(status_msg)

        # ══════════════════════════════════════════════
        # 디버그 시각화 레이어 렌더링 (구조 유지 및 변수 매핑 보정)
        # ══════════════════════════════════════════════
        overlay = roi.copy()
        overlay[mask_yellow > 0] = (0, 220, 220)  # 노란색 차선 시각화
        overlay[mask_black  > 0] = (0, 255, 0)    # 검은색 차선 가이드 시각화
        overlay[mask_white_obstacle > 0] = (255, 200, 0) # 흰색 실물 장애물 감지 영역 시각화
        overlay[mask_red    > 0] = (0, 0, 255)    # 빨간색 정지선 시각화

        crosswalk_removed = cv2.bitwise_and(mask_black_raw, cv2.bitwise_not(mask_black))
        overlay[crosswalk_removed > 0] = (180, 80, 255)

        if largest_cnt is not None:
            x, y, w, h = cv2.boundingRect(largest_cnt)
            color = (0, 0, 255) if obstacle_in_front else (0, 200, 255)
            cv2.rectangle(overlay, (x, y), (x+w, y+h), color, 2)
            cv2.putText(overlay, "OBS" + (" FRONT" if obstacle_in_front else ""),
                        (x, max(y-5, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        if cx_left != -1:
            cv2.circle(overlay, (cx_left, roi_h//2), 8, (255, 0, 255), -1)
        if cx_right != -1:
            cv2.circle(overlay, (cx_right, roi_h//2), 8, (255, 0, 255), -1)

        cv2.circle(overlay, (target_x, roi_h//2), 12, (0, 0, 255), -1)
        cv2.line(overlay, (roi_w//2, 0), (roi_w//2, roi_h), (180, 180, 180), 1)

        state_color = {"CRUISE": (255,255,255), "AVOID": (0,165,255), "STOP_TIMER": (0,0,255)}
        status_text = f"State:{self.current_control_state}"
        if self.crosswalk_detected:
            status_text += " [CROSSWALK]"
        
        cv2.putText(overlay, status_text, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, state_color.get(self.current_control_state, (255,255,255)), 2)
        cv2.putText(overlay, f"Cmd:{self.last_sent_cmd}", (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(overlay, f"RED:{'ON' if red_line_detected else 'off'}", (10, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255) if red_line_detected else (100,100,100), 1)

        def make_mask_vis(mask, color, label):
            vis = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
            vis[mask > 0] = color
            cv2.putText(vis, label, (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            return vis

        cell_w, cell_h = roi_w // 4, roi_h // 2
        m_y = cv2.resize(make_mask_vis(mask_yellow, (0, 220, 220), "YELLOW"),   (cell_w, cell_h))
        m_b = cv2.resize(make_mask_vis(mask_black,  (0, 255, 0),   "BLACK"),    (cell_w, cell_h))
        m_o = cv2.resize(make_mask_vis(mask_white_obstacle, (255, 200, 0), "WHITE_OBS"), (cell_w, cell_h))
        m_r = cv2.resize(make_mask_vis(mask_red,    (0, 0, 255),   "RED"),      (cell_w, cell_h))

        mask_row = cv2.resize(np.hstack([m_y, m_b, m_o, m_r]), (roi_w, cell_h))
        debug_final = np.vstack([overlay, mask_row])

        # 압축 이미지 변환 및 토픽 발행
        try:
            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(debug_final, dst_format='jpeg')
            self.line_image_pub.publish(compressed_msg)
            img_sent = True
        except Exception as e:
            self.get_logger().error(f'압축 이미지 변환 및 송신 실패: {e}')
            img_sent = False

        # 처리 진행 상황 터미널 화면에 주기적으로 출력 (0.5초 주기)
        obs_status = "정면 감지" if obstacle_in_front else ("주변 존재" if obstacle_detected else "없음")
        self.get_logger().info(
            f"[진행 내용] 오차(Error): {error:6.1f} | "
            f"횡단보도: {'[감지]' if self.crosswalk_detected else '미감지'} | "
            f"정지선: {'[적색]' if red_line_detected else '미감지'} | "
            f"장애물: {obs_status:<4} | "
            f"압축영상 전송: {'OK' if img_sent else 'FAIL'}",
            throttle_duration_sec=0.5
        )

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()