import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile_sensor_data)
        
        # 제어 노드로 비전 분석 결과를 송신할 퍼블리셔
        self.vision_pub = self.create_publisher(String, '/vision_status', 10)
        
        # 제어 노드의 현재 상태를 화면에 그려주기 위한 서브스크라이버
        self.state_sub = self.create_subscription(String, '/control_state', self.state_callback, 10)
        
        self.bridge = CvBridge()

        # 원본 변수 유지
        self.lane_width = 350
        self.last_valid_target_x = None
        self.crosswalk_detected = False
        
        # 디버그 표시용 변수
        self.current_control_state = "CRUISE"
        self.last_sent_cmd = ""

    def state_callback(self, msg):
        # 제어 노드로부터 현재 FSM 상태와 마지막 나간 명령어를 받아와 디버그 창에 업데이트
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

        if self.last_valid_target_x is None:
            self.last_valid_target_x = width // 2

        # ── 마스킹 ──
        mask_yellow = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([40, 255, 255]))
        mask_black_raw = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 50]))
        mask_gray = cv2.inRange(hsv, np.array([0, 0, 50]), np.array([180, 50, 180]))
        mask_red = cv2.bitwise_or(
            cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255])),
            cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        )

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

        # ── 장애물 분석 ──
        contours, _ = cv2.findContours(mask_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 1000]
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
                if int(width * 0.3) <= obj_cx <= int(width * 0.7):
                    obstacle_in_front = True

        red_line_detected = cv2.countNonZero(mask_red) > 5000

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
        # 구조: 에러값|정지선감지|장애물전방|횡단보도감지|회피방향|장애물감지자체여부
        status_msg = String()
        status_msg.data = f"{error}|{1 if red_line_detected else 0}|{1 if obstacle_in_front else 0}|{1 if self.crosswalk_detected else 0}|{avoid_direction}|{1 if obstacle_detected else 0}"
        self.vision_pub.publish(status_msg)

        # ══════════════════════════════════════════════
        # 디버그 시각화 (원본 로직 100% 동일 유지)
        # ══════════════════════════════════════════════
        overlay = roi.copy()
        overlay[mask_yellow > 0] = (0, 220, 220)
        overlay[mask_black  > 0] = (0, 255, 0)
        overlay[mask_gray   > 0] = (255, 200, 0)
        overlay[mask_red    > 0] = (0, 0, 255)

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
        m_o = cv2.resize(make_mask_vis(mask_gray,   (255, 200, 0), "OBSTACLE"), (cell_w, cell_h))
        m_r = cv2.resize(make_mask_vis(mask_red,    (0, 0, 255),   "RED"),      (cell_w, cell_h))

        mask_row = cv2.resize(np.hstack([m_y, m_b, m_o, m_r]), (roi_w, cell_h))
        debug_final = np.vstack([overlay, mask_row])
        cv2.imshow("Lane Follower Debug", debug_final)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
