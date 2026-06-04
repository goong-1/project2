import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import time

os.environ['ROS_DOMAIN_ID'] = '30'

class ControlHandler(Node):
    def __init__(self):
        super().__init__('control_handler')
        self.vision_sub = self.create_subscription(String, '/vision_status', self.vision_callback, 10)
        self.status_sub = self.create_subscription(String, '/esp_status', self.status_callback, 10)
        self.publisher = self.create_publisher(String, '/esp_command', 10)
        self.state_pub = self.create_publisher(String, '/control_state', 10)
        self.yolo_sub = self.create_subscription(String, '/traffic_sign_topic', self.yolo_callback, 10)
        
        self.lidar_sub = self.create_subscription(Float32MultiArray, '/obstacle_status', self.lidar_callback, 10)
        self.lidar_obstacle_detected = False
        self.lidar_obstacle_distance = 0.0

        self.yolo_action = "GO"
        self.speed_limit_end_time = 0.0
        self.turn_left_substate = 1
        self.turn_left_timer = 0.0
        self.tl_g_sent = False

        self.transmit_interval = 0.2
        self.transmit_timer = self.create_timer(self.transmit_interval, self.timer_fallback_transmit)

        self.state = "CRUISE"
        self.last_command = ""
        self.stop_start_time = 0.0
        self.red_line_latched = False
        self.avoid_direction = 0
        self.avoid_substate = 0
        self.waiting_for_esp = False
        self.current_speed = 115

        self.last_vision_time = time.time()
        self.last_yolo_time = time.time()
        self.last_change_log_time = 0.0
        self.last_logged_state = ""
        self.last_logged_cmd = ""
        
        self.vision_warned = False
        self.yolo_warned = False
        self.both_warned = False
        
        # ★ 장애물 3초 대기용 타이머 변수
        self.obstacle_wait_start_time = 0.0
        
        self.monitor_timer = self.create_timer(2.0, self.monitor_status)

    def lidar_callback(self, msg):
        if len(msg.data) >= 2:
            status = msg.data[0]
            self.lidar_obstacle_distance = msg.data[1]
            if status == 1.0:
                self.lidar_obstacle_detected = True
            else:
                self.lidar_obstacle_detected = False

    def print_log(self, is_change=False, reason=""):
        display_state = f"{self.state} Step{self.avoid_substate}" if self.state == "AVOID" else self.state
        if self.state == "YOLO_TURN_LEFT": display_state = f"{self.state} Step{self.turn_left_substate}"
        if self.waiting_for_esp: display_state += " [WAIT]"

        log_str = f"상태: [{display_state: <15}] | 명령: [{self.last_command: <4}] | 속도: [{self.current_speed: <3}]"
        if reason:
            log_str += f" ◀ 판단: {reason}"

        if is_change:
            self.get_logger().info(f"🔄 [명령 변경] {log_str}")
            self.last_change_log_time = time.time()
            self.last_logged_state = self.state
            self.last_logged_cmd = self.last_command
        else:
            self.get_logger().info(f"▶️ [명령 유지] {log_str}")

    def monitor_status(self):
        current_time = time.time()
        vision_delayed = (current_time - self.last_vision_time) > 2.0
        yolo_delayed = (current_time - self.last_yolo_time) > 2.0

        if vision_delayed and yolo_delayed:
            if not self.both_warned:
                self.get_logger().warning("⚠️ [신호 경고] 차선 감지(Line)와 YOLO 신호가 모두 없습니다. (기본 대기 상태)")
                self.both_warned = True
                self.vision_warned = False
                self.yolo_warned = False
        elif vision_delayed and not yolo_delayed:
            if not self.vision_warned:
                self.get_logger().warning("⚠️ [신호 경고] YOLO 신호는 들어오지만 차선 감지(Line) 신호가 없습니다. (주행 판단 불가/대기 중)")
                self.vision_warned = True
                self.both_warned = False
        elif not vision_delayed and yolo_delayed:
            if not self.yolo_warned:
                self.get_logger().warning("⚠️ [신호 경고] 차선 감지(Line) 신호는 정상이지만 YOLO 신호가 없습니다. (표지판 무시 모드)")
                self.yolo_warned = True
                self.both_warned = False
        else:
            if self.vision_warned or self.yolo_warned or self.both_warned:
                self.get_logger().info("✅ [신호 복구] 라인과 YOLO 신호가 모두 정상적으로 들어오고 있습니다.")
                self.vision_warned = False
                self.yolo_warned = False
                self.both_warned = False
            
            if current_time - self.last_change_log_time >= 2.0:
                self.print_log(is_change=False, reason="입력 대기 (유지 중)")

    def yolo_callback(self, msg):
        self.last_yolo_time = time.time()
        self.yolo_action = msg.data
        if self.yolo_action == "SPEED_LIMIT":
            self.speed_limit_end_time = time.time() + 5.0

    def status_callback(self, msg):
        if msg.data == "DONE":
            self.waiting_for_esp = False
            self.last_command = ""
            self.current_speed = 115 

    def timer_fallback_transmit(self):
        if self.last_command:
            msg = String()
            msg.data = self.last_command
            self.publisher.publish(msg)

    def send_command(self, cmd_str):
        if self.waiting_for_esp and cmd_str != self.last_command:
            return

        if cmd_str != self.last_command:
            msg = String()
            msg.data = cmd_str
            self.publisher.publish(msg)
            self.last_command = cmd_str
            
            if cmd_str.startswith("T") or cmd_str == "S":
                self.waiting_for_esp = True

        elif cmd_str == self.last_command and self.waiting_for_esp:
            msg = String()
            msg.data = cmd_str
            self.publisher.publish(msg)

        state_msg = String()
        display_state = f"{self.state} Step{self.avoid_substate}" if self.state == "AVOID" else self.state
        if self.state == "YOLO_TURN_LEFT":
            display_state = f"{self.state} Step{self.turn_left_substate}"
        if self.waiting_for_esp: display_state += " [WAIT]"
        state_msg.data = f"{display_state}|{self.last_command}"
        self.state_pub.publish(state_msg)

    def vision_callback(self, msg):
        self.last_vision_time = time.time()
        current_reason = "" 

        try:
            data = msg.data.split('|')
                
            error = float(data[0])
            red_line_detected = bool(int(data[1]))
            crosswalk_detected = bool(int(data[2]))
            yellow_line_detected = bool(int(data[3])) 
            
            # ★ 변경된 line 포맷에서 동적으로 회피 방향 수용
            avoid_dir = int(data[4]) 
            
            obstacle_in_front = self.lidar_obstacle_detected
            
        except (ValueError, IndexError):
            return

        if not red_line_detected: self.red_line_latched = False

        current_time = time.time()

        if current_time < getattr(self, 'speed_limit_end_time', 0.0):
            if self.current_speed != 80:
                current_reason = "YOLO 감속(SPEED_LIMIT) 신호 확인 -> 5초간 감속"
                self.current_speed = 80
        else:
            if getattr(self, 'current_speed', 115) == 80:
                current_reason = "감속 구간 5초 종료 -> 일반 속도 복구"
            self.current_speed = 115

        # ★ CRUISE(기본 주행)일 때만 라이다 장애물 판별 적용
        if self.state == "CRUISE" and not crosswalk_detected:
            if getattr(self, 'yolo_action', 'GO') == "STOP":
                current_reason = "YOLO 정지(STOP) 신호 감지 -> 정지"
                self.state = "YOLO_STOP"
            elif red_line_detected and not self.red_line_latched:
                current_reason = "바닥 정지선(Red Line) 감지 -> 1.5초 정지 타이머"
                self.state = "STOP_TIMER"
                self.stop_start_time = current_time
                self.red_line_latched = True
            elif obstacle_in_front:
                current_reason = f"라이다 전방 장애물 감지({self.lidar_obstacle_distance:.2f}m) -> 정지 후 3초 판별 대기"
                self.state = "OBSTACLE_WAIT"
                self.obstacle_wait_start_time = current_time
                self.avoid_direction = avoid_dir
            elif getattr(self, 'yolo_action', 'GO') == "TURN_LEFT":
                current_reason = "YOLO 좌회전 신호 감지 -> 시퀀스 진입"
                self.state = "YOLO_TURN_LEFT"
                self.turn_left_substate = 1
                self.turn_left_timer = current_time

        # 횡단보도 최우선 처리
        if crosswalk_detected and self.state not in ["AVOID", "YOLO_TURN_LEFT", "OBSTACLE_WAIT"]:
            self.last_command = ""
            self.send_command(f"G{self.current_speed}")
            if getattr(self, 'last_logged_cmd', "") != self.last_command:
                current_reason = "횡단보도 위 -> 조향 잠금 및 직진 통과"

        # ★ 동적/정적 장애물 3초 판별 상태 추가
        elif self.state == "OBSTACLE_WAIT":
            if not self.lidar_obstacle_detected:
                current_reason = "장애물 사라짐(동적 장애물 판단) -> 직진 주행 재개"
                self.state = "CRUISE"
            elif current_time - self.obstacle_wait_start_time >= 3.0:
                current_reason = f"3초 경과(정적 장애물 확정) -> 차선 회피({self.avoid_direction}) 기동 진입"
                self.state = "AVOID"
                self.avoid_substate = 1
                self.waiting_for_esp = False
            else:
                self.send_command("S")

        elif self.state == "YOLO_STOP":
            if getattr(self, 'yolo_action', 'GO') == "GO":
                current_reason = "YOLO 초록불(GO) 감지 -> 주행 재개"
                self.state = "CRUISE"
            else:
                self.send_command("S")

        # ★ 좌회전 중 장애물 회피 판별 삭제 (라이다 무시)
        elif self.state == "YOLO_TURN_LEFT":
            if self.turn_left_substate == 1:
                if current_time - self.turn_left_timer <= 1.0:
                    self.send_command(f"G{self.current_speed}")
                else:
                    current_reason = "좌회전 1초 직진 완료 -> 45도 회전 시작"
                    self.turn_left_substate = 2

            elif self.turn_left_substate == 2:
                if not self.waiting_for_esp:
                    self.send_command("T45")
                    current_reason = "좌회전 1차 45도 회전 대기"
                    self.turn_left_substate = 3

            elif self.turn_left_substate == 3:
                if self.waiting_for_esp:
                    pass
                else:
                    if not self.tl_g_sent:
                        self.turn_left_timer = current_time
                        self.tl_g_sent = True
                        current_reason = "좌회전 1차 회전 완료 -> 0.5초 직진"
                    
                    if current_time - self.turn_left_timer <= 0.5:
                        self.send_command(f"G{self.current_speed}")
                    else:
                        current_reason = "좌회전 0.5초 직진 완료 -> 복귀 회전 시작"
                        self.turn_left_substate = 4
                        self.tl_g_sent = False

            elif self.turn_left_substate == 4:
                if not self.waiting_for_esp:
                    self.send_command("T45")
                    current_reason = "좌회전 2차 45도 회전 대기"
                    self.turn_left_substate = 5

            elif self.turn_left_substate == 5:
                if not self.waiting_for_esp:
                    current_reason = "좌회전 시퀀스 모두 완료 -> 기본 주행 복귀"
                    self.state = "CRUISE"

        elif self.state == "STOP_TIMER":
            if current_time - self.stop_start_time > 1.5:
                current_reason = "1.5초 정지 완료 -> 주행 재개"
                self.state = "CRUISE"
            else:
                self.send_command("S")

        elif self.state == "AVOID":
            if self.avoid_substate == 1:
                if not self.waiting_for_esp:
                    self.send_command(f"T{45 * self.avoid_direction}")
                    current_reason = f"회피 1단계: 라이다 정적 감지 {45 * self.avoid_direction}도 회전"
                    self.avoid_substate = 2      

            elif self.avoid_substate == 2:
                if self.waiting_for_esp:
                    pass
                else:
                    if not getattr(self, 'g_cmd_sent', False):
                        self.send_command(f"G{self.current_speed}")
                        current_reason = "회전 완료 -> 우회 직진 중"
                        self.g_cmd_sent = True

                    if yellow_line_detected:
                        current_reason = "노란선 감지 -> 차선 복귀 회전 준비"
                        self.avoid_substate = 3
                        self.g_cmd_sent = False 

            elif self.avoid_substate == 3:
                if not self.waiting_for_esp:
                    self.send_command(f"T{-45 * self.avoid_direction}")
                    current_reason = f"회피 3단계: {-45 * self.avoid_direction}도 복귀 회전"
                    self.avoid_substate = 4

            elif self.avoid_substate == 4:
                if not self.waiting_for_esp:
                    current_reason = "회피 기동 종료 -> 기본 주행 복귀"
                    self.state = "CRUISE"
                    self.avoid_substate = 1

        elif self.state == "CRUISE":
            if abs(error) < 10:
                self.send_command(f"G{self.current_speed}")
            else:
                turn_deg = int(error * -0.1)
                if abs(turn_deg) >= 2:
                    self.send_command(f"T{turn_deg}")
                else:
                    self.send_command(f"G{self.current_speed}")

        state_changed = (self.state != self.last_logged_state)
        cmd_changed = (self.last_command != getattr(self, 'last_logged_cmd', ""))
        
        if state_changed or cmd_changed or current_reason:
            if current_time - self.last_change_log_time >= 0.5:
                if not current_reason:
                    if cmd_changed and self.state == "CRUISE": current_reason = "차선 오차 보정 조향"
                    else: current_reason = "진행 상태 변경"
                
                self.print_log(is_change=True, reason=current_reason)

def main(args=None):
    rclpy.init(args=args)
    node = ControlHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()