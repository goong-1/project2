import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

os.environ['ROS_DOMAIN_ID'] = '30'

class ControlHandler(Node):
    def __init__(self):
        super().__init__('control_handler')
        # 비전 노드 정보 구독 (원본 유지)
        self.vision_sub = self.create_subscription(String, '/vision_status', self.vision_callback, 10)
        
        # Virtual ESP32 토픽 통신 (원본 유지)
        self.status_sub = self.create_subscription(String, '/esp_status', self.status_callback, 10)
        self.publisher = self.create_publisher(String, '/esp_command', 10)

        # 비전 창 업데이트용 상태 퍼블리셔 (원본 유지)
        self.state_pub = self.create_publisher(String, '/control_state', 10)

        # ★ YOLO 정보 구독 및 상태 변수 (좌회전 시퀀스 변수 추가)
        self.yolo_sub = self.create_subscription(String, '/traffic_sign_topic', self.yolo_callback, 10)
        self.yolo_action = "GO"
        self.speed_limit_end_time = 0.0
        self.turn_left_substate = 1
        self.turn_left_timer = 0.0
        self.tl_g_sent = False

        # ★ [원본 유지] 0.2초 주기로 마지막 명령을 지속 전송하는 타이머 (ESP32 타임아웃 방지)
        self.transmit_interval = 0.2
        self.transmit_timer = self.create_timer(self.transmit_interval, self.timer_fallback_transmit)

        # FSM 변수 (원본 유지)
        self.state = "CRUISE"
        self.last_command = ""
        self.stop_start_time = 0.0
        self.red_line_latched = False
        self.obstacle_latched = False
        self.avoid_direction = 0
        self.avoid_substate = 0
        self.waiting_for_esp = False
        
        # 기본 직진 속도 설정
        self.current_speed = 115

    def yolo_callback(self, msg):
        self.yolo_action = msg.data
        if self.yolo_action == "SPEED_LIMIT":
            self.speed_limit_end_time = time.time() + 5.0

    def status_callback(self, msg):
        if msg.data == "DONE":
            self.waiting_for_esp = False
            self.last_command = ""
            self.current_speed = 115  # 작업 완료 후 직진 속도를 다시 기본(115)으로 복구
            self.get_logger().info("ESP32 작업 완료 신호(DONE) 접수.")

    def timer_fallback_transmit(self):
        """ 타이머에 의해 0.2초마다 마지막 명령을 반복 전송하여 하드웨어 정지 방지 """
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
            self.get_logger().info(f"Sent to ESP32: {cmd_str}")
            
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
        try:
            data = msg.data.split('|')
            error = float(data[0])
            red_line_detected = bool(int(data[1]))
            obstacle_in_front = bool(int(data[2]))
            crosswalk_detected = bool(int(data[3]))
            avoid_dir = int(data[4])
            obstacle_detected = bool(int(data[5]))
            yellow_line_detected = bool(int(data[6])) 
        except (ValueError, IndexError):
            return

        if not red_line_detected: self.red_line_latched = False
        if not obstacle_detected: self.obstacle_latched = False

        current_time = time.time()

        if current_time < getattr(self, 'speed_limit_end_time', 0.0):
            self.current_speed = 80
        else:
            self.current_speed = 115

        # FSM 상태 전환
        if self.state == "CRUISE" and not crosswalk_detected:
            if getattr(self, 'yolo_action', 'GO') == "STOP":
                self.state = "YOLO_STOP"
            elif red_line_detected and not self.red_line_latched:
                self.state = "STOP_TIMER"
                self.stop_start_time = current_time
                self.red_line_latched = True
            elif obstacle_in_front and not self.obstacle_latched:
                self.state = "AVOID"
                self.avoid_substate = 1
                self.waiting_for_esp = False
                self.avoid_direction = avoid_dir
            # ★ 좌회전 신호 진입 시퀀스 설정
            elif getattr(self, 'yolo_action', 'GO') == "TURN_LEFT":
                self.state = "YOLO_TURN_LEFT"
                self.turn_left_substate = 1
                self.turn_left_timer = current_time

        # ★ 횡단보도 감지 시 정밀 주행 유지 (회피 및 좌회전 시퀀스 중에는 간섭 금지)
        if crosswalk_detected and self.state not in ["AVOID", "YOLO_TURN_LEFT"]:
            self.last_command = ""
            self.send_command(f"G{self.current_speed}")

        elif self.state == "YOLO_STOP":
            if getattr(self, 'yolo_action', 'GO') == "GO":
                self.state = "CRUISE"
            else:
                self.send_command("S")

        # ★ 하드코딩된 좌회전 시퀀스
        elif self.state == "YOLO_TURN_LEFT":
            if obstacle_in_front and not self.obstacle_latched:
                self.state = "AVOID"
                self.avoid_substate = 1
                self.waiting_for_esp = False
                self.avoid_direction = avoid_dir
            else:
                # [Step 1] 1초 직진
                if self.turn_left_substate == 1:
                    if current_time - self.turn_left_timer <= 1.0:
                        self.send_command(f"G{self.current_speed}")
                    else:
                        self.turn_left_substate = 2

                # [Step 2] 왼쪽으로 45도 회전
                elif self.turn_left_substate == 2:
                    if not self.waiting_for_esp:
                        self.send_command("T45")
                        self.turn_left_substate = 3

                # [Step 3] 회전 완료 대기 후 0.5초 직진
                elif self.turn_left_substate == 3:
                    if self.waiting_for_esp:
                        return
                    
                    if not self.tl_g_sent:
                        self.turn_left_timer = current_time
                        self.tl_g_sent = True
                    
                    if current_time - self.turn_left_timer <= 0.5:
                        self.send_command(f"G{self.current_speed}")
                    else:
                        self.turn_left_substate = 4
                        self.tl_g_sent = False

                # [Step 4] 다시 왼쪽으로 45도 회전
                elif self.turn_left_substate == 4:
                    if not self.waiting_for_esp:
                        self.send_command("T45")
                        self.turn_left_substate = 5

                # [Step 5] 일반 주행 모드로 복귀 (직진)
                elif self.turn_left_substate == 5:
                    if not self.waiting_for_esp:
                        self.state = "CRUISE"

        elif self.state == "STOP_TIMER":
            if current_time - self.stop_start_time > 1.5:
                self.state = "CRUISE"
            else:
                self.send_command("S")

        elif self.state == "AVOID":
            if self.avoid_substate == 1:
                if not self.waiting_for_esp:
                    self.send_command(f"T{45 * self.avoid_direction}")
                    self.avoid_substate = 2      

            elif self.avoid_substate == 2:
                if self.waiting_for_esp:
                    return

                if not getattr(self, 'g_cmd_sent', False):
                    self.send_command(f"G{self.current_speed}")
                    self.g_cmd_sent = True

                if yellow_line_detected:
                    self.avoid_substate = 3
                    self.g_cmd_sent = False 

            elif self.avoid_substate == 3:
                if not self.waiting_for_esp:
                    self.send_command(f"T{-45 * self.avoid_direction}")
                    self.avoid_substate = 4

            elif self.avoid_substate == 4:
                if not self.waiting_for_esp:
                    self.state = "CRUISE"
                    self.avoid_substate = 1

        elif self.state == "CRUISE":
            if abs(error) < 10:
                self.send_command(f"G{self.current_speed}")
            else:
                turn_deg = int(error * -0.1)
                self.send_command(f"T{turn_deg}" if abs(turn_deg) >= 2 else f"G{self.current_speed}")

def main(args=None):
    rclpy.init(args=args)
    node = ControlHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
