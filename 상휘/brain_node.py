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
        
        # 기본 직진 속도 설정 (요청 반영)
        self.current_speed = 115


    def status_callback(self, msg):
        if msg.data == "DONE":
            self.waiting_for_esp = False
            self.last_command = ""
            self.current_speed = 115  # 작업 완료 후 직진 속도를 다시 기본(115)으로 복구
            self.get_logger().info("ESP32 작업 완료 신호(DONE) 접수.")

    # ★ [원본 유지] 하드웨어 멈춤 방지용 백그라운드 송신 함수
    def timer_fallback_transmit(self):
        """ 타이머에 의해 0.2초마다 마지막 명령을 반복 전송하여 하드웨어 정지 방지 """
        if self.last_command:
            msg = String()
            msg.data = self.last_command
            self.publisher.publish(msg)

    def send_command(self, cmd_str):
        # [수정] ESP 피드백(DONE)을 대기 중일 때, '새로운 명령'으로의 변경은 차단하되 
        # 타이머가 공급하는 '기존 마지막 명령'의 반복 전송은 허용하도록 필터링 최적화
        if self.waiting_for_esp and cmd_str != self.last_command:
            return

        if cmd_str != self.last_command:
            msg = String()
            msg.data = cmd_str
            self.publisher.publish(msg)
            self.last_command = cmd_str
            self.get_logger().info(f"Sent to ESP32: {cmd_str}")
            
            # 회전(T)이나 정지(S) 명령이 '처음' 나갔을 때만 대기 상태(Lock) 활성화
            if cmd_str.startswith("T") or cmd_str == "S":
                self.waiting_for_esp = True

        # ★ 0.2초 타이머 백그라운드 송신에 의해 동일한 명령(cmd_str == self.last_command)이 
        # 계속 들어올 때는, 위 조건문을 거치지 않고 이 분기를 통해 하드웨어로 명령이 계속 쏘아집니다.
        elif cmd_str == self.last_command and self.waiting_for_esp:
            msg = String()
            msg.data = cmd_str
            self.publisher.publish(msg)

        # 비전 노드로 상태 송신 (원본 유지)
        state_msg = String()
        display_state = f"{self.state} Step{self.avoid_substate}" if self.state == "AVOID" else self.state
        if self.waiting_for_esp: display_state += " [WAIT]"
        state_msg.data = f"{display_state}|{self.last_command}"
        self.state_pub.publish(state_msg)

    def vision_callback(self, msg):
        # 데이터 언팩 (에러값|정지선|장애물전방|횡단보도|회피방향|장애물자체|노란선여부) 
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

        # 래치 해제 (원본 유지)
        if not red_line_detected: self.red_line_latched = False
        if not obstacle_detected: self.obstacle_latched = False

        current_time = time.time()

        # FSM 상태 전환 (원본 유지)
        if self.state == "CRUISE" and not crosswalk_detected:
            if red_line_detected and not self.red_line_latched:
                self.state = "STOP_TIMER"
                self.stop_start_time = current_time
                self.red_line_latched = True
            elif obstacle_in_front and not self.obstacle_latched:
                self.state = "AVOID"
                self.avoid_substate = 1
                self.waiting_for_esp = False
                self.avoid_direction = avoid_dir

        # FSM 명령 실행 (원본 유지 및 속도 변수 적용)
        if crosswalk_detected and self.state != "AVOID":
            self.last_command = ""
            self.send_command(f"G{self.current_speed}")

        elif self.state == "STOP_TIMER":
            if current_time - self.stop_start_time > 1.5:
                self.state = "CRUISE"
            else:
                self.send_command("S")

        elif self.state == "AVOID":
            # [Step 1] 45도 회전 시작 (원본 유지)
            if self.avoid_substate == 1:
                if not self.waiting_for_esp:
                    self.send_command(f"T{45 * self.avoid_direction}")
                    self.avoid_substate = 2      

            # [Step 2] 회전 완료 대기 후 -> 우회 직진 (원본 유지 및 속도 변수 적용)
            elif self.avoid_substate == 2:
                if self.waiting_for_esp:
                    return

                if not getattr(self, 'g_cmd_sent', False):
                    self.send_command(f"G{self.current_speed}")
                    self.g_cmd_sent = True

                if yellow_line_detected:
                    self.avoid_substate = 3
                    self.g_cmd_sent = False 

            # [Step 3] 다시 원래 차선으로 45도 복귀 회전 (원본 유지)
            elif self.avoid_substate == 3:
                if not self.waiting_for_esp:
                    self.send_command(f"T{-45 * self.avoid_direction}")
                    self.avoid_substate = 4

            # [Step 4] 일반 주행 모드로 복귀 (원본 유지)
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
