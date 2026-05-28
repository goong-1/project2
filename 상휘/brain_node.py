import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class ControlHandler(Node):
    def __init__(self):
        super().__init__('control_handler')
        # 비전 노드 정보 구독
        self.vision_sub = self.create_subscription(String, '/vision_status', self.vision_callback, 10)
        # Virtual ESP32 토픽 통신
        self.status_sub = self.create_subscription(String, '/esp_status', self.status_callback, 10)
        self.publisher = self.create_publisher(String, '/esp_command', 10)
        
        # 비전 창 업데이트용 상태 퍼블리셔
        self.state_pub = self.create_publisher(String, '/control_state', 10)

        # 원본 제어 FSM 변수 완벽 이식
        self.state = "CRUISE"
        self.last_command = ""
        self.stop_start_time = 0.0
        self.red_line_latched = False
        self.obstacle_latched = False
        self.avoid_direction = 0
        self.avoid_substate = 0
        self.waiting_for_esp = False

    def status_callback(self, msg):
        if msg.data == "DONE":
            self.waiting_for_esp = False
            self.get_logger().info("ESP32 작업 완료 신호(DONE) 접수.")

    def send_command(self, cmd_str):
        if cmd_str != self.last_command:
            msg = String()
            msg.data = cmd_str
            self.publisher.publish(msg)
            self.last_command = cmd_str
            self.get_logger().info(f"Sent to ESP32: {cmd_str}")
        
        # 현재 상태를 비전 노드로 공유 (디버그 텍스트 동기화용)
        state_msg = String()
        display_state = f"{self.state} Step{self.avoid_substate}" if self.state == "AVOID" else self.state
        if self.waiting_for_esp:
            display_state += " [WAIT]"
        state_msg.data = f"{display_state}|{self.last_command}"
        self.state_pub.publish(state_msg)

    def vision_callback(self, msg):
        # 데이터 언팩 (에러값|정지선|장애물전방|횡단보도|회피방향|장애물자체여부)
        try:
            data = msg.data.split('|')
            error = float(data[0])
            red_line_detected = bool(int(data[1]))
            obstacle_in_front = bool(int(data[2]))
            crosswalk_detected = bool(int(data[3]))
            avoid_dir = int(data[4])
            obstacle_detected = bool(int(data[5]))
        except (ValueError, IndexError):
            return

        # ── 래치 해제 제어 ──
        if not red_line_detected:
            self.red_line_latched = False
        if not obstacle_detected:
            self.obstacle_latched = False

        current_time = time.time()

        # ── 상태 전환 FSM ──
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

        # ── FSM 명령 실행 (원본 구조 100% 보존) ──
        
        # 횡단보도 위 처리
        if crosswalk_detected and self.state != "AVOID":
            self.last_command = ""
            self.send_command("G200")

        elif self.state == "STOP_TIMER":
            if current_time - self.stop_start_time > 1.5:
                self.state = "CRUISE"
            else:
                self.send_command("S")

        elif self.state == "AVOID":
            if not self.waiting_for_esp:
                if self.avoid_substate == 1:
                    self.send_command(f"T{45 * self.avoid_direction}")
                    self.waiting_for_esp = True
                    self.avoid_substate = 2
                elif self.avoid_substate == 2:
                    self.send_command("G200")
                    self.waiting_for_esp = True
                    self.avoid_substate = 3
                elif self.avoid_substate == 3:
                    self.send_command(f"T{-45 * self.avoid_direction}")
                    self.waiting_for_esp = True
                    self.avoid_substate = 4
                elif self.avoid_substate == 4:
                    self.state = "CRUISE"
                    self.obstacle_latched = True
                    self.last_command = ""

        elif self.state == "CRUISE":
            if abs(error) < 10:
                self.send_command("G200")
            else:
                turn_deg = int(error * -0.1)
                self.send_command(f"T{turn_deg}" if abs(turn_deg) >= 2 else "G200")

def main(args=None):
    rclpy.init(args=args)
    node = ControlHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
