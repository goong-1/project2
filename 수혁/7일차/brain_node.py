import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class ControlHandler(Node):
    def __init__(self):
        super().__init__('control_handler')
        # 비전 및 ESP32 상태 토픽 구독
        self.vision_sub = self.create_subscription(String, '/vision_status', self.vision_callback, 10)
        self.status_sub = self.create_subscription(String, '/esp_status', self.status_callback, 10)
        
        self.publisher = self.create_publisher(String, '/esp_command', 10)
        self.state_pub = self.create_publisher(String, '/control_state', 10)

        # ★ [추가] 0.2초 주기로 마지막 명령을 지속 전송하는 타이머 가동 (ESP32 타임아웃 방지)
        self.transmit_interval = 0.2
        self.transmit_timer = self.create_timer(self.transmit_interval, self.timer_fallback_transmit)

        self.state = "CRUISE"
        self.last_command = ""
        self.stop_start_time = 0.0
        self.red_line_latched = False
        self.obstacle_latched = False
        self.avoid_direction = 0
        self.avoid_substate = 0
        
        # ESP32 시퀀스 제어 변수
        self.waiting_for_esp = False
        self.stop_done_time = 0.0  # 정지 명령 후 ESP32로부터 대답을 수신한 시점 저장

    def status_callback(self, msg):
        if msg.data == "DONE":
            if self.waiting_for_esp:
                self.waiting_for_esp = False
                # 마지막 명령이 정지(S)였다면 대답을 받은 시점의 타임스탬프 락온
                if self.last_command == "S":
                    self.stop_done_time = time.time()
                self.get_logger().info(f"ESP32 피드백 접수 완료. (마지막 명령: {self.last_command})")

    def timer_fallback_transmit(self):
        """ 타이머에 의해 0.2초마다 마지막 명령을 반복 전송하여 하드웨어 정지 방지 """
        if self.last_command:
            msg = String()
            msg.data = self.last_command
            self.publisher.publish(msg)

    def send_command(self, cmd_str):
        current_time = time.time()
        
        # 완전 동일한 명령인 경우 새롭게 바꿀 필요 없이 0.2초 타이머가 송신하도록 양보
        if cmd_str == self.last_command:
            return

        # ── 새로운 다른 명령으로 전환할 때의 필터링 조건 구조화 ──
        if self.last_command:
            # 1. 마지막 명령이 회전(T)이었던 경우: 완료 신호(DONE)가 올 때까지 다른 명령 수행 불가
            if self.last_command.startswith("T") and self.waiting_for_esp:
                return
            
            # 2. 마지막 명령이 정지(S)였던 경우: 대답을 수신하고 추가로 1초가 지나야 다른 명령 수행 가능
            if self.last_command == "S":
                if self.waiting_for_esp:  # 아직 ESP32 대답 미수신 상태
                    return
                if self.stop_done_time == 0.0 or (current_time - self.stop_done_time < 1.0):  # 대답 수신 후 1초 미만 경과
                    return
            
            # 3. 마지막 명령이 직진(G)이었던 경우: 제약 조건 없이 즉시 통과하여 새 명령 접수

        # 조건 통과 시 새 명령 등록
        self.last_command = cmd_str
        self.stop_done_time = 0.0
        
        # 새 명령이 회전(T)이거나 정지(S)인 경우 피드백을 기다리도록 플래그 세팅
        if cmd_str.startswith("T") or cmd_str == "S":
            self.waiting_for_esp = True
        else:
            self.waiting_for_esp = False

        # 변경 즉시 반응성을 극대화하기 위해 즉시 1회 발행
        msg = String()
        msg.data = cmd_str
        self.publisher.publish(msg)
        self.get_logger().info(f"명령 전환 성공 -> Bridge 전송: {cmd_str}")
        
        # 디버그 상태 공유
        state_msg = String()
        display_state = f"{self.state} Step{self.avoid_substate}" if self.state == "AVOID" else self.state
        if self.waiting_for_esp:
            display_state += " [WAIT]"
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
        except (ValueError, IndexError):
            return

        if not red_line_detected:
            self.red_line_latched = False
        if not obstacle_detected:
            self.obstacle_latched = False

        current_time = time.time()

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

        # ── FSM 상태별 명령 하달 ──
        if crosswalk_detected and self.state != "AVOID":
            self.send_command("G10")

        elif self.state == "STOP_TIMER":
            if current_time - self.stop_start_time > 1.5:
                self.state = "CRUISE"
            else:
                self.send_command("S")

        elif self.state == "AVOID":
            # 회전이나 주행 잠금이 해제된 안전 시점에만 다음 서브 스테이지 명령을 하달
            if not self.waiting_for_esp:
                if self.avoid_substate == 1:
                    self.send_command(f"T{45 * self.avoid_direction}")
                    self.avoid_substate = 2
                elif self.avoid_substate == 2:
                    self.send_command("G50")
                    self.avoid_substate = 3
                elif self.avoid_substate == 3:
                    self.send_command(f"T{-45 * self.avoid_direction}")
                    self.avoid_substate = 4
                elif self.avoid_substate == 4:
                    self.state = "CRUISE"
                    self.obstacle_latched = True
                    self.send_command("G200")

        elif self.state == "CRUISE":
            if abs(error) < 20:
                self.send_command("G200")
            else:
                turn_deg = 15 if error > 0 else -15
                self.send_command(f"T{turn_deg}")

def main(args=None):
    rclpy.init(args=args)
    node = ControlHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
