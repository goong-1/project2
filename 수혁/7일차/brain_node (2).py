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

        # 0.2초 주기로 마지막 명령을 지속 전송하는 타이머 (ESP32 타임아웃 방지)
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
        self.stop_done_time = 0.0  # 정지 명령 후 ESP32로부터 대답을 수신한 시점

    def status_callback(self, msg):
        if msg.data == "DONE":
            if self.waiting_for_esp:
                self.waiting_for_esp = False
                # 마지막 명령이 정지(S)였다면 대답을 받은 시점 저장
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
        
        # 완전히 동일한 명령이면 중복 발송하지 않고 0.2초 타이머에 위임
        if cmd_str == self.last_command:
            return

        # ── 다른 명령으로 전환할 때의 필터링 조건 ──
        if self.last_command:
            # 1. 마지막 명령이 회전(T)이었던 경우: 완료 신호(DONE)가 올 때까지 전환 불가
            if self.last_command.startswith("T") and self.waiting_for_esp:
                return
            
            # 2. 마지막 명령이 정지(S)였던 경우: 대답을 수신하고 1초가 지나야 전환 가능
            if self.last_command == "S":
                if self.waiting_for_esp:
                    return
                if self.stop_done_time == 0.0 or (current_time - self.stop_done_time < 1.0):
                    return
            
            # 3. 마지막 명령이 직진(G)이었던 경우: 제약 없이 즉시 새 명령 수행

        # 조건 통과 시 새 명령 등록
        self.last_command = cmd_str
        self.stop_done_time = 0.0
        
        # 새 명령이 회전(T)이거나 정지(S)인 경우 피드백을 기다리도록 플래그 세팅
        if cmd_str.startswith("T") or cmd_str == "S":
            self.waiting_for_esp = True
        else:
            self.waiting_for_esp = False

        # 변경 즉시 반응성을 극대화하기 위해 1회 강제 발행
        msg = String()
        msg.data = cmd_str
        self.publisher.publish(msg)
        self.get_logger().info(f"명령 전환 -> Bridge 전송: {cmd_str}")
        
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

        # ── FSM 상태별 명령 하달 (속도 기반 반영) ──
        
        # 횡단보도 위 처리 -> 느린 속도인 80으로 직진
        if crosswalk_detected and self.state != "AVOID":
            self.send_command("G80")

        elif self.state == "STOP_TIMER":
            if current_time - self.stop_start_time > 1.5:
                self.state = "CRUISE"
            else:
                self.send_command("S")

        elif self.state == "AVOID":
            # 회전이나 주행 잠금이 해제된 안전 시점에만 다음 서브 스테이지 진행
            if not self.waiting_for_esp:
                if self.avoid_substate == 1:
                    # 회전 기동
                    self.send_command(f"T{45 * self.avoid_direction}")
                    self.avoid_substate = 2
                elif self.avoid_substate == 2:
                    # 회피 주행은 조심스럽게 느린 속도 80으로 전진
                    self.send_command("G80")
                    # 속도 명령(G)은 대답을 기다리지 않으므로 서브스테이지 강제 진행을 막기 위해 잠시 대기 시간 부여 가능
                    # 여기서는 속도 하달 후 즉시 다음 복귀 회전 준비를 위해 substate를 넘깁니다.
                    self.avoid_substate = 3
                elif self.avoid_substate == 3:
                    # 복귀 회전 기동
                    self.send_command(f"T{-45 * self.avoid_direction}")
                    self.avoid_substate = 4
                elif self.avoid_substate == 4:
                    self.state = "CRUISE"
                    self.obstacle_latched = True
                    self.send_command("G115")

        elif self.state == "CRUISE":
            if abs(error) < 20:
                # 정상 차선 안일 때는 지정해주신 빠른 주행 속도 115로 직진
                self.send_command("G115")
            else:
                # 차선을 벗어나서 조향이 필요할 때는 지정 각도로 회전 명령 하달
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