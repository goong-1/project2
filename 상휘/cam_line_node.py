import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading
import sys
import tty
import termios


class ControlHandler(Node):

    def __init__(self):
        super().__init__('control_handler')

        self.vision_sub = self.create_subscription(String, '/vision_status', self.vision_callback, 10)
        self.status_sub = self.create_subscription(String, '/esp_status', self.status_callback, 10)
        self.publisher  = self.create_publisher(String, '/esp_command', 10)
        self.state_pub  = self.create_publisher(String, '/control_state', 10)

        self.transmit_interval = 0.2
        self.transmit_timer = self.create_timer(self.transmit_interval, self.timer_fallback_transmit)

        # FSM 변수
        self.state             = "CRUISE"
        self.last_command      = ""
        self.stop_start_time   = 0.0
        self.red_line_latched  = False
        self.obstacle_latched  = False
        self.avoid_direction   = 0
        self.avoid_substate    = 0
        self.waiting_for_esp   = False
        self.current_speed     = 115

        # ── PID 파라미터 ──
        self.kp = 0.08   # 비례: 클수록 즉각 반응, 너무 크면 진동
        self.ki = 0.001  # 적분: 누적 오차 보정, 너무 크면 과보정
        self.kd = 0.05    # 미분: 급격한 변화 억제, 너무 크면 떨림

        self.prev_error = 0.0
        self.integral   = 0.0
        self.prev_time  = time.time()

        # 긴급 정지 플래그
        self.emergency_stop = False

        # 키보드 입력 스레드
        self.kb_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self.kb_thread.start()
        self.get_logger().info("키보드 리스너 시작 — 's' 키: 긴급 정지 / 'r' 키: 재개")

    # ─────────────────────────────────────────────
    # 키보드 리스너
    # ─────────────────────────────────────────────
    def _keyboard_listener(self):
        fd  = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                ch = sys.stdin.read(1)
                if ch == 's':
                    self.emergency_stop = True
                    self.get_logger().warn("🛑 긴급 정지 활성화 (재개: 'r')")
                    self._force_stop()
                elif ch == 'r':
                    self.emergency_stop = False
                    self.last_command   = ""
                    # PID 상태 초기화 (정지 후 재개 시 적분 누적 리셋)
                    self.prev_error = 0.0
                    self.integral   = 0.0
                    self.prev_time  = time.time()
                    self.get_logger().info("▶ 긴급 정지 해제 — 주행 재개")
                elif ch == '\x03':  # Ctrl+C
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _force_stop(self):
        msg      = String()
        msg.data = "S"
        self.publisher.publish(msg)
        self.last_command = "S"

    # ─────────────────────────────────────────────
    # ESP32 통신
    # ─────────────────────────────────────────────
    def status_callback(self, msg):
        if msg.data == "DONE":
            self.waiting_for_esp = False
            self.last_command    = ""
            self.current_speed   = 115
            self.get_logger().info("ESP32 작업 완료 신호(DONE) 접수.")

    def timer_fallback_transmit(self):
        if self.emergency_stop:
            self._force_stop()
            return
        if self.last_command:
            msg      = String()
            msg.data = self.last_command
            self.publisher.publish(msg)

    def send_command(self, cmd_str):
        if self.emergency_stop:
            return

        if self.waiting_for_esp and cmd_str != self.last_command:
            return

        if cmd_str != self.last_command:
            msg      = String()
            msg.data = cmd_str
            self.publisher.publish(msg)
            self.last_command = cmd_str
            self.get_logger().info(f"Sent to ESP32: {cmd_str}")

            if cmd_str.startswith("T") or cmd_str == "S":
                self.waiting_for_esp = True

        elif cmd_str == self.last_command and self.waiting_for_esp:
            msg      = String()
            msg.data = cmd_str
            self.publisher.publish(msg)

        state_msg = String()
        display_state = (
            f"{self.state} Step{self.avoid_substate}"
            if self.state == "AVOID" else self.state
        )
        if self.waiting_for_esp: display_state += " [WAIT]"
        if self.emergency_stop:  display_state += " [E-STOP]"
        state_msg.data = f"{display_state}|{self.last_command}"
        self.state_pub.publish(state_msg)

    # ─────────────────────────────────────────────
    # 비전 콜백 + FSM
    # ─────────────────────────────────────────────
    def vision_callback(self, msg):
        if self.emergency_stop:
            return

        try:
            data = msg.data.split('|')
            error                = float(data[0])
            red_line_detected    = bool(int(data[1]))
            crosswalk_detected   = bool(int(data[2]))
            yellow_line_detected = bool(int(data[3]))
            avoid_direction      = int(data[4])
        except (ValueError, IndexError):
            return

        if not red_line_detected: self.red_line_latched = False
        #if not obstacle_detected: self.obstacle_latched = False

        current_time = time.time()

        # ── 상태 전환 ──
        if self.state == "CRUISE" and not crosswalk_detected:
            if red_line_detected and not self.red_line_latched:
                self.state            = "STOP_TIMER"
                self.stop_start_time  = current_time
                self.red_line_latched = True
            # elif obstacle_in_front and not self.obstacle_latched:
            #     self.state           = "AVOID"
            #     self.avoid_substate  = 1
            #     self.waiting_for_esp = False
            #     self.avoid_direction = avoid_dir

        # ── 명령 실행 ──
        if crosswalk_detected and self.state != "AVOID":
            self.last_command = ""
            self.send_command(f"G{self.current_speed}")

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
                    self.g_cmd_sent     = False

            elif self.avoid_substate == 3:
                if not self.waiting_for_esp:
                    self.send_command(f"T{-45 * self.avoid_direction}")
                    self.avoid_substate = 4

            elif self.avoid_substate == 4:
                if not self.waiting_for_esp:
                    self.state          = "CRUISE"
                    self.avoid_substate = 1

        elif self.state == "CRUISE":
            # ── PID 제어 ──
            now = time.time()
            dt  = now - self.prev_time
            if dt <= 0:
                dt = 0.01

            self.integral += error * dt
            self.integral  = max(-500.0, min(500.0, self.integral))  # 와인드업 방지

            derivative = (error - self.prev_error) / dt
            pid_output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

            self.prev_error = error
            self.prev_time  = now

            turn_deg = int(-pid_output)
            turn_deg = max(-30, min(30, turn_deg))  # 최대 조향각 ±30도 제한

            if abs(turn_deg) < 2:
                self.send_command(f"G{self.current_speed}")
            else:
                self.send_command(f"T{turn_deg}")


def main(args=None):
    rclpy.init(args=args)
    node = ControlHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
