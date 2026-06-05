import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import time
import threading
import sys
import tty
import termios
import os

os.environ['ROS_DOMAIN_ID'] = '30'

class ControlHandler(Node):

    def __init__(self):
        super().__init__('control_handler')

        self.vision_sub = self.create_subscription(String, '/vision_status', self.vision_callback, 10)
        self.status_sub = self.create_subscription(String, '/esp_status', self.status_callback, 10)
        self.publisher  = self.create_publisher(String, '/esp_command', 10)
        self.state_pub  = self.create_publisher(String, '/control_state', 10)

        # ★ YOLO 및 LiDAR 토픽 구독 추가
        self.yolo_sub   = self.create_subscription(String, '/traffic_sign_topic', self.yolo_callback, 10)
        self.lidar_sub  = self.create_subscription(Float32MultiArray, '/obstacle_status', self.lidar_callback, 10)

        self.transmit_interval = 0.2
        self.transmit_timer = self.create_timer(self.transmit_interval, self.timer_fallback_transmit)

        # FSM 변수
        self.state             = "CRUISE"
        self.last_command      = ""
        self.stop_start_time   = 0.0
        self.red_line_seen     = False  
        self.red_line_lost_time= 0.0    
        self.avoid_direction   = 0
        self.avoid_substate    = 0
        self.waiting_for_esp   = False
        self.wait_start_time   = 0.0  
        self.current_speed     = 115

        # PID 파라미터
        self.kp = 0.02
        self.ki = 0.001
        self.kd = 0.0

        self.prev_error = 0.0
        self.integral   = 0.0
        self.prev_time  = time.time()

        # 긴급 정지 플래그
        self.emergency_stop = False

        # ★ YOLO / LiDAR 관련 변수
        self.yolo_action = "GO"
        self.speed_limit_end_time = 0.0
        self.turn_left_substate = 1
        self.turn_left_timer = 0.0
        self.tl_g_sent = False
        
        self.lidar_obstacle_detected = False
        self.last_lidar_time = 0.0
        self.obstacle_wait_start_time = 0.0
        self.last_obs_log_time = 0.0 # [추가] 장애물 대기 로그 주기 제어용

        # 키보드 입력 스레드
        self.kb_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self.kb_thread.start()
        self.get_logger().info("키보드 리스너 시작 — 's' 키: 긴급 정지 / 'r' 키: 재개")

    # ─────────────────────────────────────────────
    # LiDAR & YOLO 콜백
    # ─────────────────────────────────────────────
    def yolo_callback(self, msg):
        self.yolo_action = msg.data
        if self.yolo_action == "SPEED_LIMIT":
            self.speed_limit_end_time = time.time() + 5.0

    def lidar_callback(self, msg):
        # lidar_node가 위험할 때만 [1.0]을 보내므로, 토픽이 오면 발견된 것으로 처리
        if len(msg.data) >= 1 and msg.data[0] == 1.0:
            self.lidar_obstacle_detected = True
            self.last_lidar_time = time.time()

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
                    self.get_logger().warn("긴급 정지 활성화 (재개: 'r')")
                    self._force_stop()
                elif ch == 'r':
                    self.emergency_stop  = False
                    self.last_command    = ""
                    self.waiting_for_esp = False
                    self.prev_error      = 0.0
                    self.integral        = 0.0
                    self.prev_time       = time.time()
                    self.get_logger().info("긴급 정지 해제 — 주행 재개")
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
            self.prev_error      = 0.0   # DONE 수신 시 PID 리셋
            self.integral        = 0.0
            # self.get_logger().info("ESP32 작업 완료 신호(DONE) 접수.")

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
            # self.get_logger().info(f"Sent to ESP32: {cmd_str}")  # 터미널 도배 방지를 위해 생략 가능

            # T, S 만 DONE 대기 / G 는 즉시 다음 명령 허용
            if cmd_str.startswith("T") or cmd_str == "S":
                self.waiting_for_esp = True
                self.wait_start_time = time.time()
            else:
                self.waiting_for_esp = False

        elif cmd_str == self.last_command and self.waiting_for_esp:
            msg      = String()
            msg.data = cmd_str
            self.publisher.publish(msg)

        state_msg = String()
        display_state = (
            f"{self.state} Step{self.avoid_substate}"
            if self.state == "AVOID" else self.state
        )
        if self.state == "YOLO_TURN_LEFT":
            display_state = f"{self.state} Step{self.turn_left_substate}"
            
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
            data                 = msg.data.split('|')
            error                = float(data[0])
            red_line_detected    = bool(int(data[1]))
            crosswalk_detected   = bool(int(data[2]))
            yellow_line_detected = bool(int(data[3]))
            avoid_direction      = int(data[4])
        except (ValueError, IndexError):
            return

        current_time = time.time()

        # ★ 라이다가 위험할 때만 신호를 쏘므로, 0.3초 이상 신호가 없으면 안전한 것으로 리셋
        if current_time - self.last_lidar_time > 0.3:
            self.lidar_obstacle_detected = False

        # ★ YOLO 속도 제한 (SPEED_LIMIT)
        if current_time < self.speed_limit_end_time:
            self.current_speed = 80
        else:
            self.current_speed = 115

        # 빨간 선이 보였다가 사라지는 시점을 추적
        if red_line_detected
