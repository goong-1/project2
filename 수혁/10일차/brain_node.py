import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
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

        # ★ YOLO 토픽 구독
        self.yolo_sub   = self.create_subscription(String, '/traffic_sign_topic', self.yolo_callback, 10)
        
        # ★ [수정] LiDAR 토픽 구독 (라이다 노드의 BEST_EFFORT QoS 규격에 맞춰야 통신이 연결됨)
        lidar_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.lidar_sub  = self.create_subscription(Float32MultiArray, '/obstacle_status', self.lidar_callback, lidar_qos)

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

        # YOLO / LiDAR 관련 변수
        self.yolo_action = "GO"
        self.speed_limit_end_time = 0.0
        self.turn_left_substate = 1
        self.turn_left_timer = 0.0
        self.tl_g_sent = False
        
        self.lidar_obstacle_detected = False
        self.last_lidar_time = 0.0
        self.obstacle_wait_start_time = 0.0
        self.last_obs_log_time = 0.0 
        self.last_lidar_rx_log_time = 0.0 # [추가] 통신 확인 로그 주기 제어용

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
            
            # ★ [추가] 통신 정상 여부를 터미널에서 즉시 확인하기 위한 수신 로그 (도배 방지를 위해 0.5초마다 출력)
            if time.time() - self.last_lidar_rx_log_time > 0.5:
                self.get_logger().info("📥 [통신 확인] 라이다 노드로부터 장애물 감지 신호(1.0) 수신!")
                self.last_lidar_rx_log_time = time.time()

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

        # 라이다가 위험할 때만 신호를 쏘므로, 0.3초 이상 신호가 없으면 안전한 것으로 리셋
        if current_time - self.last_lidar_time > 0.3:
            self.lidar_obstacle_detected = False

        # YOLO 속도 제한 (SPEED_LIMIT)
        if current_time < self.speed_limit_end_time:
            self.current_speed = 80
        else:
            self.current_speed = 115

        # 빨간 선이 보였다가 사라지는 시점을 추적
        if red_line_detected:
            self.red_line_seen = True
            self.red_line_lost_time = 0.0
        elif self.red_line_seen and self.red_line_lost_time == 0.0:
            self.red_line_lost_time = current_time
        
        if self.waiting_for_esp and (current_time - self.wait_start_time > 0.5):
            self.get_logger().warn("ESP32 응답 시간 초과 (0.5초). 다음 명령으로 넘어갑니다.")
            self.waiting_for_esp = False
            self.last_command = ""
            self.prev_error = 0.0  
            self.integral = 0.0
            self.prev_time = current_time
            
        # ── 상태 전환 ──
        if self.state == "CRUISE" and not crosswalk_detected:
            # 1순위: YOLO STOP
            if getattr(self, 'yolo_action', 'GO') == "STOP":
                self.get_logger().info("YOLO 빨간불(STOP) 신호 감지 -> 정지 모드 진입")
                self.state = "YOLO_STOP"
            # 2순위: 정지선
            elif self.red_line_seen and not red_line_detected:
                if current_time - self.red_line_lost_time >= 0.1:
                    self.get_logger().info("바닥 정지선(Red Line) 통과 확인 -> 1.5초 정지 타이머")
                    self.state              = "STOP_TIMER"
                    self.stop_start_time    = current_time
                    self.red_line_seen      = False
                    self.red_line_lost_time = 0.0
            # 3순위: LiDAR 전방 장애물 감지
            elif self.lidar_obstacle_detected:
                self.get_logger().warn("🛑 [위험] 라이다(LiDAR) 전방 장애물 감지! 즉시 정지명령(S) 하달 및 3초 판별 대기 시작")
                self.state = "OBSTACLE_WAIT"
                self.obstacle_wait_start_time = current_time
                self.last_obs_log_time = current_time
                self.avoid_direction = avoid_direction  
            # 4순위: YOLO 좌회전
            elif getattr(self, 'yolo_action', 'GO') == "TURN_LEFT":
                self.get_logger().info("YOLO 좌회전 신호 감지 -> 좌회전 시퀀스 진입")
                self.state = "YOLO_TURN_LEFT"
                self.turn_left_substate = 1
                self.turn_left_timer = current_time

        # ── 명령 실행 ──
        # 횡단보도 최우선 처리 (시퀀스 중이 아닐 때)
        if crosswalk_detected and self.state not in ["AVOID", "YOLO_TURN_LEFT", "OBSTACLE_WAIT"]:
            self.last_command    = ""
            self.waiting_for_esp = False
            self.send_command(f"G{self.current_speed}")

        # 장애물 3초 판별 대기
        elif self.state == "OBSTACLE_WAIT":
            if not self.lidar_obstacle_detected:
                self.get_logger().info("🟢 장애물 사라짐 (동적 장애물 판단) -> 모터 정지 해제 및 주행 재개")
                self.state = "CRUISE"
                self.prev_error = 0.0
                self.integral = 0.0
                self.prev_time = time.time()
            elif current_time - self.obstacle_wait_start_time >= 3.0:
                self.get_logger().warn(f"🚨 3초 경과 (정적 장애물 확정) -> 차선 회피({self.avoid_direction}) 기동 진입")
                self.state = "AVOID"
                self.avoid_substate = 1
                self.waiting_for_esp = False
            else:
                self.send_command("S")
                if current_time - getattr(self, 'last_obs_log_time', 0) >= 0.5:
                    elapsed = current_time - self.obstacle_wait_start_time
                    self.get_logger().info(f"⚠️ 전방 장애물 대기 중... 모터 정지(S) 유지 ({elapsed:.1f}초 / 3.0초)")
                    self.last_obs_log_time = current_time

        # YOLO 신호 대기
        elif self.state == "YOLO_STOP":
            if getattr(self, 'yolo_action', 'GO') == "GO":
                self.get_logger().info("YOLO 초록불(GO) 감지 -> 주행 재개")
                self.state = "CRUISE"
                self.prev_error = 0.0
                self.integral = 0.0
                self.prev_time = time.time()
            else:
                self.send_command("S")

        # 정지선 1.5초 정지
        elif self.state == "STOP_TIMER":
            if current_time - self.stop_start_time > 1.5:
                self.get_logger().info("1.5초 정지 완료 -> 주행 재개")
                self.state = "CRUISE"
            else:
                self.send_command("S")

        # YOLO 하드코딩 좌회전 시퀀스
        elif self.state == "YOLO_TURN_LEFT":
            if self.turn_left_substate == 1:
                if current_time - self.turn_left_timer <= 1.0:
                    self.send_command(f"G{self.current_speed}")
                else:
                    self.get_logger().info("좌회전: 1초 직진 완료 -> 1차 45도 회전 시작")
                    self.turn_left_substate = 2

            elif self.turn_left_substate == 2:
                if not self.waiting_for_esp:
                    self.send_command("T45")
                    self.turn_left_substate = 3

            elif self.turn_left_substate == 3:
                if self.waiting_for_esp:
                    self.send_command("T45")
                    return
                if not getattr(self, 'tl_g_sent', False):
                    self.turn_left_timer = current_time
                    self.tl_g_sent = True
                
                if current_time - self.turn_left_timer <= 0.5:
                    self.send_command(f"G{self.current_speed}")
                else:
                    self.get_logger().info("좌회전: 0.5초 직진 완료 -> 2차 45도 회전 시작")
                    self.turn_left_substate = 4
                    self.tl_g_sent = False

            elif self.turn_left_substate == 4:
                if not self.waiting_for_esp:
                    self.send_command("T45")
                    self.turn_left_substate = 5

            elif self.turn_left_substate == 5:
                if self.waiting_for_esp:
                    self.send_command("T45")
                    return
                self.get_logger().info("좌회전 시퀀스 모두 완료 -> 기본 주행(CRUISE) 복귀")
                self.state = "CRUISE"
                self.turn_left_substate = 1
                self.prev_error = 0.0
                self.integral = 0.0
                self.prev_time = time.time()

        # 장애물 회피 기동
        elif self.state == "AVOID":
            if self.avoid_substate == 1:
                if not self.waiting_for_esp:
                    self.send_command(f"T{45 * self.avoid_direction}")
                    self.avoid_substate  = 2

            elif self.avoid_substate == 2:
                if self.waiting_for_esp:
                    self.send_command(f"T{45 * self.avoid_direction}")
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
                if self.waiting_for_esp:
                    self.send_command(f"T{-45 * self.avoid_direction}")
                    return
                # 복귀 완료
                self.get_logger().info("장애물 회피 완료 -> 기본 주행(CRUISE) 복귀")
                self.state          = "CRUISE"
                self.avoid_substate = 1
                self.prev_error     = 0.0
                self.integral       = 0.0
                self.prev_time      = time.time()

        elif self.state == "CRUISE":
            # ── PID 제어 ──
            if self.waiting_for_esp:
                return

            now = time.time()
            dt  = now - self.prev_time
            if dt <= 0:
                dt = 0.01

            self.integral += error * dt
            self.integral  = max(-500.0, min(500.0, self.integral))

            derivative = (error - self.prev_error) / dt
            pid_output = -((self.kp * error) + (self.ki * self.integral) + (self.kd * derivative))

            self.prev_error = error
            self.prev_time  = now

            turn_deg = int(pid_output)
            turn_deg = max(-5, min(5, turn_deg))

            if abs(error) < 30 or turn_deg < 1:
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
