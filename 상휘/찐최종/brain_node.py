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

        self.yolo_sub  = self.create_subscription(String, '/traffic_sign_topic', self.yolo_callback, 10)

        lidar_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.lidar_sub = self.create_subscription(Float32MultiArray, '/obstacle_status', self.lidar_callback, lidar_qos)

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
        self.pending_lane_change = 0

        # PID 파라미터
        self.kp = 0.02
        self.ki = 0.001
        self.kd = 0.0

        self.prev_error = 0.0
        self.integral   = 0.0
        self.prev_time  = time.time()

        # 긴급 정지 플래그
        self.emergency_stop = False

        # ── YOLO 관련 변수 ──
        self.yolo_action          = None          # 마지막 수신 토픽
        self.speed_limit_end_time = 0.0           # limit_sign 속도 제한 종료 시각
        self.stop_sign_start_time = 0.0           # stop_sign 5초 타이머
        self.stop_sign_done_time  = 0.0           # stop_sign 완료 시각 (쿨다운용)
        self.stop_sign_cooldown   = 10.0          # stop_sign 재진입 금지 시간(초)
        self.redlight_active      = False         # redlight 대기 중 플래그

        # ── 좌회전(traffic_left_light_green) 시퀀스 변수 ──
        # 시퀀스: 노란선 사라진 뒤 0.3초 직진 → 왼쪽 90도 회전 → CRUISE
        # 이후 첫 번째 빨간선 무시, 두 번째 빨간선 사라진 뒤 0.3초 후 영구 정지
        self.tl_substate          = 0             # 0=비활성
        self.tl_yellow_seen       = False
        self.tl_yellow_lost_time  = 0.0
        self.tl_red_count         = 0             # 좌회전 후 빨간선 통과 횟수
        self.tl_red_seen          = False
        self.tl_red_lost_time     = 0.0
        self.tl_final_stop        = False         # 영구 정지 플래그

        # ── LiDAR 관련 변수 ──
        self.lidar_obstacle_detected  = False
        self.last_lidar_time          = 0.0
        self.obstacle_wait_start_time = 0.0
        self.last_obs_log_time        = 0.0
        self.last_lidar_rx_log_time   = 0.0

        # AVOID 노란선 추적 변수
        self.yellow_seen_in_avoid = False
        self.yellow_lost_time     = 0.0

        # 키보드 입력 스레드
        self.kb_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self.kb_thread.start()
        self.get_logger().info("키보드 리스너 시작 — 's' 키: 긴급 정지 / 'r' 키: 재개")

    # ─────────────────────────────────────────────
    # YOLO 콜백
    # ─────────────────────────────────────────────
    def yolo_callback(self, msg):
        action = msg.data
        self.get_logger().info(f"🚦 [YOLO] 표지판/신호 감지: {action}")
        self.yolo_action = action

        if action == "limit_sign":
            # 5초간 속도 제한
            self.speed_limit_end_time = time.time() + 5.0

        elif action == "redlight":
            # 신호 대기 진입 (greenlight 또는 traffic_left_light_green 올 때까지)
            if self.state == "CRUISE":
                self.get_logger().info("🔴 적신호 감지 → 신호 대기 진입")
                self.state = "REDLIGHT_WAIT"
                self.redlight_active = True

        elif action == "greenlight":
            # 단순 직진 재개
            if self.state == "REDLIGHT_WAIT":
                self.get_logger().info("🟢 녹신호 감지 → 주행 재개")
                self.state = "CRUISE"
                self.redlight_active = False
                self._reset_pid()

        elif action == "traffic_left_light_green":
            # 좌회전 시퀀스 진입 (CRUISE 또는 REDLIGHT_WAIT 상태에서 접수)
            if self.state in ["CRUISE", "REDLIGHT_WAIT"]:
                self.get_logger().info("🟢↰ 좌회전 녹신호 감지 → 좌회전 시퀀스 진입")
                self.state            = "TL_TURN_LEFT"
                self.redlight_active  = False
                self.tl_substate      = 1
                self.tl_yellow_seen   = False
                self.tl_yellow_lost_time = 0.0
                self._reset_pid()

        elif action == "stop_sign":
            # 5초 정지 (쿨다운 내 재진입 금지)
            if self.state == "CRUISE" and time.time() - self.stop_sign_done_time > self.stop_sign_cooldown:
                self.get_logger().info("🛑 정지 표지판 감지 → 5초 정지")
                self.state = "STOP_SIGN_WAIT"
                self.stop_sign_start_time = time.time()

    # ─────────────────────────────────────────────
    # LiDAR 콜백
    # ─────────────────────────────────────────────
    def lidar_callback(self, msg):
        if len(msg.data) >= 1 and msg.data[0] == 1.0:
            self.lidar_obstacle_detected = True
            self.last_lidar_time = time.time()

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
                    self.emergency_stop = False
                    self.last_command   = ""
                    self.waiting_for_esp= False
                    self._reset_pid()
                    self.get_logger().info("긴급 정지 해제 — 주행 재개")
                elif ch == '\x03':
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _force_stop(self):
        msg      = String()
        msg.data = "S"
        self.publisher.publish(msg)
        self.last_command = "S"

    def _reset_pid(self):
        self.prev_error = 0.0
        self.integral   = 0.0
        self.prev_time  = time.time()

    # ─────────────────────────────────────────────
    # ESP32 통신
    # ─────────────────────────────────────────────
    def status_callback(self, msg):
        if msg.data == "DONE":
            self.waiting_for_esp = False
            self.last_command    = ""
            self.prev_error      = 0.0
            self.integral        = 0.0

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

            if cmd_str.startswith("T") or cmd_str == "S":
                self.waiting_for_esp = True
                self.wait_start_time = time.time()
            else:
                self.waiting_for_esp = False

        elif cmd_str == self.last_command and self.waiting_for_esp:
            msg      = String()
            msg.data = cmd_str
            self.publisher.publish(msg)

        # 상태 발행
        state_msg = String()
        if hasattr(self, 'pending_lane_change') and self.pending_lane_change > 0:
            display_state = f"LANE_CHANGE:{self.pending_lane_change}"
            self.pending_lane_change = 0
        else:
            if self.state == "AVOID":
                display_state = f"{self.state} Step{self.avoid_substate}"
            elif self.state == "TL_TURN_LEFT":
                display_state = f"{self.state} Step{self.tl_substate}"
            else:
                display_state = self.state

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

        # 영구 정지 상태면 아무것도 하지 않음
        if self.tl_final_stop:
            self.send_command("S")
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

        # 라이다 신호 0.3초 없으면 안전으로 리셋
        if current_time - self.last_lidar_time > 0.3:
            self.lidar_obstacle_detected = False

        # 속도 제한 (limit_sign)
        if current_time < self.speed_limit_end_time:
            self.current_speed = 80
        else:
            self.current_speed = 115

        # ESP32 응답 타임아웃
        if self.waiting_for_esp and (current_time - self.wait_start_time > 0.5):
            self.get_logger().warn("ESP32 응답 시간 초과 (0.5초). 다음 명령으로 넘어갑니다.")
            self.waiting_for_esp = False
            self.last_command    = ""
            self._reset_pid()
            self.prev_time = current_time

        # ── 상태 전환 (CRUISE일 때만) ──
        if self.state == "CRUISE" and not crosswalk_detected:
            # 빨간선 추적
            if red_line_detected:
                self.red_line_seen      = True
                self.red_line_lost_time = 0.0
            elif self.red_line_seen and self.red_line_lost_time == 0.0:
                self.red_line_lost_time = current_time

            if self.red_line_seen and not red_line_detected:
                if current_time - self.red_line_lost_time >= 0.3:
                    self.get_logger().info("바닥 정지선(Red Line) 통과 확인 → 1.5초 정지 타이머")
                    self.state              = "STOP_TIMER"
                    self.stop_start_time    = current_time
                    self.red_line_seen      = False
                    self.red_line_lost_time = 0.0
            elif self.lidar_obstacle_detected:
                self.get_logger().warn("🛑 LiDAR 전방 장애물 감지 → 3초 판별 대기")
                self.state                    = "OBSTACLE_WAIT"
                self.obstacle_wait_start_time = current_time
                self.last_obs_log_time        = current_time
                self.avoid_direction          = avoid_direction

        # ── 명령 실행 ──

        # 횡단보도 최우선 (일부 시퀀스 중엔 무시)
        if crosswalk_detected and self.state not in ["AVOID", "TL_TURN_LEFT", "OBSTACLE_WAIT",
                                                      "REDLIGHT_WAIT", "STOP_SIGN_WAIT", "TL_FINAL_APPROACH"]:
            self.last_command    = ""
            self.waiting_for_esp = False
            self.send_command(f"G{self.current_speed}")

        # ── 적신호 대기 ──
        elif self.state == "REDLIGHT_WAIT":
            # greenlight / traffic_left_light_green 은 yolo_callback에서 상태 전환됨
            self.send_command("S")

        # ── 정지 표지판 5초 대기 ──
        elif self.state == "STOP_SIGN_WAIT":
            if current_time - self.stop_sign_start_time >= 5.0:
                self.get_logger().info("🛑 정지 표지판 5초 경과 → 주행 재개")
                self.state = "CRUISE"
                self.stop_sign_done_time = current_time  # 쿨다운 시작
                self._reset_pid()
            else:
                self.send_command("S")

        # ── 장애물 3초 판별 대기 ──
        elif self.state == "OBSTACLE_WAIT":
            if not self.lidar_obstacle_detected:
                self.get_logger().info("🟢 장애물 사라짐 (동적) → 주행 재개")
                self.state = "CRUISE"
                self._reset_pid()
            elif current_time - self.obstacle_wait_start_time >= 3.0:
                self.get_logger().warn(f"🚨 정적 장애물 확정 → 회피({self.avoid_direction}) 기동")
                self.state               = "AVOID"
                self.avoid_substate      = 1
                self.waiting_for_esp     = False
                self.yellow_seen_in_avoid= False
                self.yellow_lost_time    = 0.0
            else:
                self.send_command("S")
                if current_time - self.last_obs_log_time >= 0.5:
                    elapsed = current_time - self.obstacle_wait_start_time
                    self.get_logger().info(f"⚠️ 장애물 대기 중... ({elapsed:.1f}s / 3.0s)")
                    self.last_obs_log_time = current_time

        # ── 바닥 정지선 1.5초 정지 ──
        elif self.state == "STOP_TIMER":
            if current_time - self.stop_start_time > 1.5:
                self.get_logger().info("1.5초 정지 완료 → 주행 재개")
                self.state = "CRUISE"
            else:
                self.send_command("S")

        # ── 좌회전 시퀀스 (traffic_left_light_green) ──
        # Step1: 노란선 사라진 뒤 0.3초까지 직진
        # Step2: 왼쪽 90도 회전
        # Step3: CRUISE 복귀, 이후 빨간선 카운트 시작
        elif self.state == "TL_TURN_LEFT":
            if self.tl_substate == 1:
                # 전진하며 노란선 추적
                self.send_command(f"G{self.current_speed}")

                if yellow_line_detected:
                    if not self.tl_yellow_seen:
                        self.get_logger().info("↰ 좌회전 Step1: 노란선 감지 — 통과 대기 중")
                    self.tl_yellow_seen      = True
                    self.tl_yellow_lost_time = 0.0
                elif self.tl_yellow_seen and self.tl_yellow_lost_time == 0.0:
                    self.tl_yellow_lost_time = current_time
                    self.get_logger().info("↰ 좌회전 Step1: 노란선 사라짐 — 0.3초 카운트 시작")

                if self.tl_yellow_seen and self.tl_yellow_lost_time > 0.0:
                    if current_time - self.tl_yellow_lost_time >= 0.25:
                        self.get_logger().info("↰ 좌회전 Step1: 0.3초 직진 완료 → 왼쪽 90도 회전")
                        self.tl_yellow_seen      = False
                        self.tl_yellow_lost_time = 0.0
                        self.tl_substate         = 2

            elif self.tl_substate == 2:
                # 왼쪽 90도 회전 명령
                if not self.waiting_for_esp:
                    self.send_command("T90")
                    self.tl_substate = 3

            elif self.tl_substate == 3:
                # 회전 완료 대기
                if self.waiting_for_esp:
                    self.send_command("T90")
                    return
                self.get_logger().info("↰ 좌회전 완료 → CRUISE 복귀, 빨간선 카운트 시작")
                self.state       = "TL_FINAL_APPROACH"
                self.tl_substate = 0
                self.tl_red_count     = 0
                self.tl_red_seen      = False
                self.tl_red_lost_time = 0.0
                self._reset_pid()

        # ── 좌회전 후 최종 접근 (빨간선 2번째 통과 후 영구 정지) ──
        elif self.state == "TL_FINAL_APPROACH":
            # 빨간선 추적
            if red_line_detected:
                self.tl_red_seen      = True
                self.tl_red_lost_time = 0.0
            elif self.tl_red_seen and self.tl_red_lost_time == 0.0:
                self.tl_red_lost_time = current_time

            # 빨간선이 사라진 뒤 0.3초 경과 시 카운트
            if self.tl_red_seen and self.tl_red_lost_time > 0.0:
                if current_time - self.tl_red_lost_time >= 0.3:
                    self.tl_red_count += 1
                    self.tl_red_seen      = False
                    self.tl_red_lost_time = 0.0
                    self.get_logger().info(f"🔴 빨간선 통과 카운트: {self.tl_red_count}/2")

                    if self.tl_red_count == 1:
                        self.get_logger().info("🔴 첫 번째 빨간선 무시 → 계속 주행")
                    elif self.tl_red_count >= 2:
                        self.get_logger().info("🏁 두 번째 빨간선 통과 → 영구 정지!")
                        self.tl_final_stop = True
                        self.send_command("S")
                        return

            # 영구 정지 전까지 PID 주행
            if self.waiting_for_esp:
                return

            now = time.time()
            dt  = now - self.prev_time
            if dt <= 0:
                dt = 0.01

            self.integral  += error * dt
            self.integral   = max(-500.0, min(500.0, self.integral))
            derivative      = (error - self.prev_error) / dt
            pid_output      = -((self.kp * error) + (self.ki * self.integral) + (self.kd * derivative))
            self.prev_error = error
            self.prev_time  = now

            turn_deg = int(pid_output)
            turn_deg = max(-5, min(5, turn_deg))

            if abs(turn_deg) < 2:
                self.send_command(f"G{self.current_speed}")
            else:
                self.send_command(f"T{turn_deg}")

        # ── 장애물 회피 ──
        elif self.state == "AVOID":
            if self.avoid_substate == 1:
                if not self.waiting_for_esp:
                    self.get_logger().info(f"회피 Step1: {'우' if self.avoid_direction == 1 else '좌'} 90도 회전")
                    self.send_command(f"T{-90 * self.avoid_direction}")
                    self.avoid_substate = 2

            elif self.avoid_substate == 2:
                if self.waiting_for_esp:
                    self.send_command(f"T{-90 * self.avoid_direction}")
                    return
                self.send_command(f"G{self.current_speed}")

                if yellow_line_detected:
                    if not self.yellow_seen_in_avoid:
                        self.get_logger().info("회피 Step2: 노란선 감지 — 통과 대기 중")
                    self.yellow_seen_in_avoid = True
                    self.yellow_lost_time     = 0.0
                elif self.yellow_seen_in_avoid and self.yellow_lost_time == 0.0:
                    self.yellow_lost_time = current_time
                    self.get_logger().info("회피 Step2: 노란선 사라짐 — 0.3초 카운트 시작")

                if self.yellow_seen_in_avoid and self.yellow_lost_time > 0.0:
                    if current_time - self.yellow_lost_time >= 0.3:
                        self.get_logger().info("회피 Step2: 노란선 통과 확인 → 반대 방향 90도 회전")
                        self.yellow_seen_in_avoid = False
                        self.yellow_lost_time     = 0.0
                        self.avoid_substate       = 3

            elif self.avoid_substate == 3:
                if not self.waiting_for_esp:
                    self.get_logger().info(f"회피 Step3: {'좌' if self.avoid_direction == 1 else '우'} 90도 회전")
                    self.send_command(f"T{90 * self.avoid_direction}")
                    self.avoid_substate = 4

            elif self.avoid_substate == 4:
                if self.waiting_for_esp:
                    self.send_command(f"T{90 * self.avoid_direction}")
                    return
                new_lane = 2 if self.avoid_direction == 1 else 1
                self.get_logger().info(f"✅ 장애물 회피 완료 → {new_lane}차선, 주행 재개")
                self.state          = "CRUISE"
                self.avoid_substate = 1
                self._reset_pid()
                lane_msg      = String()
                lane_msg.data = f"LANE_CHANGE:{new_lane}|"
                self.state_pub.publish(lane_msg)

        # ── 기본 주행 (PID) ──
        elif self.state == "CRUISE":
            if self.waiting_for_esp:
                return

            now = time.time()
            dt  = now - self.prev_time
            if dt <= 0:
                dt = 0.01

            self.integral  += error * dt
            self.integral   = max(-500.0, min(500.0, self.integral))
            derivative      = (error - self.prev_error) / dt
            pid_output      = -((self.kp * error) + (self.ki * self.integral) + (self.kd * derivative))
            self.prev_error = error
            self.prev_time  = now

            turn_deg = int(pid_output)
            turn_deg = max(-5, min(5, turn_deg))

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
