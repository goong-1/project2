#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import threading

class ControlHandler(Node):
    def __init__(self):
        super().__init__('control_handler')
        
        # 1. 토픽 설정
        self.vision_sub = self.create_subscription(String, '/vision_status', self.vision_callback, 10)
        self.state_pub = self.create_publisher(String, '/control_state', 10)

        # 2. UART 통신 포트 오픈
        try:
            self.ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=0.01)
            self.ser.flush()
            self.get_logger().info('[UART] ESP32 연결용 시리얼 포트(/dev/ttyAMA0) 오픈 완료.')
        except Exception as e:
            self.get_logger().error(f'[UART] 시리얼 포트를 열 수 없습니다: {e}')

        # 3. FSM 및 제어 변수
        self.state = "CRUISE"
        self.last_command = "<s>"
        self.stop_start_time = 0.0
        self.red_line_latched = False
        self.obstacle_latched = False
        self.avoid_direction = 0
        self.avoid_substate = 0
        self.waiting_for_esp = False
        self.avoid_drive_start = 0.0  

        # 4. 0.2초 주기 하트비트 반복 송신 타이머
        self.timer = self.create_timer(0.2, self.heartbeat_loop)
        
        # 5. 수신 전용 백그라운드 스레드
        self.uart_thread = threading.Thread(target=self.uart_read_loop, daemon=True)
        self.uart_thread.start()

    def uart_read_loop(self):
        """ ESP32가 던지는 메시지 중 순수하게 'DONE' 행만 골라내어 락을 해제합니다. """
        while rclpy.ok():
            if hasattr(self, 'ser') and self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line == "DONE":
                        self.waiting_for_esp = False
                        self.get_logger().info(">> [LOCK 해제] ESP32로부터 작업 완료 신호(DONE) 접수.")
                except:
                    pass
            time.sleep(0.005)

    def set_command(self, cmd_str, request_lock=False):
        """ 명령어를 갱신하고 송신합니다. 회전/정지 등 대기가 필요한 명령만 선택적으로 락을 겁니다. """
        if self.last_command != cmd_str:
            self.last_command = cmd_str
            self.get_logger().info(f"명령 전송: {cmd_str} (Lock 설정: {request_lock})")
            if hasattr(self, 'ser'):
                self.ser.write(cmd_str.encode('utf-8'))
            if request_lock:
                self.waiting_for_esp = True

    def heartbeat_loop(self):
        """ 0.2초마다 현재 락온된 명령을 계속 밀어넣어 ESP32 세이프가드 작동을 방해합니다. """
        if hasattr(self, 'ser') and self.last_command:
            self.ser.write(self.last_command.encode('utf-8'))
            
        # AVOID 2단계 직진(G) 우회 구간은 시간(3.0초) 기반 자동 탈출 유지
        if self.state == "AVOID" and self.avoid_substate == 3 and self.waiting_for_esp:
            if time.time() - self.avoid_drive_start > 3.0: 
                self.waiting_for_esp = False
                self.get_logger().info(">> 장애물 측면 회피 직진 완료. 다음 회전 단계 진입.")

        # 상태 시각화 퍼블리시
        state_msg = String()
        display_state = f"{self.state} Step{self.avoid_substate}" if self.state == "AVOID" else self.state
        if self.waiting_for_esp:
            display_state += " [WAIT]"
        state_msg.data = f"{display_state}|{self.last_command}"
        self.state_pub.publish(state_msg)

    def vision_callback(self, msg):
        """ 비전 센서 데이터를 받아서 라인 추적 및 FSM을 구동합니다. """
        if self.waiting_for_esp:
            return  # 💡 회전 기동 중(LOCK)일 때는 비전 명령 분석을 일시 스킵하여 꼬임을 예방합니다.

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

        # [상태 전환]
        if self.state == "CRUISE" and not crosswalk_detected:
            if red_line_detected and not self.red_line_latched:
                self.state = "STOP_TIMER"
                self.stop_start_time = current_time
                self.red_line_latched = True
            elif obstacle_in_front and not self.obstacle_latched:
                self.state = "AVOID"
                self.avoid_substate = 1
                self.avoid_direction = avoid_dir

        # [명령 실행 및 락 선택 제어]
        if crosswalk_detected and self.state != "AVOID":
            self.set_command("<g,108>", request_lock=False) # 직진은 락을 걸지 않음

        elif self.state == "STOP_TIMER":
            if current_time - self.stop_start_time > 1.5:
                self.state = "CRUISE"
            else:
                self.set_command("<s>", request_lock=True) # 정지는 안전을 위해 락 처리

        elif self.state == "AVOID":
            if self.avoid_substate == 1:
                self.set_command(f"<t,{45 * self.avoid_direction}>", request_lock=True) # 회전 완료까지 대기(락)
                self.avoid_substate = 2
            elif self.avoid_substate == 2:
                self.set_command("<g,108>", request_lock=True) # 우회 직진용 타이머 구동을 위한 임시 락
                self.avoid_drive_start = current_time
                self.avoid_substate = 3
            elif self.avoid_substate == 3:
                self.set_command(f"<t,{-45 * self.avoid_direction}>", request_lock=True) # 복귀 회전 완료까지 대기(락)
                self.avoid_substate = 4
            elif self.avoid_substate == 4:
                self.state = "CRUISE"
                self.obstacle_latched = True
                self.set_command("<g,108>", request_lock=False)

        elif self.state == "CRUISE":
            # 💡 라인 추적(직진/미세 조향) 구간은 락을 절대 걸지 않고 실시간 속도/방향 업데이트 허용!
            if abs(error) < 10:
                self.set_command("<g,108>", request_lock=False)
            else:
                turn_deg = int(error * -0.1)
                self.set_command(f"<t,{turn_deg}>" if abs(turn_deg) >= 2 else "<g,108>", request_lock=False)

def main(args=None):
    rclpy.init(args=args)
    node = ControlHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser'):
            node.ser.write(b"<s>")
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()