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

        # 2. UART 통신 포트 오픈 (수정 완료: ttyAMA0, 개행문자 없음)
        try:
            self.ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=0.05)
            self.ser.flush()
            self.get_logger().info('[UART] ESP32 연결용 시리얼 포트(/dev/ttyAMA0) 오픈 완료.')
        except Exception as e:
            self.get_logger().error(f'[UART] 시리얼 포트를 열 수 없습니다: {e}')

        # 3. FSM 및 제어 변수 (원본 구조 완벽 이식)
        self.state = "CRUISE"
        self.last_command = "<s>"
        self.stop_start_time = 0.0
        self.red_line_latched = False
        self.obstacle_latched = False
        self.avoid_direction = 0
        self.avoid_substate = 0
        self.waiting_for_esp = False
        self.avoid_drive_start = 0.0  # 직진 우회용 타이머

        # 4. 0.2초 주기 하트비트 전송 타이머 (세이프가드 유지용)
        self.timer = self.create_timer(0.2, self.heartbeat_loop)
        
        # 5. 수신 전용 백그라운드 스레드 (DONE 신호 감지용)
        self.uart_thread = threading.Thread(target=self.uart_read_loop, daemon=True)
        self.uart_thread.start()

    def uart_read_loop(self):
        """ ESP32가 보내는 DONE 신호를 실시간으로 가로채어 락(Lock)을 해제합니다. """
        while rclpy.ok():
            if hasattr(self, 'ser') and self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if "DONE" in line:
                        self.waiting_for_esp = False
                        self.get_logger().info(">> ESP32 작업 완료 신호(DONE) 접수. 다음 명령 대기 해제.")
                except:
                    pass
            time.sleep(0.01)

    def set_command(self, cmd_str):
        """ 상태가 변할 때 즉시 명령을 갱신하고 송신합니다. (\n 제거된 꺾쇠 패킷 규격) """
        if self.last_command != cmd_str:
            self.last_command = cmd_str
            self.get_logger().info(f"명령 갱신 전송: {cmd_str}")
            if hasattr(self, 'ser'):
                self.ser.write(cmd_str.encode('utf-8'))

    def heartbeat_loop(self):
        """ 0.2초마다 현재 명령을 반복 송신하여 ESP32의 2초 세이프가드를 방어합니다. """
        if hasattr(self, 'ser') and self.last_command:
            self.ser.write(self.last_command.encode('utf-8'))
            
        # AVOID 2단계 직진(G) 기동은 완료 신호가 없으므로 파이썬에서 시간(3.0초)으로 끊어줍니다.
        if self.state == "AVOID" and self.avoid_substate == 3 and self.waiting_for_esp:
            if time.time() - self.avoid_drive_start > 3.0: 
                self.waiting_for_esp = False
                self.get_logger().info(">> 장애물 회피 직진 구간 통과 완료.")

        # 상태 시각화 퍼블리시
        state_msg = String()
        display_state = f"{self.state} Step{self.avoid_substate}" if self.state == "AVOID" else self.state
        if self.waiting_for_esp:
            display_state += " [WAIT]"
        state_msg.data = f"{display_state}|{self.last_command}"
        self.state_pub.publish(state_msg)

    def vision_callback(self, msg):
        """ 비전 데이터를 기반으로 FSM 상태 머신을 굴립니다. """
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

        # [상태 전환 로직]
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

        # [명령 실행 로직]
        if crosswalk_detected and self.state != "AVOID":
            self.set_command("<g,108>") # 기존 G200 대신 하드웨어에 맞는 PWM값 적용

        elif self.state == "STOP_TIMER":
            if current_time - self.stop_start_time > 1.5:
                self.state = "CRUISE"
            else:
                self.set_command("<s>")

        elif self.state == "AVOID":
            if not self.waiting_for_esp:
                if self.avoid_substate == 1:
                    self.set_command(f"<t,{45 * self.avoid_direction}>")
                    self.waiting_for_esp = True
                    self.avoid_substate = 2
                elif self.avoid_substate == 2:
                    self.set_command("<g,108>")
                    self.waiting_for_esp = True
                    self.avoid_drive_start = current_time
                    self.avoid_substate = 3
                elif self.avoid_substate == 3:
                    self.set_command(f"<t,{-45 * self.avoid_direction}>")
                    self.waiting_for_esp = True
                    self.avoid_substate = 4
                elif self.avoid_substate == 4:
                    self.state = "CRUISE"
                    self.obstacle_latched = True
                    self.set_command("<g,108>")

        elif self.state == "CRUISE":
            if abs(error) < 10:
                self.set_command("<g,108>")
            else:
                turn_deg = int(error * -0.1)
                # 오차가 작으면 부드럽게 가도록 직진, 크면 회전
                self.set_command(f"<t,{turn_deg}>" if abs(turn_deg) >= 2 else "<g,108>")

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
            node.get_logger().info('UART 포트 반납 및 시스템 정지 완료')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()