#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
import serial

class SensorBypassGateway(Node):
    def __init__(self):
        super().__init__('sensor_bypass_gateway')
        
        self.USE_UART_COMMUNICATION = True
        self.front_min_distance = 0.4
        self.angle_range_deg = 30.0

        # 하드웨어 시리얼 포트 경로 명시
        self.SERIAL_PORT = '/dev/ttyAMA0'

        if self.USE_UART_COMMUNICATION:
            try:
                # pySerial 객체 생성 시 내부적으로 보레이트와 raw 모드 설정을 자동으로 수행합니다.
                self.ser = serial.Serial(
                    port=self.SERIAL_PORT,
                    baudrate=115200,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.02
                )
                self.ser.flush()
                self.get_logger().info(f'[UART] 포트({self.SERIAL_PORT}) 자동 최적화 및 오픈 성공!')
            except Exception as e:
                self.get_logger().error(f'[UART] 시리얼 에러 ({self.SERIAL_PORT}): {e}')
                self.get_logger().warn('★ 만약 Permission denied 에러가 발생하면 터미널에 "sudo usermod -aG dialout $USER"를 실행하고 꼭 재부팅하세요!')
                self.USE_UART_COMMUNICATION = False

        # 통신 인터페이스 구성
        self.cmd_sub = self.create_subscription(String, '/esp_command', self.cmd_callback, 10)
        self.status_pub = self.create_publisher(String, '/esp_status', 10)

        # 센서 및 비전 토픽 구독
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.camera_sub = self.create_subscription(String, 'traffic_sign_topic', self.camera_callback, 10)
        
        self.traffic_light_status = "UNKNOWN"
        self.obstacle_detected = False

        # ESP32의 무선 피드백 상태 파싱을 위한 고속 타이머 (50Hz)
        if self.USE_UART_COMMUNICATION:
            self.read_timer = self.create_timer(0.02, self.read_serial_feedback)

    def camera_callback(self, msg):
        self.traffic_light_status = msg.data

    def scan_callback(self, msg):
        self.obstacle_detected = False
        angle_range_rad = math.radians(self.angle_range_deg)
        
        for i, distance in enumerate(msg.ranges):
            if distance < msg.range_min or distance > msg.range_max:
                continue
            current_angle = msg.angle_min + (i * msg.angle_increment)
            if -angle_range_rad <= current_angle <= angle_range_rad:
                if distance < self.front_min_distance:
                    self.obstacle_detected = True
                    break

    def cmd_callback(self, msg):
        # 최우선 순위 하드웨어 세이프가드 필터링 (YOLO 빨간불 또는 라이다 장애물 발견 시 즉시 정지)
        if self.traffic_light_status == "STOP" or self.obstacle_detected:
            self.send_uart_packet("<s>\n")
            return

        raw_cmd = msg.data
        uart_packet = ""
        
        # Brain 노드의 속도 기반 제어 명령을 ESP32 프로토콜로 번역
        if raw_cmd.startswith("G"):
            uart_packet = f"<g,{raw_cmd[1:]}>\n"
        elif raw_cmd.startswith("T"):
            uart_packet = f"<t,{raw_cmd[1:]}>\n"
        elif raw_cmd == "S":
            uart_packet = "<x>\n"
            
        if uart_packet:
            self.send_uart_packet(uart_packet)

    def send_uart_packet(self, packet_str):
        if self.USE_UART_COMMUNICATION:
            try:
                self.ser.write(packet_str.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f'[UART] 송신 에러: {e}')

    def read_serial_feedback(self):
        """ ESP32의 동작 도달 신호([도달]) 및 정지 확답(MODE: IDLE)을 분석하여 Brain 노드로 피드백 발송 """
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    # 회전 기동이 끝났거나 정지 상태 진입이 확인되면 DONE 신호 발행
                    if "[도달]" in line or "MODE: IDLE" in line:
                        status_msg = String()
                        status_msg.data = "DONE"
                        self.status_pub.publish(status_msg)
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = SensorBypassGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.USE_UART_COMMUNICATION:
            try:
                # 노드 종료 시 차량이 폭주하지 않도록 세이프가드 정지 패킷 송신 후 자원 반납
                node.ser.write(b"<x>\n")
                node.ser.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()