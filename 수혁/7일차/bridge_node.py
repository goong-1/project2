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

        if self.USE_UART_COMMUNICATION:
            try:
                self.ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=0.02)
                self.ser.flush()
                self.get_logger().info('[UART] ESP32 연결용 시리얼 포트 오픈 완료.')
            except Exception as e:
                self.get_logger().error(f'[UART] 시리얼 에러: {e}')
                self.USE_UART_COMMUNICATION = False

        # 통신 인터페이스 구성
        self.cmd_sub = self.create_subscription(String, '/esp_command', self.cmd_callback, 10)
        self.status_pub = self.create_publisher(String, '/esp_status', 10)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.camera_sub = self.create_subscription(String, 'traffic_sign_topic', self.camera_callback, 10)
        
        self.traffic_light_status = "UNKNOWN"
        self.obstacle_detected = False

        # 시리얼 피드백 분석용 고속 타이머(50hz)
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
        # 최우선 순위 하드웨어 세이프가드 필터링
        if self.traffic_light_status == "STOP" or self.obstacle_detected:
            self.send_uart_packet("<s>\n")
            return

        raw_cmd = msg.data
        uart_packet = ""
        
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
        """ ESP32의 동작 도달 신호 및 정지 확답(IDLE 상태 진입)을 동시 캐치 """
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    # 회전 완료 혹은 정지 명령 접수 후 IDLE 상태 피드백 확인 시 DONE 발행
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
                node.ser.write(b"<x>\n")
                node.ser.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()