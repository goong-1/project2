#!/usr/bin/env python3
"""
[라즈베리파이4 전용] bridge_node.py

원본 대비 변경점:
  - 라이다 / 카메라 직접 구독 제거
  - 자체 판단 로직 (장애물, 신호 우선순위) 전부 제거
  - PC의 brain_node에서 내려오는 /esp_command 만 받아서 UART 전달
  - ESP32에서 올라오는 응답을 /esp_status 로 발행 (brain_node가 사용)

역할:
  ROS2 토픽  ↔  UART  의 단순 게이트웨이
"""

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial


class EspBridgeNode(Node):
    def __init__(self):
        super().__init__('esp_bridge_node')

        # ── UART 포트 ──
        self.SERIAL_PORT = '/dev/serial0'
        self.BAUDRATE    = 115200

        try:
            self.ser = serial.Serial(self.SERIAL_PORT, self.BAUDRATE, timeout=0.1)
            self.ser.flush()
            self.get_logger().info(f'[UART] {self.SERIAL_PORT} @ {self.BAUDRATE} 연결 완료')
        except Exception as e:
            self.get_logger().error(f'[UART] 시리얼 포트 열기 실패: {e}')
            self.ser = None

        # ── 구독: PC의 brain_node로부터 명령 수신 ──
        self.cmd_sub = self.create_subscription(
            String, '/esp_command', self._cmd_callback, 10
        )

        # ── 발행: ESP32 응답을 PC로 전달 ──
        self.status_pub = self.create_publisher(String, '/esp_status', 10)

        # ── ESP32 응답 수신 스레드 ──
        self._stop_flag = False
        if self.ser:
            self._rx_thread = threading.Thread(target=self._uart_rx_loop, daemon=True)
            self._rx_thread.start()

        self.get_logger().info('==========================================')
        self.get_logger().info(' Bridge Node 시작                          ')
        self.get_logger().info('   /esp_command → UART → ESP32            ')
        self.get_logger().info('   ESP32 → UART → /esp_status             ')
        self.get_logger().info('==========================================')

    def _cmd_callback(self, msg):
        """brain_node 명령을 UART로 즉시 전달 (지연 최소화)"""
        if not self.ser:
            return

        # brain_node 가 보내는 명령 포맷: "G200", "S", "T45" 등
        cmd = msg.data.strip()
        if not cmd:
            return

        # ESP32 가 기대하는 패킷 포맷으로 감싸기
        # 원본 코드 호환:
        #   G200  → <g,200>
        #   T45   → <t,45>
        #   S     → <s>
        packet = self._wrap_packet(cmd)
        if packet is None:
            self.get_logger().warn(f'알 수 없는 명령: {cmd}')
            return

        try:
            self.ser.write(packet.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'[UART] 송신 실패: {e}')

    @staticmethod
    def _wrap_packet(cmd: str) -> str | None:
        """brain_node 단순 명령을 ESP32 패킷 포맷으로 변환"""
        if cmd == 'S':
            return "<s>\n"
        if cmd.startswith('G'):
            try:
                pwm = int(cmd[1:])
                return f"<g,{pwm}>\n"
            except ValueError:
                return None
        if cmd.startswith('T'):
            try:
                deg = int(cmd[1:])
                return f"<t,{deg}>\n"
            except ValueError:
                return None
        return None

    def _uart_rx_loop(self):
        """ESP32에서 올라오는 응답 (예: DONE) 을 /esp_status 로 발행"""
        buf = b''
        while not self._stop_flag and rclpy.ok():
            try:
                data = self.ser.read(64)
                if not data:
                    continue
                buf += data
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    text = line.decode('utf-8', errors='ignore').strip()
                    if text:
                        msg = String()
                        msg.data = text
                        self.status_pub.publish(msg)
            except Exception as e:
                self.get_logger().warn(f'[UART] 수신 에러: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = EspBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_flag = True
        if node.ser:
            try:
                # 안전 정지
                node.ser.write(b"<s>\n")
                node.ser.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
