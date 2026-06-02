import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge_node')
        
        # [기존 원본 코드 환경 완벽 반영] 시리얼 포트 설정 및 자동 최적화 오픈
        self.SERIAL_PORT = '/dev/ttyAMA0'
        
        try:
            # 원본 bridge_node (2).py에 작성된 세부 옵션을 그대로 적용합니다.
            self.ser = serial.Serial(
                port=self.SERIAL_PORT,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.02
            )
            self.ser.flush()
            self.get_logger().info(f"[UART] 포트({self.SERIAL_PORT}) 자동 최적화 및 오픈 성공!")
        except Exception as e:
            self.get_logger().error(f"[UART] 시리얼 에러 ({self.SERIAL_PORT}): {e}")
            self.ser = None
            
        # Brain 노드(ControlHandler)에서 오는 제어 명령 수신
        self.subscription = self.create_subscription(
            String,
            '/esp_command',
            self.cmd_callback,
            10)
            
        # ESP32에서 올라오는 상태/피드백을 Brain 노드로 송신
        self.publisher_ = self.create_publisher(String, '/esp_status', 10)
        
        # 50ms 주기로 시리얼 읽기
        self.timer = self.create_timer(0.05, self.read_serial)
        
        self.get_logger().info("Bridge Node (순수 통신 중계 모드) 실행됨.")

    def cmd_callback(self, msg):
        raw_cmd = msg.data.strip()
        uart_packet = ""
        
        # ESP32 프로토콜 변환: <g,속도>, <t,각도>, <s>, <x>
        if raw_cmd.startswith("G"):
            uart_packet = f"<g,{raw_cmd[1:]}>\n"
        elif raw_cmd.startswith("T"):
            uart_packet = f"<t,{raw_cmd[1:]}>\n"
        elif raw_cmd == "S":
            uart_packet = "<s>\n"
        elif raw_cmd == "X":
            uart_packet = "<x>\n"
            
        if uart_packet and self.ser is not None:
            try:
                self.ser.write(uart_packet.encode('utf-8'))
                # ★ [추가 반영] 브레인 노드에서 명령이 전해지면 터미널 창에 실시간으로 로그를 출력합니다.
                self.get_logger().info(f"[Brain -> Bridge] 수신: {raw_cmd} ➔ [UART 송신]: {uart_packet.strip()}")
            except Exception as e:
                self.get_logger().error(f'[UART] 송신 에러: {e}')

    def read_serial(self):
        if self.ser is not None and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    # ESP32의 텔레메트리나 DONE 신호를 판단 없이 그대로 ROS 토픽으로 올려보냄
                    msg = String()
                    msg.data = line
                    self.publisher_.publish(msg)
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None:
            try:
                node.ser.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
