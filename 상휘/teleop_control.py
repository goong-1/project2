#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

msg_text = """
========================================
    🚗 로봇 수동 조종 모드 시작! 🚗
========================================
이동 (기본 방향키 형태):
        w
   a    s    d

  w : 전진 (G 명령)
  a : 좌회전 (T- 명령)
  d : 우회전 (T+ 명령)
  s : 정지 (S 명령)

설정 변경:
  i / k : 전진 속도(Speed) 5 증가 / 감소
  j / l : 회전 각도(Angle) 5 증가 / 감소

  q 또는 Ctrl+C : 종료
========================================
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.pub = self.create_publisher(String, '/esp_command', 10)
        
        # 기본 설정값 (기존 제어 코드 기준)
        self.speed = 115
        self.angle = 45

    def send_command(self, cmd):
        msg = String()
        msg.data = cmd
        self.pub.publish(msg)
        # 터미널 화면이 지저분해지지 않도록 \r 을 사용하여 덮어쓰기 출력
        print(f"\r[발송 완료] {cmd:<6} (현재 속도: {self.speed}, 회전각: {self.angle})   ", end='')

def get_key(settings):
    """키보드 입력을 즉시 1바이트 읽어오는 함수"""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    # 기존 터미널 설정 백업
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init(args=args)
    node = KeyboardTeleop()

    print(msg_text)

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key == 'w':
                node.send_command(f"G{node.speed}")
            elif key == 'a':
                node.send_command(f"T{-node.angle}")
            elif key == 'd':
                node.send_command(f"T{node.angle}")
            elif key == 's' or key == ' ':
                node.send_command("S")
            elif key == 'i':
                node.speed = min(255, node.speed + 5)
                print(f"\r[설정 변경] 속도 증가 -> {node.speed}                  ", end='')
            elif key == 'k':
                node.speed = max(0, node.speed - 5)
                print(f"\r[설정 변경] 속도 감소 -> {node.speed}                  ", end='')
            elif key == 'j':
                node.angle = min(90, node.angle + 5)
                print(f"\r[설정 변경] 회전각 증가 -> {node.angle}                 ", end='')
            elif key == 'l':
                node.angle = max(0, node.angle - 5)
                print(f"\r[설정 변경] 회전각 감소 -> {node.angle}                 ", end='')
            elif key == 'q' or key == '\x03': # q 또는 Ctrl+C
                print("\n\n수동 조종을 종료합니다.")
                break
                
    except Exception as e:
        print(f"\n오류 발생: {e}")
    finally:
        # 노드 종료 시 로봇이 폭주하지 않도록 무조건 정지 명령 발송
        print("\n안전 정지(S) 명령 전송 중...")
        stop_msg = String()
        stop_msg.data = "S"
        node.pub.publish(stop_msg)
        
        # 터미널 설정 복구 및 노드 종료
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
