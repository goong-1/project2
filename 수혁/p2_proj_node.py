#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
import serial  # 라즈베리 파이 -> ESP32 UART 통신용

class SensorBypassGateway(Node):
    def __init__(self):
        super().__init__('sensor_bypass_gateway')
        
        # =========================================================================
        # [기능 제어 스위치 및 하드웨어 보정치]
        # =========================================================================
        self.USE_UART_COMMUNICATION = True
        self.LEFT_PWM_BALANCED = 108    # 직진(<g, 108>) 전송 시 사용할 좌측 파워 매칭값

        # [라이다 필터링 파라미터]
        self.front_min_distance = 0.4   # 정면 장애물 감지 거리 (40cm)
        self.angle_range_deg = 30.0     # 정면 기준 좌우 30도 (총 60도 부채꼴 영역)

        # =========================================================================
        # [UART 시리얼 장치 초기화] ESP32 통신 포트 개방
        # =========================================================================
        if self.USE_UART_COMMUNICATION:
            try:
                # 라즈베리 파이 기본 하드웨어 시리얼 포트(/dev/serial0), 속도 115200
                self.ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=1)
                self.ser.flush()
                self.get_logger().info('[UART] ESP32 연결용 시리얼 포트(/dev/serial0) 오픈 완료.')
            except Exception as e:
                self.get_logger().error(f'[UART] 시리얼 포트를 열 수 없습니다: {e}')
                self.USE_UART_COMMUNICATION = False

        # =========================================================================
        # [ROS 2 인터페이스] 센서 및 비전 토픽 구독 (Bypass 데이터 소스)
        # =========================================================================
        # 1. 라이다 데이터 구독
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # 2. camera_parser_node(YOLO)의 신호 상태 토픽 구독
        self.camera_sub = self.create_subscription(
            String,
            'traffic_sign_topic',
            self.camera_callback,
            10
        )
        
        # 실시간 상태 캐시 레지스터
        self.traffic_light_status = "UNKNOWN"
        self.obstacle_detected = False

        self.get_logger().info('==================================================')
        self.get_logger().info(' 센서 데이터 바이패스 전용 컨트롤 노드 가동 개시 ')
        self.get_logger().info(' 모든 주행 판단 및 모터 제어 우선순위는 ESP32에서 처리합니다. ')
        self.get_logger().info('==================================================')

    def camera_callback(self, msg):
        """ YOLO 노드가 발행한 신호 텍스트를 캐싱하고 바이패스 결정 루프 실행 """
        self.traffic_light_status = msg.data
        self.process_and_bypass()

    def scan_callback(self, msg):
        """ 라이다 영역 필터링 연산 후 바이패스 결정 루프 실행 """
        self.obstacle_detected = False
        angle_range_rad = math.radians(self.angle_range_deg)
        
        for i, distance in enumerate(msg.ranges):
            if distance < msg.range_min or distance > msg.range_max:
                continue
                
            current_angle = msg.angle_min + (i * msg.angle_increment)
            
            # 정면 지정 부채꼴 범위 안의 장애물 필터링
            if -angle_range_rad <= current_angle <= angle_range_rad:
                if distance < self.front_min_distance:
                    self.obstacle_detected = True
                    break
                    
        self.process_and_bypass()

    def process_and_bypass(self):
        """ 취합된 토픽 상태를 지정된 커스텀 패킷 규격으로 가공하여 ESP32로 단순 포워딩 """
        uart_packet = "<s>\n"  # 기본 안전 모드는 정지(<s>)로 매핑
        
        # [우선순위 1] 비전 분석 결과 빨간불(STOP) 조건 충족 시 -> 정지 패킷 발행
        if self.traffic_light_status == "STOP":
            uart_packet = "<s>\n"
            
        # [우선순위 2] 전방에 라이다 장애물 발견 시 -> 회피 기동 조건 진입 (ESP32에 타겟팅 각도 전송)
        elif self.obstacle_detected:
            # 예시: 장애물이 있으면 제자리 회전 명령 송신 (ESP32 내부 로직에 맞춰 90 또는 -90 활용 가능)
            uart_packet = "<t,-90>\n"  # 우회전 회피 패킷 
            
        # [우선순위 3] 전방 클리어 및 정상 주행 가능 상태 -> 직진 구동 패킷 발행
        else:
            uart_packet = f"<g,{self.LEFT_PWM_BALANCED}>\n"  # 직진 패킷 (<g,108>)

        # ESP32 측 하드웨어 물리 시리얼 라인으로 최종 문자열 전송
        if self.USE_UART_COMMUNICATION:
            try:
                self.ser.write(uart_packet.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f'[UART] ESP32 바이패스 송신 중 물리 에러: {e}')

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
                # 안전을 위해 시스템 종료 시 무조건 정지 패킷(<s>) 강제 전송 후 포트 반납
                node.ser.write(b"<s>\n")
                node.ser.close()
                node.get_logger().info('[UART] ESP32 시리얼 통신 포트가 안전하게 해제되었습니다.')
            except Exception as e:
                print(f"시리얼 자원 반납 실패: {e}")
                
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()