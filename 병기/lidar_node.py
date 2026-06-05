#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray 
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math

class LidarFilterSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_filter_subscriber')
        
        # [수정 불가 제어 핵심 임계값 변수]
        self.front_min_distance = 0.70   # 정면 장애물 감지 한계 기준 거리 (0.7m)
        self.angle_range_deg = 15.0     # 정면 기준 좌우 15도 (총 30도 부채꼴 영역)

        # 원본 라이다 데이터 구독
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        
        custom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.filtered_scan_pub = self.create_publisher(
            LaserScan,
            '/filtered_scan',
            custom_qos
        )
        
        self.wifi_status_pub = self.create_publisher(
            Float32MultiArray,
            '/obstacle_status', 
            custom_qos
        )
        
        self.obstacle_detected = False
        self.closest_obstacle_distance = 0.0
        self.closest_obstacle_angle = 0.0  
        
        self.get_logger().info('D200 경량형 이벤트 기반 토픽 발행 컴포넌트 로드 완료.')

    def scan_callback(self, msg):
        """ 0~360도 전방위 라이다 데이터를 정면 영역으로 정밀 필터링 """
        self.obstacle_detected = False
        min_detected_dist = float('inf') 
        target_angle_deg = 0.0  
        
        # 임계 각도 라디안 변환
        left_limit_rad = math.radians(self.angle_range_deg)            
        right_limit_rad = math.radians(360.0 - self.angle_range_deg)  
        
        # Rviz2 전송용 필터링 데이터 구조 초기화
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.ranges = [float('inf')] * len(msg.ranges)
        
        for i, distance in enumerate(msg.ranges):
            if distance < msg.range_min or distance > msg.range_max or math.isinf(distance) or math.isnan(distance):
                continue
                
            current_angle = msg.angle_min + (i * msg.angle_increment)
            
            if current_angle < 0:
                current_angle += 2 * math.pi
            
            if (current_angle <= left_limit_rad) or (current_angle >= right_limit_rad):
                filtered_msg.ranges[i] = distance
                
                if distance < self.front_min_distance:
                    self.obstacle_detected = True
                    if distance < min_detected_dist:
                        min_detected_dist = distance
                        
                        deg_val = math.degrees(current_angle)
                        
                        if deg_val > 180.0:
                            target_angle_deg = deg_val - 360.0
                        else:
                            target_angle_deg = deg_val
                        
        if self.obstacle_detected:
            self.closest_obstacle_distance = min_detected_dist
            self.closest_obstacle_angle = target_angle_deg
        else:
            self.closest_obstacle_distance = 0.0
            self.closest_obstacle_angle = 0.0
            
        self.filtered_scan_pub.publish(filtered_msg)
        self.process_obstacle_status()

    def process_obstacle_status(self):
        """ [수정] 장애물 발견 시에만 상태값 1개 발행, 평소에는 무통신(대역폭 절약) """
        if self.obstacle_detected:
            # 콘솔 내부 로그에는 모니터링을 위해 상세 실측 데이터를 계속 출력합니다.
            self.get_logger().warn(
                f'![위험] 정면 장애물 발견: {self.closest_obstacle_distance:.3f}m | '
                f'방향: {self.closest_obstacle_angle:+.1f}°'
            )
            
            # 토픽에는 거리, 각도를 빼고 오직 위험 유무 상태(1.0)만 패킹합니다.
            status_msg = Float32MultiArray()
            status_msg.data = [1.0]
            
            # 위험할 때만 발행 실행
            self.wifi_status_pub.publish(status_msg)
            
        else:
            # 안전할 때는 콘솔 로그만 찍고, 토픽 발행(Publish) 자체를 건너뜁니다.
            self.get_logger().info('전방 안전 확보됨.')

def main(args=None):
    rclpy.init(args=args)
    node = LidarFilterSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node.shutdown()

if __name__ == '__main__':
    main()