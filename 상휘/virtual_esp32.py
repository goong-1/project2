import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import time

class VirtualESP32(Node):
    def __init__(self):
        super().__init__('virtual_esp32')
        
        # 1. Pub/Sub 설정
        self.cmd_sub = self.create_subscription(String, '/esp_command', self.cmd_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # [추가] LaneFollower로 완료 상태(DONE)를 보내기 위한 Publisher
        self.status_pub = self.create_publisher(String, '/esp_status', 10)
        
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(0.01, self.control_loop)

        # 상태 변수
        self.state = 'STOP'  # STOP, DRIVE, TURN
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.target_speed = 0.0

        # PID 제어용 변수 (회전 제어용)
        self.kp_turn = 10.0
        self.ki_turn = 0.0
        self.kd_turn = 0.1
        self.prev_error = 0.0
        self.integral = 0.0
        
        # [추가] 가상 직진(G) 명령 지속 시간 체크용 변수
        self.drive_start_time = 0.0
        self.drive_done_published = False

        self.get_logger().info("Virtual ESP32 Node Started. Waiting for commands...")

    def imu_callback(self, msg):
        """ IMU의 쿼터니언 데이터를 오일러 각도(Yaw, 라디안)로 변환 """
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def publish_done(self):
        """ 상위 노드(LaneFollower)에 명령 완료 신호를 전송 """
        msg = String()
        msg.data = "DONE"
        self.status_pub.publish(msg)
        self.get_logger().info("Sent status: DONE")

    def cmd_callback(self, msg):
        """ 수신된 문자열 명령 파싱 """
        cmd_str = msg.data.strip().upper()
        if not cmd_str:
            return

        command = cmd_str[0]
        value = 0.0
        if len(cmd_str) > 1:
            try:
                value = float(cmd_str[1:])
            except ValueError:
                self.get_logger().error(f"Invalid value in command: {cmd_str}")
                return

        if command == 'S':
            self.state = 'STOP'
            self.publish_done()  # 정지 즉시 완료 신호 발행
            self.get_logger().info("Command: STOP")
            
        elif command == 'G':
            self.state = 'DRIVE'
            self.target_speed = (value / 255.0) * 1.5 
            
            # [추가] 직진 명령 타이머 리셋
            self.drive_start_time = time.time()
            self.drive_done_published = False
            
            self.get_logger().info(f"Command: GO, PWM: {value}, Speed: {self.target_speed:.2f} m/s")
            
        elif command == 'T':
            self.state = 'TURN'
            target_offset_rad = math.radians(value)
            self.target_yaw = self.normalize_angle(self.current_yaw + target_offset_rad)
            
            self.prev_error = 0.0
            self.integral = 0.0
            self.get_logger().info(f"Command: TURN, Degree: {value}, Target Yaw(rad): {self.target_yaw:.2f}")

    def normalize_angle(self, angle):
        """ 각도를 -PI ~ PI 사이로 정규화 """
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        """ 100Hz로 동작하는 메인 제어 루프 """
        twist = Twist()

        if self.state == 'STOP':
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        elif self.state == 'DRIVE':
            twist.linear.x = self.target_speed
            twist.angular.z = 0.0 
            
            # [추가] 직진(G) 명령 수행 시뮬레이션: 2초가 지나면 DONE 발행
            # CRUISE 상태일 때는 LaneFollower가 이 신호를 무시하지만,
            # AVOID 시퀀스(Step 2)에서는 이 신호를 받아야 다음 회전(Step 3)으로 넘어갑니다.
            if not self.drive_done_published and (time.time() - self.drive_start_time > 6.0):
                self.publish_done()
                self.drive_done_published = True

        elif self.state == 'TURN':
            error = self.normalize_angle(self.target_yaw - self.current_yaw)
            
            # [수정] 목표 각도에 도달 시 DONE 신호 발행
            if abs(error) < 0.026:
                self.state = 'STOP'
                self.get_logger().info("Turn completed.")
                twist.angular.z = 0.0
                self.publish_done()  # 회전 완료를 LaneFollower에 알림
            else:
                self.integral += error * 0.01
                derivative = (error - self.prev_error) / 0.01
                control_signal = (self.kp_turn * error) + (self.ki_turn * self.integral) + (self.kd_turn * derivative)
                
                twist.angular.z = max(min(control_signal, 8.0), -8.0)
                self.prev_error = error

        # Gazebo로 제어값 발행
        self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = VirtualESP32()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
