import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')

        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

        self.lane_width = 350
        self.obstacle_dist = 999.0
        self.state = "CRUISE"
        self.avoid_direction = 0

        self.safe_distance = 1.2
        self.cruise_speed = 0.5
        self.avoid_speed = 0.4

    def scan_callback(self, msg):
        front_ranges = [msg.ranges[i] for i in range(len(msg.ranges))
                        if -20 * (math.pi/180) <= (msg.angle_min + i * msg.angle_increment) <= 20 * (math.pi/180)]
        self.obstacle_dist = min(front_ranges) if front_ranges else 999.0

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        height, width, _ = cv_image.shape
        roi = cv_image[int(height/2):height, 0:width]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 마스킹 생성
        mask_yellow = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([40, 255, 255]))
        mask_black = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 50]))
        combined_mask = cv2.bitwise_or(mask_yellow, mask_black)

        # 차선 중심 검출
        left_mask = combined_mask[:, :width//2]
        right_mask = combined_mask[:, width//2:]
        cx_left = int(cv2.moments(left_mask)['m10'] / cv2.moments(left_mask)['m00']) if cv2.moments(left_mask)['m00'] > 0 else -1
        cx_right = int(cv2.moments(right_mask)['m10'] / cv2.moments(right_mask)['m00']) + (width//2) if cv2.moments(right_mask)['m00'] > 0 else -1

        if cx_left != -1 and cx_right != -1: self.lane_width = cx_right - cx_left

        # 상태 머신
        if self.state == "CRUISE" and self.obstacle_dist < self.safe_distance:
            self.state = "AVOID"
            left_yellow = cv2.countNonZero(mask_yellow[:, :width//2])
            right_yellow = cv2.countNonZero(mask_yellow[:, width//2:])
            self.avoid_direction = 1 if left_yellow > right_yellow else -1
        elif self.state == "AVOID" and self.obstacle_dist > self.safe_distance + 0.5:
            self.state = "CRUISE"

        # 조향 목표 계산
        target_x = width // 2
        if self.state == "CRUISE":
            if cx_left != -1 and cx_right != -1: target_x = (cx_left + cx_right) // 2
            elif cx_left != -1: target_x = cx_left + (self.lane_width // 2)
            elif cx_right != -1: target_x = cx_right - (self.lane_width // 2)
        else:
            target_x = (width // 2) + (self.avoid_direction * int(self.lane_width * 0.8))

        # 제어 명령 발행
        twist = Twist()
        twist.linear.x = self.cruise_speed if self.state == "CRUISE" else self.avoid_speed
        twist.angular.z = float((width / 2) - target_x) * 0.007
        self.publisher.publish(twist)

        # [디버깅] 마스킹된 이미지에 시각화
        debug_img = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
        if cx_left != -1: cv2.circle(debug_img, (cx_left, int(height/4)), 10, (0, 255, 0), -1)
        if cx_right != -1: cv2.circle(debug_img, (cx_right, int(height/4)), 10, (0, 255, 0), -1)
        cv2.circle(debug_img, (target_x, int(height/4)), 15, (0, 0, 255), -1)

        cv2.putText(debug_img, f"State: {self.state}", (10, 30), 1, 1, (255, 255, 255), 2)
        cv2.imshow("Mask Debug View", debug_img)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):  # 'q' 키를 누르면 비상 정지
            self.get_logger().warn("Emergency Stop triggered by User!")
            stop_twist = Twist() # 모든 속도를 0으로 설정
            self.publisher.publish(stop_twist)

            # 노드 종료 및 정리
            self.destroy_node()
            rclpy.shutdown()
def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
