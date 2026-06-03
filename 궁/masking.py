#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageDisplayNode(Node):
    def __init__(self):
        super().__init__('image_display_node')

        # ROS 이미지를 OpenCV 이미지로 변환하기 위한 CvBridge 초기화
        self.bridge = CvBridge()

        # 웹캠 대신 압축 이미지 토픽 구독 (qos_profile_sensor_data 사용)
        self.image_sub = self.create_subscription(
            CompressedImage, 
            'image_raw/compressed', 
            self.image_callback, 
            qos_profile_sensor_data
        )

        self.window_name = "HSV Tuning"
        cv2.namedWindow(self.window_name)

        cv2.createTrackbar("H_LOW_color_start", self.window_name, 20, 180, lambda x: None)
        cv2.createTrackbar("H_HIGH_color_end", self.window_name, 40, 180, lambda x: None)

        cv2.createTrackbar("S_LOW_vivid_min", self.window_name, 100, 255, lambda x: None)
        cv2.createTrackbar("S_HIGH_vivid_max", self.window_name, 255, 255, lambda x: None)

        cv2.createTrackbar("V_LOW_bright_min", self.window_name, 100, 255, lambda x: None)
        cv2.createTrackbar("V_HIGH_bright_max", self.window_name, 255, 255, lambda x: None)
        
        self.get_logger().info("HSV Tuning Started. Press 's' to save values, 'q' to quit.")

    # 타이머 대신 이미지가 들어올 때마다 콜백 함수가 실행되도록 변경
    def image_callback(self, msg):
        try:
            # CompressedImage 메시지를 OpenCV BGR 이미지로 변환
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        frame = cv2.resize(frame, (320, 240))

        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        h_min = cv2.getTrackbarPos("H_LOW_color_start", self.window_name)
        h_max = cv2.getTrackbarPos("H_HIGH_color_end", self.window_name)

        s_min = cv2.getTrackbarPos("S_LOW_vivid_min", self.window_name)
        s_max = cv2.getTrackbarPos("S_HIGH_vivid_max", self.window_name)

        v_min = cv2.getTrackbarPos("V_LOW_bright_min", self.window_name)
        v_max = cv2.getTrackbarPos("V_HIGH_bright_max", self.window_name)

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])

        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # mask는 흑백이라 BGR로 변환해야 원본/결과와 붙일 수 있음
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        combined = np.hstack((frame, mask_bgr, result))

        cv2.imshow(self.window_name, combined)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            print()
            print("========== [현재 HSV 범위] ==========")
            print(f"lower = np.array([{h_min}, {s_min}, {v_min}])")
            print(f"upper = np.array([{h_max}, {s_max}, {v_max}])")
            print("=====================================")
            print()

        elif key == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ImageDisplayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
