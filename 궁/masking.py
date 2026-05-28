import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge

class ImageDisplayNode(Node):
    def __init__(self):
        super().__init__('image_display_node')

        # 카메라 설정
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        # 타이머 (약 30fps)
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info("Image Display Node Started. Press 'q' to exit.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # 1. 전처리 및 마스킹
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask_yellow = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([40, 255, 255]))
        mask_black = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 50]))
        mask_gray = cv2.inRange(hsv, np.array([0, 0, 50]), np.array([180, 50, 180]))
        mask_red = cv2.bitwise_or(
            cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255])),
            cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        )

        # 2. 결과 화면 출력
        cv2.imshow('Original', frame)
        cv2.imshow('Yellow Mask', mask_yellow)
        cv2.imshow('Black Mask', mask_black)
        cv2.imshow('Gray Mask', mask_gray)
        cv2.imshow('Red Mask', mask_red)

        # 'q' 키 입력 시 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ImageDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
