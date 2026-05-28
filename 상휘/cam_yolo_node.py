#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # 최종 결과를 메인 노드로 넘겨줄 메시지 타입
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
import cv2
import json
from pathlib import Path

# 패키지에 포함되어 있는 비전 감지 알고리즘 라이브러리 로드
from p2_pkg.sign_detector import TrafficSignDetector

class CameraParserNode(Node):
    def __init__(self):
        super().__init__('camera_parser_node')
        self.br = CvBridge()

        # [디버깅 디스플레이 옵션]
        # WSL2나 가상환경 환경에서 윈도우 창(imshow)을 띄우려면 True로 설정하세요.
        # 터미널 로그로만 확인하고 싶다면 False로 두시면 됩니다.
        self.SHOW_DISPLAY = True 

        # 가중치 모델 파일(best_n_model.pt)의 절대 경로 빌드
        BASE_DIR = Path(__file__).resolve().parent
        MODEL_PATH = BASE_DIR / "best_n_model_ncnn_model"

        # sign_detector.py 라이브러리를 이용해 신뢰도 70% 기준으로 디텍터 초기화
        self.detector = TrafficSignDetector(
            model_path=str(MODEL_PATH),
            conf_threshold=0.70
        )

        # 이미지 수신용 기본 큐 사이즈 설정
        self.image_sub = self.create_subscription(
            Image,
            'video_frames',
            self.image_callback,
            10
        )

        # 메인 노드로 텍스트 정답을 쏴줄 발행자 생성
        self.result_pub = self.create_publisher(String, 'traffic_sign_topic', 10)

        self.last_print_time = self.get_clock().now()
        self.get_logger().info('==================================================')
        self.get_logger().info(' YOLO 기반 비전 분석 파서 노드가 정상 기동되었습니다. ')
        self.get_logger().info('==================================================')
         # =========================
        # ROI / one-shot publish 설정
        # =========================
        self.valid_actions = {"STOP", "GO", "SPEED_LIMIT", "TURN_LEFT"}

        # 화면 비율 기준 ROI: x1, y1, x2, y2
        # 예: 중앙 60%, 세로 20%~85% 영역만 인정
        self.roi_ratio = (0.10, 0.01, 0.90, 0.50)

        # 너무 멀리 있는 작은 박스는 무시
        # 320x240 기준 0.01이면 전체 화면의 1% 이상
        self.min_area_ratio = 0.010

        # 같은 액션을 한 번만 보내기 위한 상태값
        self.last_sent_action = None

        # 감지가 몇 프레임 사라지면 다시 같은 액션을 보낼 수 있게 할지
        self.no_detection_count = 0
        self.reset_after_missing_frames = 10
        
    def image_callback(self, msg):
        try:
            # 수신된 ROS 2 이미지를 OpenCV Matrix 데이터 포맷으로 변환
            frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'이미지 디코딩 변환 실패: {str(e)}')
            return

                # 1. 전체 객체 탐지
        all_detections = self.detector.detect_all(frame)

        # 2. ROI 안에 있고 충분히 가까운 객체만 사용
        detections = self.filter_detections_by_roi(all_detections, frame)

        # 3. ROI 안에 유효 객체가 없으면 아무것도 발행하지 않음
        if not detections:
            self.no_detection_count += 1

            # 일정 시간 안 보이면 같은 액션을 다시 보낼 수 있게 reset
            if self.no_detection_count >= self.reset_after_missing_frames:
                self.last_sent_action = None

            if self.SHOW_DISPLAY:
                debug_frame = self.detector.draw_all(frame, detections)
                x1, y1, x2, y2 = self.get_roi_box(frame)
                cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(
                    debug_frame,
                    "NO VALID DETECTION - NO PUBLISH",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2
                )
                cv2.imshow("Traffic Sign Detection (ROS2)", debug_frame)
                cv2.waitKey(1)

            return

        self.no_detection_count = 0

        # 4. 우선순위대로 대표 액션 1개 선택
        main_action = self.detector.get_main_action(detections)
        action_hint = main_action.get("action_hint", None)

        if action_hint not in self.valid_actions:
            return

        # 5. 같은 액션이 계속 보이면 딱 한 번만 발행
        if action_hint == self.last_sent_action:
            if self.SHOW_DISPLAY:
                debug_frame = self.detector.draw_all(frame, detections)
                x1, y1, x2, y2 = self.get_roi_box(frame)
                cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(
                    debug_frame,
                    f"ALREADY SENT: {action_hint}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2
                )
                cv2.imshow("Traffic Sign Detection (ROS2)", debug_frame)
                cv2.waitKey(1)

            return

        # 6. 처음 본 액션만 발행
        string_msg = String()
        string_msg.data = action_hint
        self.result_pub.publish(string_msg)
        self.last_sent_action = action_hint
        
        # 5. 화면 디스플레이 플래그가 True일 때 윈도우 창 시각화
        if self.SHOW_DISPLAY:
            debug_frame = self.detector.draw_all(frame, detections)
            x1, y1, x2, y2 = self.get_roi_box(frame)
            cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(
                debug_frame,
                f"SENT: {action_hint}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2
            )
            cv2.imshow("Traffic Sign Detection (ROS2)", debug_frame)
            cv2.waitKey(1)

        # 4. 터미널 폭주 방지를 위해 0.5초 주기로 추론 결과 출력
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.last_print_time).nanoseconds / 1e9
        
        if elapsed_time > 0.5:
            self.get_logger().info(f'[YOLO AI Result] FINAL ACTION HINT -> {action_hint}')
            self.last_print_time = current_time

        # 5. 화면 디스플레이 플래그가 True일 때 윈도우 창 시각화
        # if self.SHOW_DISPLAY:
            # annotated_frame = self.detector.draw_all(frame, detections)
            
            # cv2.putText(
            #     annotated_frame,
            #     f"MAIN ACTION: {action_hint}",
            #     (20, 40),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     1.0,
            #     (0, 255, 255),
            #     2
            # )
            
            # cv2.imshow("Traffic Sign Detection (ROS2)", annotated_frame)
            # cv2.waitKey(1)
            
    def get_roi_box(self, frame):
        h, w = frame.shape[:2]
        rx1, ry1, rx2, ry2 = self.roi_ratio

        x1 = int(w * rx1)
        y1 = int(h * ry1)
        x2 = int(w * rx2)
        y2 = int(h * ry2)

        return x1, y1, x2, y2

    def filter_detections_by_roi(self, detections, frame):
        roi_x1, roi_y1, roi_x2, roi_y2 = self.get_roi_box(frame)

        filtered = []

        for d in detections:
            # STOP / GO / SPEED_LIMIT / TURN_LEFT만 사용
            if d.action_hint not in self.valid_actions:
                continue

            cx = d.center["x"]
            cy = d.center["y"]

            # 중심점이 ROI 안에 들어온 경우만 인정
            if not (roi_x1 <= cx <= roi_x2 and roi_y1 <= cy <= roi_y2):
                continue

            # 너무 작은 객체는 아직 멀다고 보고 무시, 어느거리에서 반응할지
            if d.area_ratio < self.min_area_ratio:
                continue

            filtered.append(d)

        return filtered
    
def main(args=None):
    rclpy.init(args=args)
    node = CameraParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()