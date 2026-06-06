#!/usr/bin/env python3
"""
verify_on_video.py
──────────────────────────────────────────────────────────────────
cam_yolo_node.py 의 추적 로직(라벨/클래스별 임계값 + ROI 필터 + 액션 판단)을
녹화 영상에 그대로 적용해서 결과 영상(.mp4)으로 저장한다.

감지된 객체의 label / confidence / area_ratio 를 콘솔에 로그로 출력한다.

실행:
  python3 verify_on_video.py 입력영상.mp4
  python3 verify_on_video.py 입력영상.mp4 결과영상.mp4
──────────────────────────────────────────────────────────────────
"""

import sys
import time
from pathlib import Path

import cv2

try:
    from p2_pkg.cam_yolo_detector import TrafficSignDetector
except ImportError:
    from cam_yolo_detector import TrafficSignDetector


# 필터 전 모든 탐지를 보고 싶으면 True (디버깅용, 로그 많음)
SHOW_ALL_DETECTIONS = False


class VideoVerifier:
    def __init__(self, model_path):
        self.detector = TrafficSignDetector(
            model_path=str(model_path),
            conf_threshold=0.25
        )

        self.valid_actions = {"STOP", "GO", "SPEED_LIMIT", "TURN_LEFT"}
        self.class_thresholds = {
            "STOP": 0.70, "GO": 0.70, "TURN_LEFT": 0.70, "SPEED_LIMIT": 0.70,
        }
        self.default_conf_threshold = 0.50

        # ROI: (rx1, ry1, rx2, ry2) = (왼쪽x, 위쪽y, 오른쪽x, 아래쪽y)
        self.roi_ratio = (0.000, 0.000, 0.999, 0.999)

        # 라벨별 최소 면적 비율 (클수록 더 가까이 와야 인식)
        self.min_area_by_label = {
            "redlight":                 0.100,
            "traffic_light_red":        0.001,
            "greenlight":               0.001,
            "traffic_left_light_green": 0.001,
            "stop_sign":                0.001,
            "limit_sign":               0.001,
        }
        self.default_min_area = 0.001

        self.last_sent_action = None
        self.no_detection_count = 0
        self.reset_after_missing_frames = 10

    def get_roi_box(self, frame):
        h, w = frame.shape[:2]
        rx1, ry1, rx2, ry2 = self.roi_ratio
        return int(w*rx1), int(h*ry1), int(w*rx2), int(h*ry2)

    def get_detection_confidence(self, d):
        for attr in ["confidence", "conf", "score"]:
            if hasattr(d, attr):
                return float(getattr(d, attr))
        if isinstance(d, dict):
            for k in ["confidence", "conf", "score"]:
                if k in d:
                    return float(d[k])
        return 1.0

    def filter_detections_by_roi(self, detections, frame):
        rx1, ry1, rx2, ry2 = self.get_roi_box(frame)
        out = []
        for d in detections:
            if d.action_hint not in self.valid_actions:
                continue
            conf = self.get_detection_confidence(d)
            thr  = self.class_thresholds.get(d.action_hint, self.default_conf_threshold)
            if conf < thr:
                continue
            cx, cy = d.center["x"], d.center["y"]
            if not (rx1 <= cx <= rx2 and ry1 <= cy <= ry2):
                continue
            min_area = self.min_area_by_label.get(d.label, self.default_min_area)
            if d.area_ratio < min_area:
                continue
            out.append(d)
        return out

    def make_debug_frame(self, frame, detections, status_text):
        debug_frame = frame.copy()
        debug_frame = self.detector.draw_all(debug_frame, detections)
        x1, y1, x2, y2 = self.get_roi_box(frame)
        cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(debug_frame, status_text, (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        return debug_frame

    def process_frame(self, frame):
        all_detections = self.detector.detect_all(frame)

        if SHOW_ALL_DETECTIONS:
            for d in all_detections:
                print(f"      [전체] {d.label} conf={self.get_detection_confidence(d):.2f} "
                      f"area={d.area_ratio:.5f} center={d.center}")

        detections = self.filter_detections_by_roi(all_detections, frame)

        det_info = [
            (d.label, self.get_detection_confidence(d), d.area_ratio)
            for d in detections
        ]

        if not detections:
            self.no_detection_count += 1
            if self.no_detection_count >= self.reset_after_missing_frames:
                self.last_sent_action = None
            return self.make_debug_frame(frame, detections, "NO VALID DETECTION"), None, det_info

        self.no_detection_count = 0

        main_action = self.detector.get_main_action(detections)
        action_hint = main_action.get("action_hint", None)

        if action_hint not in self.valid_actions:
            return self.make_debug_frame(frame, detections, "INVALID ACTION"), None, det_info

        if action_hint == self.last_sent_action:
            return self.make_debug_frame(frame, detections, f"ALREADY SENT: {action_hint}"), None, det_info

        self.last_sent_action = action_hint
        return self.make_debug_frame(frame, detections, f"SENT: {action_hint}"), action_hint, det_info


def main():
    if len(sys.argv) < 2:
        print("사용법: python3 verify_on_video.py 입력영상.mp4 [결과영상.mp4]")
        sys.exit(1)

    in_path = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) >= 3 else "verify_result1.mp4"

    model_path = Path(__file__).resolve().parent / "cam_yolo_7-1.pt"
    if not model_path.exists():
        print(f"⚠ 모델을 찾을 수 없음: {model_path}")
        print("  → 스크립트와 같은 폴더에 모델을 두거나, 위 model_path 줄을 직접 수정하세요.")
        sys.exit(1)

    cap = cv2.VideoCapture(in_path)
    if not cap.isOpened():
        print(f"⚠ 영상을 열 수 없음: {in_path}")
        sys.exit(1)

    fps   = cap.get(cv2.CAP_PROP_FPS) or 20.0
    w     = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h     = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(out_path, fourcc, fps, (w, h))

    verifier = VideoVerifier(model_path)

    print(f"입력: {in_path}  ({w}x{h}, {fps:.1f}fps, {total}프레임)")
    print(f"출력: {out_path}")
    print("처리 중...")

    idx = 0
    action_changes = []
    t_start = time.perf_counter()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        debug_frame, new_action, det_info = verifier.process_frame(frame)
        out.write(debug_frame)

        t_sec = idx / fps

        if det_info:
            dets_str = ", ".join(
                f"{label} conf={conf:.2f} area={area:.4f}"
                for label, conf, area in det_info
            )
            print(f"  [{t_sec:6.2f}s] 감지: {dets_str}")

        if new_action is not None:
            action_changes.append((t_sec, new_action))
            print(f"  [{t_sec:6.2f}s] ★ 액션 -> {new_action}")

        idx += 1
        if idx % 50 == 0:
            print(f"  ... {idx}/{total} 프레임")

    cap.release()
    out.release()

    elapsed = time.perf_counter() - t_start
    print(f"\n완료! {idx}프레임 처리 ({elapsed:.1f}s, 평균 {idx/elapsed:.1f}fps)")
    print(f"결과 영상: {out_path}")
    print(f"\n감지된 액션 변화 {len(action_changes)}건:")
    for t_sec, action in action_changes:
        print(f"  {t_sec:6.2f}s : {action}")


if __name__ == '__main__':
    main()