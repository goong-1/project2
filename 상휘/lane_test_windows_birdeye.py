import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np


class BirdEyeLaneDetector:
    """
    Windows/OpenCV standalone lane detector.

    - 원본 화면에서 사다리꼴 ROI를 Bird-eye view로 변환
    - Bird-eye 화면에서 노란/흰 경계선, 검은 중앙선, 빨간 정지선 검출
    - 1차선/2차선 중심선과 target lane error 계산
    - 원본 화면 + Bird-eye 화면 둘 다 디버그 시각화
    """

    def __init__(
        self,
        resize_width=640,
        bird_width=640,
        bird_height=360,
        lane_numbering="left_to_right",
        target_lane=0,
        default_lane_width_px=210,
        boundary_color="yellow",
        calib_file="bird_eye_points.json",
    ):
        self.resize_width = int(resize_width)
        self.bird_width = int(bird_width)
        self.bird_height = int(bird_height)
        self.lane_numbering = lane_numbering
        self.target_lane = int(target_lane)  # 0=current, 1=lane1, 2=lane2
        self.default_lane_width_px = int(default_lane_width_px)
        self.boundary_color = boundary_color
        self.calib_file = Path(calib_file)

        self.frame_w = None
        self.frame_h = None
        self.src_points = None
        self.dst_points = np.float32([
            [80, 0],
            [self.bird_width - 80, 0],
            [self.bird_width - 80, self.bird_height - 1],
            [80, self.bird_height - 1],
        ])
        self.M = None
        self.M_inv = None

        self.last_divider_x = None
        self.last_left_boundary_x = None
        self.last_right_boundary_x = None
        self.last_current_lane = 0

    def initialize_perspective(self, frame_w, frame_h):
        if self.frame_w == frame_w and self.frame_h == frame_h and self.M is not None:
            return

        self.frame_w = frame_w
        self.frame_h = frame_h

        loaded = self.load_calibration(frame_w, frame_h)
        if not loaded:
            self.src_points = self.default_src_points(frame_w, frame_h)

        self.update_transform()

    def default_src_points(self, w, h):
        return np.float32([
            [0.30 * w, 0.42 * h],
            [0.70 * w, 0.42 * h],
            [0.96 * w, 0.94 * h],
            [0.04 * w, 0.94 * h],
        ])

    def update_transform(self):
        self.M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)
        self.M_inv = cv2.getPerspectiveTransform(self.dst_points, self.src_points)

    def set_src_points(self, points):
        if len(points) != 4:
            return False
        self.src_points = np.float32(points)
        self.update_transform()
        return True

    def save_calibration(self):
        if self.src_points is None:
            return

        data = {
            "frame_width": int(self.frame_w),
            "frame_height": int(self.frame_h),
            "points": self.src_points.astype(float).tolist(),
        }

        self.calib_file.write_text(
            json.dumps(data, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        print(f"[CALIB] saved: {self.calib_file}")

    def load_calibration(self, frame_w, frame_h):
        if not self.calib_file.exists():
            return False

        try:
            data = json.loads(self.calib_file.read_text(encoding="utf-8"))
            points = np.float32(data["points"])

            saved_w = float(data.get("frame_width", frame_w))
            saved_h = float(data.get("frame_height", frame_h))

            if saved_w <= 0 or saved_h <= 0:
                return False

            scale_x = frame_w / saved_w
            scale_y = frame_h / saved_h
            points[:, 0] *= scale_x
            points[:, 1] *= scale_y

            self.src_points = points
            print(f"[CALIB] loaded: {self.calib_file}")
            return True

        except Exception as e:
            print(f"[CALIB] load failed: {e}")
            return False

    def detect(self, frame):
        if frame is None:
            return None

        h, w = frame.shape[:2]
        if self.resize_width > 0 and w != self.resize_width:
            scale = self.resize_width / float(w)
            frame = cv2.resize(frame, (self.resize_width, int(h * scale)))

        h, w = frame.shape[:2]
        self.initialize_perspective(w, h)

        bird = cv2.warpPerspective(frame, self.M, (self.bird_width, self.bird_height))
        masks = self.make_masks(bird)

        divider_x = self.detect_black_divider_x(masks["black"])
        left_boundary_x, right_boundary_x, boundary_xs = self.detect_lane_boundaries(
            masks["boundary"],
            divider_x,
        )
        stop_line_visible = self.detect_stop_line(masks["red"])

        if divider_x is None and self.last_divider_x is not None:
            divider_x = self.last_divider_x
        if left_boundary_x is None and self.last_left_boundary_x is not None:
            left_boundary_x = self.last_left_boundary_x
        if right_boundary_x is None and self.last_right_boundary_x is not None:
            right_boundary_x = self.last_right_boundary_x

        visible = divider_x is not None

        if divider_x is not None:
            self.last_divider_x = divider_x
        if left_boundary_x is not None:
            self.last_left_boundary_x = left_boundary_x
        if right_boundary_x is not None:
            self.last_right_boundary_x = right_boundary_x

        current_lane = self.estimate_current_lane(divider_x)
        if current_lane != 0:
            self.last_current_lane = current_lane
        elif self.last_current_lane != 0:
            current_lane = self.last_current_lane

        lane1_center_x, lane2_center_x = self.estimate_lane_centers(
            divider_x,
            left_boundary_x,
            right_boundary_x,
        )

        if self.target_lane == 1:
            target_lane = 1
        elif self.target_lane == 2:
            target_lane = 2
        else:
            target_lane = current_lane

        target_center_x = self.select_target_center(
            target_lane,
            lane1_center_x,
            lane2_center_x,
        )

        bird_center_x = self.bird_width // 2
        if visible and target_center_x is not None:
            error = (target_center_x - bird_center_x) / float(bird_center_x)
            error = max(min(error, 1.0), -1.0)
        else:
            error = 0.0

        bird_debug = self.draw_bird_debug(
            bird=bird,
            masks=masks,
            divider_x=divider_x,
            left_boundary_x=left_boundary_x,
            right_boundary_x=right_boundary_x,
            boundary_xs=boundary_xs,
            lane1_center_x=lane1_center_x,
            lane2_center_x=lane2_center_x,
            target_lane=target_lane,
            target_center_x=target_center_x,
            current_lane=current_lane,
            error=error,
            visible=visible,
            stop_line_visible=stop_line_visible,
        )

        original_debug = self.draw_original_debug(
            frame=frame,
            visible=visible,
            current_lane=current_lane,
            target_lane=target_lane,
            error=error,
            stop_line_visible=stop_line_visible,
        )

        return {
            "frame": frame,
            "bird": bird,
            "debug": original_debug,
            "bird_debug": bird_debug,
            "visible": visible,
            "current_lane": current_lane,
            "target_lane": target_lane,
            "error": error,
            "divider_x": divider_x,
            "left_boundary_x": left_boundary_x,
            "right_boundary_x": right_boundary_x,
            "boundary_xs": boundary_xs,
            "lane1_center_x": lane1_center_x,
            "lane2_center_x": lane2_center_x,
            "stop_line_visible": stop_line_visible,
        }

    def make_masks(self, bird):
        hsv = cv2.cvtColor(bird, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(bird, cv2.COLOR_BGR2GRAY)

        yellow = self.make_yellow_mask(hsv)
        white = self.make_white_mask(hsv)
        black = self.make_black_mask(gray)
        red = self.make_red_mask(hsv)

        if self.boundary_color == "yellow":
            boundary = yellow
        elif self.boundary_color == "white":
            boundary = white
        else:
            boundary = cv2.bitwise_or(yellow, white)

        return {
            "yellow": self.clean_mask(yellow),
            "white": self.clean_mask(white),
            "black": self.clean_mask(black),
            "red": self.clean_mask(red),
            "boundary": self.clean_mask(boundary),
        }

    def make_yellow_mask(self, hsv):
        lower = np.array([18, 55, 55])
        upper = np.array([48, 255, 255])
        return cv2.inRange(hsv, lower, upper)

    def make_white_mask(self, hsv):
        lower = np.array([0, 0, 175])
        upper = np.array([180, 70, 255])
        return cv2.inRange(hsv, lower, upper)

    def make_black_mask(self, gray):
        _, mask = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
        return mask

    def make_red_mask(self, hsv):
        lower_red1 = np.array([0, 80, 80])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 80, 80])
        upper_red2 = np.array([180, 255, 255])
        return cv2.bitwise_or(
            cv2.inRange(hsv, lower_red1, upper_red1),
            cv2.inRange(hsv, lower_red2, upper_red2),
        )

    def clean_mask(self, mask):
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        vertical_kernel = np.ones((11, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, vertical_kernel)
        return mask

    def find_x_peaks(self, mask, y1_ratio=0.15, y2_ratio=0.95, min_pixels=8, peak_ratio=0.28):
        h, w = mask.shape[:2]
        y1 = int(h * y1_ratio)
        y2 = int(h * y2_ratio)

        region = mask[y1:y2, :]
        binary = region > 0
        hist = np.sum(binary, axis=0).astype(np.float32)

        if hist.max() <= 0:
            return []

        hist_img = hist.reshape(1, -1)
        hist_smooth = cv2.GaussianBlur(hist_img, (1, 31), 0).flatten()

        threshold = max(float(min_pixels), float(hist_smooth.max()) * float(peak_ratio))
        active = hist_smooth >= threshold

        peaks = []
        start = None

        for i, value in enumerate(active):
            if value and start is None:
                start = i
            elif not value and start is not None:
                end = i - 1
                peaks.append((start, end))
                start = None

        if start is not None:
            peaks.append((start, len(active) - 1))

        centers = []
        for start, end in peaks:
            if end - start < 3:
                continue
            weights = hist_smooth[start:end + 1]
            xs = np.arange(start, end + 1)
            if weights.sum() <= 0:
                centers.append(int((start + end) / 2))
            else:
                centers.append(int(np.average(xs, weights=weights)))

        return centers

    def detect_black_divider_x(self, black_mask):
        peaks = self.find_x_peaks(
            black_mask,
            y1_ratio=0.10,
            y2_ratio=0.95,
            min_pixels=10,
            peak_ratio=0.35,
        )

        if not peaks:
            return None

        w = black_mask.shape[1]
        center = w // 2

        valid = [x for x in peaks if int(w * 0.12) <= x <= int(w * 0.88)]
        if not valid:
            return None

        return min(valid, key=lambda x: abs(x - center))

    def detect_lane_boundaries(self, boundary_mask, divider_x):
        peaks = self.find_x_peaks(
            boundary_mask,
            y1_ratio=0.10,
            y2_ratio=0.95,
            min_pixels=8,
            peak_ratio=0.25,
        )

        if not peaks:
            return None, None, []

        w = boundary_mask.shape[1]
        peaks = [x for x in peaks if int(w * 0.02) <= x <= int(w * 0.98)]

        if not peaks:
            return None, None, []

        if divider_x is None:
            return min(peaks), max(peaks), peaks

        left_candidates = [x for x in peaks if x < divider_x]
        right_candidates = [x for x in peaks if x > divider_x]

        left_boundary_x = max(left_candidates) if left_candidates else None
        right_boundary_x = min(right_candidates) if right_candidates else None

        return left_boundary_x, right_boundary_x, peaks

    def detect_stop_line(self, red_mask):
        h, w = red_mask.shape[:2]
        region = red_mask[int(h * 0.25): int(h * 0.95), :]
        if region.size == 0:
            return False

        row_pixels = np.sum(region > 0, axis=1)
        if row_pixels.max() < w * 0.18:
            return False

        lines = cv2.HoughLinesP(
            region,
            rho=1,
            theta=np.pi / 180,
            threshold=35,
            minLineLength=int(w * 0.20),
            maxLineGap=30,
        )

        if lines is None:
            return False

        for line in lines:
            x1, y1, x2, y2 = line[0]
            dx = x2 - x1
            dy = y2 - y1
            if abs(dx) < w * 0.15:
                continue
            angle = abs(np.degrees(np.arctan2(dy, dx)))
            if angle < 15 or angle > 165:
                return True

        return False

    def estimate_current_lane(self, divider_x):
        if divider_x is None:
            return 0

        robot_x = self.bird_width // 2
        robot_is_left_of_divider = robot_x < divider_x

        if self.lane_numbering == "left_to_right":
            return 1 if robot_is_left_of_divider else 2

        return 2 if robot_is_left_of_divider else 1

    def estimate_lane_centers(self, divider_x, left_boundary_x, right_boundary_x):
        if divider_x is None:
            return None, None

        if left_boundary_x is None:
            left_boundary_x = int(divider_x - self.default_lane_width_px)
        if right_boundary_x is None:
            right_boundary_x = int(divider_x + self.default_lane_width_px)

        left_boundary_x = max(0, min(self.bird_width - 1, int(left_boundary_x)))
        right_boundary_x = max(0, min(self.bird_width - 1, int(right_boundary_x)))

        if self.lane_numbering == "left_to_right":
            lane1_center = int((left_boundary_x + divider_x) / 2)
            lane2_center = int((divider_x + right_boundary_x) / 2)
        else:
            lane2_center = int((left_boundary_x + divider_x) / 2)
            lane1_center = int((divider_x + right_boundary_x) / 2)

        return lane1_center, lane2_center

    def select_target_center(self, target_lane, lane1_center_x, lane2_center_x):
        if target_lane == 1:
            return lane1_center_x
        if target_lane == 2:
            return lane2_center_x
        return None

    def draw_bird_debug(
        self,
        bird,
        masks,
        divider_x,
        left_boundary_x,
        right_boundary_x,
        boundary_xs,
        lane1_center_x,
        lane2_center_x,
        target_lane,
        target_center_x,
        current_lane,
        error,
        visible,
        stop_line_visible,
    ):
        debug = bird.copy()
        h, w = debug.shape[:2]

        overlay = debug.copy()
        overlay[masks["boundary"] > 0] = (0, 255, 0)
        overlay[masks["black"] > 0] = (0, 0, 255)
        overlay[masks["red"] > 0] = (255, 0, 255)

        debug = cv2.addWeighted(overlay, 0.35, debug, 0.65, 0)

        center_x = w // 2
        cv2.line(debug, (center_x, 0), (center_x, h), (255, 255, 255), 2)
        cv2.putText(debug, "robot center", (center_x + 8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        for x in boundary_xs:
            cv2.line(debug, (int(x), 0), (int(x), h), (0, 160, 0), 1)

        if left_boundary_x is not None:
            cv2.line(debug, (int(left_boundary_x), 0), (int(left_boundary_x), h), (0, 255, 0), 3)
            cv2.putText(debug, "left boundary", (int(left_boundary_x) + 5, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if right_boundary_x is not None:
            cv2.line(debug, (int(right_boundary_x), 0), (int(right_boundary_x), h), (0, 255, 0), 3)
            cv2.putText(debug, "right boundary", (int(right_boundary_x) + 5, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if divider_x is not None:
            cv2.line(debug, (int(divider_x), 0), (int(divider_x), h), (0, 0, 255), 3)
            cv2.putText(debug, "black divider", (int(divider_x) + 5, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        if lane1_center_x is not None:
            cv2.line(debug, (int(lane1_center_x), 0), (int(lane1_center_x), h), (255, 255, 0), 2)
            cv2.putText(debug, "lane1 center", (int(lane1_center_x) + 5, h - 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        if lane2_center_x is not None:
            cv2.line(debug, (int(lane2_center_x), 0), (int(lane2_center_x), h), (255, 160, 0), 2)
            cv2.putText(debug, "lane2 center", (int(lane2_center_x) + 5, h - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 160, 0), 1)

        if target_center_x is not None:
            cv2.line(debug, (int(target_center_x), 0), (int(target_center_x), h), (0, 255, 255), 4)
            cv2.circle(debug, (int(target_center_x), int(h * 0.78)), 9, (0, 255, 255), -1)
            cv2.line(debug, (center_x, int(h * 0.78)), (int(target_center_x), int(h * 0.78)), (0, 255, 255), 2)

        cv2.putText(debug, f"visible: {visible}", (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.putText(debug, f"current_lane: {current_lane}", (15, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.putText(debug, f"target_lane: {target_lane}", (15, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.putText(debug, f"error: {error:.2f}", (15, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.putText(
            debug,
            f"stop_line: {stop_line_visible}",
            (15, 125),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255) if stop_line_visible else (0, 255, 255),
            2,
        )

        return debug

    def draw_original_debug(
        self,
        frame,
        visible,
        current_lane,
        target_lane,
        error,
        stop_line_visible,
    ):
        debug = frame.copy()

        pts = self.src_points.astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(debug, [pts], isClosed=True, color=(255, 0, 0), thickness=2)

        for idx, p in enumerate(self.src_points.astype(int)):
            cv2.circle(debug, tuple(p), 6, (0, 255, 255), -1)
            cv2.putText(debug, str(idx + 1), (p[0] + 8, p[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        if self.M_inv is not None:
            bird_center = self.bird_width // 2
            guide_points = np.float32([
                [bird_center, self.bird_height - 1],
                [bird_center, int(self.bird_height * 0.55)],
            ]).reshape(-1, 1, 2)
            original_points = cv2.perspectiveTransform(guide_points, self.M_inv).astype(int)
            p1 = tuple(original_points[0, 0])
            p2 = tuple(original_points[1, 0])
            cv2.line(debug, p1, p2, (255, 255, 255), 2)

        cv2.putText(debug, "Original + Bird-eye ROI", (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
        cv2.putText(debug, "p: pause | m: mark 4 ROI points | r: reset | q: quit", (15, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2)
        cv2.putText(
            debug,
            f"visible={visible} current={current_lane} target={target_lane} error={error:.2f} stop={stop_line_visible}",
            (15, 88),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 255),
            2,
        )

        return debug


class MouseCalibrator:
    def __init__(self, detector):
        self.detector = detector
        self.active = False
        self.points = []

    def start(self):
        self.active = True
        self.points = []
        print("[CALIB] click 4 points in order: left-top, right-top, right-bottom, left-bottom")

    def reset_default(self):
        if self.detector.frame_w is None or self.detector.frame_h is None:
            return
        self.detector.src_points = self.detector.default_src_points(self.detector.frame_w, self.detector.frame_h)
        self.detector.update_transform()
        self.detector.save_calibration()
        print("[CALIB] reset to default")

    def on_mouse(self, event, x, y, flags, param):
        if not self.active:
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            self.points.append([x, y])
            print(f"[CALIB] point {len(self.points)}: ({x}, {y})")

            if len(self.points) == 4:
                self.detector.set_src_points(self.points)
                self.detector.save_calibration()
                self.active = False
                print("[CALIB] completed")


def parse_source(source):
    if source.isdigit():
        return int(source)
    return source


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="0", help="camera index or video path")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--resize-width", type=int, default=640)

    parser.add_argument("--bird-width", type=int, default=640)
    parser.add_argument("--bird-height", type=int, default=360)
    parser.add_argument("--target-lane", type=int, default=0, choices=[0, 1, 2], help="0=current lane, 1=lane 1, 2=lane 2")
    parser.add_argument("--lane-numbering", default="left_to_right", choices=["left_to_right", "right_to_left"])
    parser.add_argument("--boundary-color", default="yellow", choices=["yellow", "white", "yellow_white"])
    parser.add_argument("--calib-file", default="bird_eye_points.json")
    parser.add_argument("--default-lane-width", type=int, default=210)

    args = parser.parse_args()

    source = parse_source(args.source)

    if isinstance(source, int):
        cap = cv2.VideoCapture(source, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS, args.fps)
    else:
        cap = cv2.VideoCapture(source)

    if not cap.isOpened():
        print("카메라 또는 영상 열기 실패")
        return

    detector = BirdEyeLaneDetector(
        resize_width=args.resize_width,
        bird_width=args.bird_width,
        bird_height=args.bird_height,
        lane_numbering=args.lane_numbering,
        target_lane=args.target_lane,
        default_lane_width_px=args.default_lane_width,
        boundary_color=args.boundary_color,
        calib_file=args.calib_file,
    )

    calibrator = MouseCalibrator(detector)

    cv2.namedWindow("Original View", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Bird Eye Lane Debug", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Original View", calibrator.on_mouse)

    print("Bird-eye 차선감지 테스트 시작")
    print("q: 종료")
    print("p: 일시정지")
    print("m: 원본 화면에서 bird-eye ROI 4점 찍기")
    print("r: bird-eye ROI 기본값으로 리셋")
    print("1: target_lane=1")
    print("2: target_lane=2")
    print("0: target_lane=current")

    last_print_time = 0
    paused = False
    last_frame = None

    while True:
        if not paused:
            ret, frame = cap.read()

            if not ret or frame is None:
                print("프레임 읽기 실패 또는 영상 종료")
                break

            last_frame = frame
        else:
            frame = last_frame

        if frame is not None:
            result = detector.detect(frame)

            if result is not None:
                original_debug = result["debug"].copy()
                bird_debug = result["bird_debug"].copy()

                if calibrator.active:
                    for idx, p in enumerate(calibrator.points):
                        cv2.circle(original_debug, tuple(p), 7, (0, 0, 255), -1)
                        cv2.putText(
                            original_debug,
                            str(idx + 1),
                            (p[0] + 8, p[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 0, 255),
                            2,
                        )
                    cv2.putText(
                        original_debug,
                        "CALIB MODE: click 4 points LT, RT, RB, LB",
                        (15, 120),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (0, 0, 255),
                        2,
                    )

                now = time.time()
                if now - last_print_time > 0.5:
                    print(
                        f"visible={result['visible']} | "
                        f"current_lane={result['current_lane']} | "
                        f"target_lane={result['target_lane']} | "
                        f"error={result['error']:.2f} | "
                        f"left={result['left_boundary_x']} | "
                        f"divider={result['divider_x']} | "
                        f"right={result['right_boundary_x']} | "
                        f"lane1_center={result['lane1_center_x']} | "
                        f"lane2_center={result['lane2_center_x']} | "
                        f"stop_line={result['stop_line_visible']}"
                    )
                    last_print_time = now

                cv2.imshow("Original View", original_debug)
                cv2.imshow("Bird Eye Lane Debug", bird_debug)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        elif key == ord("p"):
            paused = not paused
            print("paused:", paused)

        elif key == ord("m"):
            paused = True
            calibrator.start()

        elif key == ord("r"):
            calibrator.reset_default()

        elif key == ord("1"):
            detector.target_lane = 1
            print("target_lane=1")

        elif key == ord("2"):
            detector.target_lane = 2
            print("target_lane=2")

        elif key == ord("0"):
            detector.target_lane = 0
            print("target_lane=current")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
