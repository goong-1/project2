from dataclasses import dataclass, asdict
import time
import json
import cv2
from ultralytics import YOLO


@dataclass
class SignDetection:
    detected: bool
    label: str
    confidence: float
    action_hint: str
    bbox: dict
    center: dict
    area_ratio: float
    timestamp: float

    def to_dict(self):
        return asdict(self)


class TrafficSignDetector:
    def __init__(self, model_path: str, conf_threshold: float = 0.25):
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold

        print("model names:", self.model.names)

        self.action_map = {
            "greenlight": "GO",
            "traffic_left_light_green": "TURN_LEFT",

            "redlight": "STOP",
            "traffic_light_red": "STOP",
            "stop_sign": "STOP",

            "limit_sign": "SPEED_LIMIT",
            "construction": "OBSTACLE",
        }
        
         # ── 여기에 추가 ──
        self.label_merge = {
            "traffic_light_red": "redlight",   # 빨간불 두 종류를 redlight로 통일
        }

        # ── 라벨별 confidence 임계값 (없는 라벨은 default 사용) ──
        # conf_threshold(YOLO 1차 관문)보다 높은 값으로 줘야 의미가 있음
        self.label_thresholds = {
            "greenlight":               0.70,
            "traffic_left_light_green": 0.70,
            "redlight":                 0.70,
            "traffic_light_red":        0.70,
            "stop_sign":                0.70, 
            "limit_sign":               0.70,
            "construction":             0.70,
        }
        self.default_label_threshold = 0.40

        self.color_map = {
            "STOP": (0, 0, 255),         # red
            "GO": (0, 255, 0),           # green
            "SPEED_LIMIT": (255, 0, 0),  # blue
            "UNKNOWN": (255, 255, 255),  # white
            "TURN_LEFT": (255, 255, 0),  # cyan
            "OBSTACLE": (0, 255, 255),   # yellow
        }

    def detect_all(self, frame):
        h, w = frame.shape[:2]

        results = self.model.predict(
            source=frame,
            conf=self.conf_threshold,
            imgsz=640,
            agnostic_nms=True,
            iou=0.5,
            verbose=False
        )

        result = results[0]
        detections = []

        if result.boxes is None or len(result.boxes) == 0:
            return detections

        allowed_labels = {
            "greenlight",
            "traffic_left_light_green",
            "redlight",
            "traffic_light_red",
            "stop_sign",
            "limit_sign",
            #"construction",
        }

        for box in result.boxes:
            cls_id = int(box.cls[0])
            confidence = float(box.conf[0])
            label = self.model.names[cls_id]
            
            self.min_area_by_label = {
                "redlight":                 0.0052,
                "traffic_light_red":        0.0029,
                "greenlight":               0.001,
                "traffic_left_light_green": 0.0029,
                "stop_sign":                0.001,
                "limit_sign":               0.001,
                "construction":             0.001,
            }

            if label not in allowed_labels:
                continue

            # ── 라벨별 임계값 적용 ──
            label_thr = self.label_thresholds.get(label, self.default_label_threshold)
            if confidence < label_thr:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

            bw = max(0, x2 - x1)
            bh = max(0, y2 - y1)

            detection = SignDetection(
                detected=True,
                label=label,
                confidence=confidence,
                action_hint=self.action_map.get(label, "GO"),
                bbox={
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                },
                center={
                    "x": int((x1 + x2) / 2),
                    "y": int((y1 + y2) / 2),
                },
                area_ratio=(bw * bh) / float(w * h),
                timestamp=time.time(),
            )

            detections.append(detection)

        detections = self.remove_overlapping_detections(detections, iou_threshold=0.5)
        return detections

    def draw_all(self, frame, detections):
        for detection in detections:
            bbox = detection.bbox

            x1 = bbox["x1"]
            y1 = bbox["y1"]
            x2 = bbox["x2"]
            y2 = bbox["y2"]

            color = self.color_map.get(detection.action_hint, (255, 255, 255))

            text = f"{detection.label} {detection.confidence:.2f} {detection.action_hint}"

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            cv2.putText(
                frame,
                text,
                (x1, max(25, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2
            )

        return frame

    def get_main_action(self, detections):
        if not detections:
            return {
                "detected": False,
                "label": None,
                "confidence": 0.0,
                "action_hint": "GO",
            }

        # STOP 계열을 우선순위로 둘 수도 있음
        priority = {
            "STOP": 3,
            "SPEED_LIMIT": 2,
            "TURN_LEFT": 1,
            "GO": 0,
        }

        valid_actions = {"STOP", "GO", "SPEED_LIMIT", "TURN_LEFT"}

        valid_detections = [
            d for d in detections
            if d.action_hint in valid_actions
        ]

        if not valid_detections:
            return {
                "detected": False,
                "label": None,
                "confidence": 0.0,
                "action_hint": "GO",
            }

        best = max(
            valid_detections,
            key=lambda d: (priority.get(d.action_hint, 0), d.confidence)
        )

        return best.to_dict()

    def remove_overlapping_detections(self, detections, iou_threshold=0.5):
        def iou(box_a, box_b):
            ax1, ay1, ax2, ay2 = box_a["x1"], box_a["y1"], box_a["x2"], box_a["y2"]
            bx1, by1, bx2, by2 = box_b["x1"], box_b["y1"], box_b["x2"], box_b["y2"]

            inter_x1 = max(ax1, bx1)
            inter_y1 = max(ay1, by1)
            inter_x2 = min(ax2, bx2)
            inter_y2 = min(ay2, by2)

            inter_w = max(0, inter_x2 - inter_x1)
            inter_h = max(0, inter_y2 - inter_y1)
            inter_area = inter_w * inter_h

            area_a = max(0, ax2 - ax1) * max(0, ay2 - ay1)
            area_b = max(0, bx2 - bx1) * max(0, by2 - by1)

            union = area_a + area_b - inter_area
            if union == 0:
                return 0

            return inter_area / union

        detections = sorted(detections, key=lambda d: d.confidence, reverse=True)
        kept = []

        for det in detections:
            duplicated = False

            for kept_det in kept:
                if iou(det.bbox, kept_det.bbox) > iou_threshold:
                    duplicated = True
                    break

            if not duplicated:
                kept.append(det)

        return kept

    # def has_bright_red(
    #     self,
    #     frame,
    #     bbox,
    #     min_ratio=0.003,
    #     min_pixels=20,
    #     max_check_size=160,
    #     use_top_region=False,
    #     debug=False
    # ):
    #     """
    #     bbox 영역 안에 '실제로 밝게 켜진 빨간색'이 있는지 검사한다.
    #     (현재 detect_all 에서는 사용하지 않음. 필요 시 다시 연결)
    #     """
    #     h, w = frame.shape[:2]

    #     x1 = max(0, min(int(bbox["x1"]), w - 1))
    #     y1 = max(0, min(int(bbox["y1"]), h - 1))
    #     x2 = max(0, min(int(bbox["x2"]), w))
    #     y2 = max(0, min(int(bbox["y2"]), h))

    #     if x2 <= x1 or y2 <= y1:
    #         return (False, 0.0, 0) if debug else False

    #     crop = frame[y1:y2, x1:x2]

    #     if crop.size == 0:
    #         return (False, 0.0, 0) if debug else False

    #     if use_top_region:
    #         ch = crop.shape[0]
    #         crop = crop[: max(1, int(ch * 0.45)), :]

    #     ch, cw = crop.shape[:2]
    #     max_side = max(ch, cw)

    #     if max_side > max_check_size:
    #         scale = max_check_size / max_side
    #         new_w = max(1, int(cw * scale))
    #         new_h = max(1, int(ch * scale))
    #         crop = cv2.resize(crop, (new_w, new_h), interpolation=cv2.INTER_AREA)

    #     hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

    #     lower_red1 = (0, 90, 150)
    #     upper_red1 = (10, 255, 255)
    #     lower_red2 = (170, 90, 150)
    #     upper_red2 = (180, 255, 255)

    #     mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    #     mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    #     red_mask = cv2.bitwise_or(mask1, mask2)

    #     red_pixels = cv2.countNonZero(red_mask)
    #     total_pixels = crop.shape[0] * crop.shape[1]
    #     red_ratio = red_pixels / max(total_pixels, 1)

    #     ok = red_pixels >= min_pixels and red_ratio >= min_ratio

    #     if debug:
    #         return ok, red_ratio, red_pixels

    #     return ok