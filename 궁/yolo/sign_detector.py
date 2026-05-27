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
    def __init__(self, model_path: str, conf_threshold: float = 0.35):
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold

        print("model names:", self.model.names)

        self.action_map = {
            "greenlight": "GO",
            "traffic_left_light_green": "TURN_LEFT",

            "redlight": "STOP",
            "traffic_light_red": "STOP",
            "stop_sign": "STOP",

            #"yellowlight": "SLOW",
            "limit_sign": "SPEED_LIMIT",

            #"light_off": "IGNORE",
        }

        self.color_map = {
            "STOP": (0, 0, 255),         # red
            "GO": (0, 255, 0),           # green
            #"SLOW": (0, 255, 255),       # yellow
            "SPEED_LIMIT": (255, 0, 0),  # blue
            #"IGNORE": (160, 160, 160),   # gray
            "UNKNOWN": (255, 255, 255),  # white
            "TURN_LEFT": (255, 255, 0),   # cyan
        }

    def detect_all(self, frame):
        h, w = frame.shape[:2]

        results = self.model.predict(
            source=frame,
            conf=self.conf_threshold,
            imgsz=320,
            agnostic_nms=True,
            iou=0.5,
            verbose=False
        )

        result = results[0]
        detections = []

        if result.boxes is None or len(result.boxes) == 0:
            return detections

        for box in result.boxes:
            cls_id = int(box.cls[0])
            confidence = float(box.conf[0])
            label = self.model.names[cls_id]

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
            
        detections = self.remove_overlapping_detections(detections,iou_threshold=0.5)
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
            #"IGNORE": 0,
            #"UNKNOWN": 0,
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
    
    def has_bright_red(
        self,
        frame,
        bbox,
        min_ratio=0.003,
        min_pixels=20,
        max_check_size=160,
        use_top_region=False,
        debug=False
    ):
        """
        bbox 영역 안에 '실제로 밝게 켜진 빨간색'이 있는지 검사한다.

        Args:
            frame: BGR 이미지
            bbox: {"x1": int, "y1": int, "x2": int, "y2": int}
            min_ratio: crop 영역 대비 빨간 픽셀 비율 기준
            min_pixels: 최소 빨간 픽셀 개수
            max_check_size: HSV 검사 전 crop 최대 크기 제한
            use_top_region: 신호등 전체 bbox일 때 위쪽 영역만 검사할지 여부
            debug: True면 (판단값, red_ratio, red_pixels) 반환

        Returns:
            debug=False: bool
            debug=True: (bool, red_ratio, red_pixels)
        """

        h, w = frame.shape[:2]

        # bbox 안전 보정
        x1 = max(0, min(int(bbox["x1"]), w - 1))
        y1 = max(0, min(int(bbox["y1"]), h - 1))
        x2 = max(0, min(int(bbox["x2"]), w))
        y2 = max(0, min(int(bbox["y2"]), h))

        if x2 <= x1 or y2 <= y1:
            return (False, 0.0, 0) if debug else False

        crop = frame[y1:y2, x1:x2]

        if crop.size == 0:
            return (False, 0.0, 0) if debug else False

        # 신호등 전체를 bbox로 잡는 경우, 빨간불은 보통 위쪽에 있음
        # 단, redlight 램프만 bbox로 잡는 모델이면 False로 두는 게 낫다.
        if use_top_region:
            ch = crop.shape[0]
            crop = crop[: max(1, int(ch * 0.45)), :]

        # 너무 큰 crop은 줄여서 HSV 연산량 감소
        ch, cw = crop.shape[:2]
        max_side = max(ch, cw)

        if max_side > max_check_size:
            scale = max_check_size / max_side
            new_w = max(1, int(cw * scale))
            new_h = max(1, int(ch * scale))
            crop = cv2.resize(crop, (new_w, new_h), interpolation=cv2.INTER_AREA)

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

        # 꺼진 빨간 렌즈는 어둡기 때문에 V 기준을 조금 높게 둠
        # H: 빨강 범위, S: 채도, V: 밝기
        lower_red1 = (0, 90, 150)
        upper_red1 = (10, 255, 255)

        lower_red2 = (170, 90, 150)
        upper_red2 = (180, 255, 255)

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        red_mask = cv2.bitwise_or(mask1, mask2)

        red_pixels = cv2.countNonZero(red_mask)
        total_pixels = crop.shape[0] * crop.shape[1]

        red_ratio = red_pixels / max(total_pixels, 1)

        ok = red_pixels >= min_pixels and red_ratio >= min_ratio

        if debug:
            return ok, red_ratio, red_pixels

        return ok
