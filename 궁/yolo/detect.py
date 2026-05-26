from pathlib import Path
import cv2
import time
import json

from sign_detector import TrafficSignDetector

BASE_DIR = Path(__file__).resolve().parent
MODEL_PATH = BASE_DIR / "best_v2.pt"

detector = TrafficSignDetector(
    model_path=str(MODEL_PATH),
    conf_threshold=0.70
)

DEVICE_ID = 0

cap = cv2.VideoCapture(DEVICE_ID, cv2.CAP_DSHOW)

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 15)

if not cap.isOpened():
    print("카메라 열기 실패")
    exit()

print("카메라 열기 성공")
print("q: 종료")

last_print_time = 0

while True:
    ret, frame = cap.read()

    if not ret or frame is None:
        print("프레임 읽기 실패")
        time.sleep(0.1)
        continue

    # 모든 객체 탐지
    detections = detector.detect_all(frame)

    # 제어용 대표 판단 1개 선택
    main_action = detector.get_main_action(detections)

    now = time.time()

    if now - last_print_time > 0.5:
        print("ALL:")
        print(json.dumps([d.to_dict() for d in detections], ensure_ascii=False, indent=2))

        print("MAIN:")
        print(json.dumps(main_action, ensure_ascii=False, indent=2))

        last_print_time = now

    # 화면에는 모든 박스 표시
    frame = detector.draw_all(frame, detections)

    cv2.imshow("Traffic Sign Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()