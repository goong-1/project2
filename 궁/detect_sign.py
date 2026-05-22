import cv2
import time
from ultralytics import YOLO

MODEL_PATH = "traffic_light_best.pt"

DEVICE_ID = 0

WIDTH = 640
HEIGHT = 480
FPS = 15
FOURCC = "MJPG"

CONF = 0.35

model = YOLO(MODEL_PATH)

cap = cv2.VideoCapture(DEVICE_ID, cv2.CAP_DSHOW)

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*FOURCC))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)

if not cap.isOpened():
    print("카메라 열기 실패")
    print("DEVICE_ID를 1 또는 2로 바꿔보세요.")
    exit()

real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
real_fps = cap.get(cv2.CAP_PROP_FPS)

fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
fourcc_str = "".join(chr((fourcc >> 8 * i) & 0xFF) for i in range(4))

print("카메라 열기 성공")
print(f"실제 설정: {real_w}x{real_h}, FPS={real_fps}, FOURCC={fourcc_str}")
print("q: 종료")

prev_time = time.time()
last_print_time = 0

while True:
    ret, frame = cap.read()

    if not ret or frame is None:
        print("프레임 읽기 실패")
        time.sleep(0.1)
        continue

    results = model.predict(
        source=frame,
        conf=CONF,
        imgsz=640,
        verbose=False
    )

    result = results[0]
    annotated = result.plot()

    now = time.time()
    fps = 1.0 / max(now - prev_time, 1e-6)
    prev_time = now

    cv2.putText(
        annotated,
        f"FPS: {fps:.1f}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 0),
        2
    )

    if now - last_print_time > 0.5:
        detected = []

        if result.boxes is not None:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                score = float(box.conf[0])
                name = model.names[cls_id]
                detected.append(f"{name}:{score:.2f}")

        if detected:
            print("Detected:", ", ".join(detected))

        last_print_time = now

    cv2.imshow("Traffic Sign Detection - Windows", annotated)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
