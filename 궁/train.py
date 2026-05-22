from ultralytics import YOLO

def main():
    model = YOLO("yolov8n.pt")

    model.train(
        data="data.yaml",
        epochs=80,
        imgsz=640,
        batch=4,
        patience=20,
        workers=2,
        project="runs/detect",
        name="traffic_sign_v1"
    )

if __name__ == "__main__":
    main()