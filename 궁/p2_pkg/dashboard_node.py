#!/usr/bin/env python3
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from flask import Flask, Response, render_template_string, jsonify


HTML_PAGE = """
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <title>Robot Vision Dashboard</title>
    <style>
        body {
            margin: 0;
            background: #111;
            color: #eee;
            font-family: Arial, sans-serif;
        }

        header {
            background: #1f1f1f;
            padding: 16px 24px;
            border-bottom: 1px solid #333;
        }

        h1 {
            margin: 0;
            font-size: 24px;
        }

        .container {
            display: grid;
            grid-template-columns: 1fr 1fr 360px;
            gap: 16px;
            padding: 20px;
        }

        .card {
            background: #1c1c1c;
            border: 1px solid #333;
            border-radius: 12px;
            padding: 16px;
        }

        .camera-box {
            text-align: center;
        }

        img {
            width: 100%;
            max-width: 640px;
            border: 2px solid #00ff99;
            border-radius: 8px;
            background: #000;
        }

        .yolo-img {
            border-color: #00aaff;
        }

        .status-value {
            font-size: 32px;
            font-weight: bold;
            color: #00ff99;
            margin-top: 12px;
        }

        .small {
            color: #aaa;
            font-size: 14px;
        }

        .row {
            margin-bottom: 16px;
        }

        .badge {
            display: inline-block;
            padding: 6px 10px;
            border-radius: 8px;
            background: #333;
            color: #fff;
            font-size: 13px;
        }

        @media (max-width: 1200px) {
            .container {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body>
    <header>
        <h1>Robot Vision Dashboard</h1>
        <div class="small">Raspberry Pi + ROS2 + Flask Debug View</div>
    </header>

    <div class="container">
        <div class="card camera-box">
            <h2>Raw Camera</h2>
            <div class="small">/image_raw/compressed</div>
            <br>
            <img src="/raw_video_feed" alt="raw camera stream">
        </div>

        <div class="card camera-box">
            <h2>YOLO Debug</h2>
            <div class="small">/image_yolo/compressed</div>
            <br>
            <img class="yolo-img" src="/yolo_video_feed" alt="yolo debug stream">
        </div>

        <div class="card">
            <h2>Status</h2>

            <div class="row">
                <div class="small">Raw Image Topic</div>
                <div class="badge">/image_raw/compressed</div>
            </div>

            <div class="row">
                <div class="small">YOLO Image Topic</div>
                <div class="badge">/image_yolo/compressed</div>
            </div>

            <div class="row">
                <div class="small">Traffic Sign Topic</div>
                <div class="badge">/traffic_sign_topic</div>
            </div>

            <div class="row">
                <div class="small">Last Published Action</div>
                <div id="traffic_sign" class="status-value">UNKNOWN</div>
            </div>

            <div class="row">
                <div class="small">Last Raw Image Time</div>
                <div id="last_raw_image_time">-</div>
            </div>

            <div class="row">
                <div class="small">Last YOLO Image Time</div>
                <div id="last_yolo_image_time">-</div>
            </div>

            <div class="row">
                <div class="small">Last Sign Time</div>
                <div id="last_sign_time">-</div>
            </div>
        </div>
    </div>

    <script>
        async function updateStatus() {
            try {
                const res = await fetch('/status');
                const data = await res.json();

                document.getElementById('traffic_sign').innerText = data.traffic_sign;
                document.getElementById('last_raw_image_time').innerText = data.last_raw_image_time;
                document.getElementById('last_yolo_image_time').innerText = data.last_yolo_image_time;
                document.getElementById('last_sign_time').innerText = data.last_sign_time;
            } catch (e) {
                console.error(e);
            }
        }

        setInterval(updateStatus, 500);
        updateStatus();
    </script>
</body>
</html>
"""


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')

        self.latest_raw_jpeg = None
        self.latest_yolo_jpeg = None

        self.latest_raw_image_time = "-"
        self.latest_yolo_image_time = "-"

        self.latest_sign = "UNKNOWN"
        self.latest_sign_time = "-"

        self.lock = threading.Lock()

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 원본 카메라 압축 이미지
        self.raw_image_sub = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.raw_image_callback,
            image_qos
        )

        # YOLO 박스/ROI가 그려진 압축 이미지
        self.yolo_image_sub = self.create_subscription(
            CompressedImage,
            'image_yolo/compressed',
            self.yolo_image_callback,
            image_qos
        )

        # 최종 표지판 판정 문자열
        self.sign_sub = self.create_subscription(
            String,
            'traffic_sign_topic',
            self.sign_callback,
            10
        )

        self.get_logger().info('Dashboard node started.')
        self.get_logger().info('Subscribing: /image_raw/compressed')
        self.get_logger().info('Subscribing: /image_yolo/compressed')
        self.get_logger().info('Subscribing: /traffic_sign_topic')

    def raw_image_callback(self, msg):
        with self.lock:
            self.latest_raw_jpeg = bytes(msg.data)
            self.latest_raw_image_time = time.strftime('%H:%M:%S')

    def yolo_image_callback(self, msg):
        with self.lock:
            self.latest_yolo_jpeg = bytes(msg.data)
            self.latest_yolo_image_time = time.strftime('%H:%M:%S')

    def sign_callback(self, msg):
        with self.lock:
            self.latest_sign = msg.data
            self.latest_sign_time = time.strftime('%H:%M:%S')

    def get_latest_raw_jpeg(self):
        with self.lock:
            return self.latest_raw_jpeg

    def get_latest_yolo_jpeg(self):
        with self.lock:
            return self.latest_yolo_jpeg

    def get_status(self):
        with self.lock:
            return {
                "traffic_sign": self.latest_sign,
                "last_raw_image_time": self.latest_raw_image_time,
                "last_yolo_image_time": self.latest_yolo_image_time,
                "last_sign_time": self.latest_sign_time,
            }


def create_flask_app(node: DashboardNode):
    app = Flask(__name__)

    @app.route('/')
    def index():
        return render_template_string(HTML_PAGE)

    @app.route('/status')
    def status():
        return jsonify(node.get_status())

    @app.route('/raw_video_feed')
    def raw_video_feed():
        def generate():
            while True:
                frame = node.get_latest_raw_jpeg()

                if frame is not None:
                    yield (
                        b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n' +
                        frame +
                        b'\r\n'
                    )

                time.sleep(0.05)

        return Response(
            generate(),
            mimetype='multipart/x-mixed-replace; boundary=frame'
        )

    @app.route('/yolo_video_feed')
    def yolo_video_feed():
        def generate():
            while True:
                frame = node.get_latest_yolo_jpeg()

                if frame is not None:
                    yield (
                        b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n' +
                        frame +
                        b'\r\n'
                    )

                time.sleep(0.05)

        return Response(
            generate(),
            mimetype='multipart/x-mixed-replace; boundary=frame'
        )

    return app


def main(args=None):
    rclpy.init(args=args)

    node = DashboardNode()
    app = create_flask_app(node)

    flask_thread = threading.Thread(
        target=lambda: app.run(
            host='0.0.0.0',
            port=5000,
            debug=False,
            use_reloader=False,
            threaded=True
        ),
        daemon=True
    )
    flask_thread.start()

    node.get_logger().info('Flask dashboard running at http://0.0.0.0:5000')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()