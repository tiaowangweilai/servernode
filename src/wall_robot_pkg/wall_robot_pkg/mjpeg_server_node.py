#!/usr/bin/env python3
"""
HTTP MJPEG 视频流服务节点
- 只提供 /vision/edge_preview 一个话题
- 打开 http://192.168.137.197:5000 直接显示视频
- 帧率 10fps，JPEG 质量 30，保持 640x480
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading
import cv2
import numpy as np

# ========== MJPEG 流处理器 ==========

mjpeg_frame = None
frame_lock = threading.Lock()

class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            html = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Robot Edge Preview</title>
<style>
  body { margin: 0; background: #000; display: flex; justify-content: center; align-items: center; height: 100vh; }
  img { max-width: 100vw; max-height: 100vh; }
</style>
</head>
<body>
<img src="/stream" />
</body>
</html>"""
            self.wfile.write(html.encode())
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "close")
            self.end_headers()
            while rclpy.ok():
                with frame_lock:
                    frame = mjpeg_frame
                if frame is not None:
                    try:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                    except:
                        break
                import time
                time.sleep(0.1)  # ~10fps
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # 不输出访问日志


class MJPEGServerNode(Node):
    def __init__(self):
        super().__init__("mjpeg_server_node")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, "/vision/edge_preview", self.image_callback, 10)
        self.frame_count = 0
        self.skip_every = 3  # 30fps -> 10fps

        # 启动 HTTP 服务（端口 5000）
        self.httpd = HTTPServer(("0.0.0.0", 5000), MJPEGHandler)
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info("MJPEG server started: http://0.0.0.0:5000  (640x480, 10fps, edge_preview only)")

    def image_callback(self, msg):
        self.frame_count += 1
        # 跳帧: 每 3 帧只处理 1 帧 -> 30fps 降到 10fps
        if self.frame_count % self.skip_every != 0:
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # JPEG 压缩，质量 30，保持 640x480
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
            result, encimg = cv2.imencode(".jpg", cv_img, encode_param)
            if result:
                with frame_lock:
                    global mjpeg_frame
                    mjpeg_frame = encimg.tobytes()
        except Exception as e:
            self.get_logger().error(f"Compress error: {e}")

    def destroy_node(self):
        self.httpd.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MJPEGServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
