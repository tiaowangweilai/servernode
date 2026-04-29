#!/usr/bin/env python3
"""
HTTP MJPEG 视频流服务节点
- 订阅 /vision/edge_preview/compressed（预压缩 JPEG 数据）
- 打开 http://IP:5000 直接显示视频
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import threading

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
<img src="/video_feed" />
</body>
</html>"""
            self.wfile.write(html.encode())
        elif self.path == "/video_feed":
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
                time.sleep(0.05)
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass

class MJPEGServerNode(Node):
    def __init__(self):
        super().__init__("mjpeg_server_node")
        # 订阅预压缩的 JPEG 数据（直接转发，无需再编码）
        self.sub = self.create_subscription(
            CompressedImage, "/vision/edge_preview/compressed",
            self.image_callback, 10)
        self.frame_count = 0
        self.skip_every = 4  # 30fps -> 7.5fps

        self.httpd = ThreadingHTTPServer(("0.0.0.0", 5000), MJPEGHandler)
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info("MJPEG server: http://0.0.0.0:5000/video_feed (pre-compressed, 15fps)")

    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.skip_every != 0:
            return
        # msg.data 已经是 JPEG 字节流，直接存储转发
        with frame_lock:
            global mjpeg_frame
            mjpeg_frame = msg.data

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
