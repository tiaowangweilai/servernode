#!/usr/bin/env python3
"""
上位机 WebSocket 视频接收服务
用法: python3 ws_video_server.py
然后浏览器打开 http://192.168.137.134:8080 即可查看机器人视频流
"""

import asyncio
import websockets
import json
import base64
import logging
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
import os

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("VideoServer")

# 存储最新一帧图片的 base64 数据
latest_frame = None
frame_lock = threading.Lock()

# ========== WebSocket 服务端 (接收机器人发来的图片) ==========

async def ws_handler(ws):
    global latest_frame
    logger.info(f"机器人已连接: {ws.remote_address}")
    async for message in ws:
        try:
            data = json.loads(message)
            if data.get("type") == "video_stream":
                img_data = data.get("image_data", "")
                with frame_lock:
                    latest_frame = img_data
        except Exception as e:
            logger.error(f"解析错误: {e}")

async def ws_server():
    async with websockets.serve(ws_handler, "0.0.0.0", 5000):
        logger.info("WebSocket 服务已启动，端口: 5000")
        await asyncio.Future()  # 永远运行

# ========== HTTP 服务 (提供网页查看视频) ==========

HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>机器人视频流</title>
<style>
  body { background: #000; margin: 0; display: flex; flex-direction: column; align-items: center; height: 100vh; font-family: Arial; }
  h2 { color: #fff; margin: 10px; }
  img { max-width: 95vw; max-height: 85vh; border: 2px solid #333; border-radius: 8px; }
  .status { color: #aaa; margin: 5px; font-size: 14px; }
</style>
</head>
<body>
<h2>🤖 机器人实时视频</h2>
<div class="status" id="status">等待连接...</div>
<img id="video" src="">
<script>
  const ws = new WebSocket("ws://" + location.hostname + ":5000");
  const img = document.getElementById("video");
  const status = document.getElementById("status");

  ws.onopen = () => { status.textContent = "已连接"; };
  ws.onclose = () => { status.textContent = "已断开，等待重连..."; setTimeout(() => location.reload(), 3000); };
  ws.onerror = () => { status.textContent = "连接错误"; };
  ws.onmessage = (e) => {
    try {
      const data = JSON.parse(e.data);
      if (data.image_data) img.src = data.image_data;
    } catch(err) {}
  };
</script>
</body>
</html>"""

class VideoPageHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(HTML_PAGE.encode("utf-8"))

def http_server():
    server = HTTPServer(("0.0.0.0", 8080), VideoPageHandler)
    logger.info("HTTP 网页服务已启动: http://0.0.0.0:8080")
    server.serve_forever()

# ========== 主函数 ==========

def main():
    # 启动 HTTP 服务线程
    http_thread = threading.Thread(target=http_server, daemon=True)
    http_thread.start()

    # 运行 WebSocket 服务
    asyncio.run(ws_server())

if __name__ == "__main__":
    main()
