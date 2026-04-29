#!/usr/bin/env python3
"""
上位机 WebSocket 视频接收端
在上位机 (192.168.137.134) 上运行：
  pip3 install websockets opencv-python
  python3 ws_viewer.py

机器人 vision_processor_node 的 ws_url 保持不变：
  ws://192.168.137.134:5000
"""

import asyncio
import websockets
import json
import base64
import numpy as np
import cv2
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("WSViewer")

latest_frame = None

async def handler(ws):
    global latest_frame
    addr = ws.remote_address
    logger.info(f"机器人已连接: {addr}")
    try:
        async for message in ws:
            data = json.loads(message)
            if data.get("type") == "video_stream":
                img_data = data.get("image_data", "")
                if img_data.startswith("data:image/jpeg;base64,"):
                    b64_str = img_data[len("data:image/jpeg;base64,"):]
                    img_bytes = base64.b64decode(b64_str)
                    np_arr = np.frombuffer(img_bytes, np.uint8)
                    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    if frame is not None:
                        latest_frame = frame
    except websockets.exceptions.ConnectionClosed:
        logger.info(f"机器人断开: {addr}")

async def main():
    # 启动 WebSocket 服务
    server = await websockets.serve(handler, "0.0.0.0", 5000)
    logger.info("WebSocket 接收端已启动, 端口: 5000")
    logger.info("等待机器人连接...")

    # OpenCV 显示窗口
    cv2.namedWindow("Robot Video Stream", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Robot Video Stream", 960, 720)

    while True:
        if latest_frame is not None:
            cv2.imshow("Robot Video Stream", latest_frame)
        key = cv2.waitKey(30) & 0xFF
        if key == 27 or key == ord('q'):  # ESC or q 退出
            break
        await asyncio.sleep(0.03)

    cv2.destroyAllWindows()
    server.close()
    logger.info("已退出")

if __name__ == "__main__":
    asyncio.run(main())
