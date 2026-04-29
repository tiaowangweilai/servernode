#!/usr/bin/env python3
"""
上位机 WebSocket 视频接收端（轻量版，仅需 websockets 库）
运行: pip3 install websockets && python3 ws_viewer_lite.py

收到图片自动保存到当前目录，方便其他程序读取显示。
"""

import asyncio
import websockets
import json
import base64
import logging
import os

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("WSViewer")

frame_count = 0

async def handler(ws):
    global frame_count
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
                    frame_count += 1
                    # 保存最新帧到文件
                    with open("latest_frame.jpg", "wb") as f:
                        f.write(img_bytes)
                    if frame_count % 30 == 0:
                        logger.info(f"已接收 {frame_count} 帧")
    except websockets.exceptions.ConnectionClosed:
        logger.info(f"机器人断开: {addr}")

async def main():
    await websockets.serve(handler, "0.0.0.0", 5000)
    logger.info("WebSocket 接收端已启动, 端口: 5000")
    logger.info("等待机器人连接...")
    logger.info("最新帧保存到: ./latest_frame.jpg")
    await asyncio.Future()  # 永远运行

if __name__ == "__main__":
    asyncio.run(main())
