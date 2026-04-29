#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
主控逻辑：
1. 通过主串口发送 'A'；
2. 等待返回 'down'；
3. 调用 position_mode_test()；
4. 等待返回 'arrive'；
5. 循环进行。
"""

import serial
import time
from bujinmotor1 import position_mode_test  # 引入你的 DMTP 驱动代码

# ==================== 可调参数 ====================
SERIAL_MAIN_PORT = '/dev/ttyCH341USB0'    # 主控串口，用于 'A' <-> 'down'
SERIAL_MAIN_BAUD = 115200

READ_TIMEOUT = 5.0        # 等待反馈的超时时间
SLEEP_AFTER_SEND = 0.05   # 发送后等待硬件响应的延迟
MAX_LOOP = 20             # 最大循环次数
# ==================================================


def wait_for_feedback(ser, target: bytes, timeout: float = READ_TIMEOUT) -> bool:
    """
    等待串口返回指定字符串 target，若超时则返回 False
    """
    start_time = time.time()
    buffer = b''

    while time.time() - start_time < timeout:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            buffer += data
            try:
                s = buffer.decode(errors='ignore')
                print(f"[RX] {s}", end='', flush=True)
            except UnicodeDecodeError:
                pass
            if target in buffer:
                print(f"\n[INFO] 收到目标反馈 {target.decode()}")
                return True
        time.sleep(0.05)

    print(f"\n[ERROR] 超时未收到 {target.decode()} 响应")
    return False


def main_loop():
    try:
        ser_main = serial.Serial(SERIAL_MAIN_PORT, SERIAL_MAIN_BAUD, timeout=0.1)
        print(f"[INFO] 主串口已打开: {SERIAL_MAIN_PORT}, 波特率: {SERIAL_MAIN_BAUD}")
    except Exception as e:
        print(f"[ERROR] 主串口打开失败: {e}")
        return

    # ================= 循环逻辑 =================
    loop_idx = 0
    while loop_idx < MAX_LOOP:
        loop_idx += 1
        print(f"\n===== 循环 {loop_idx}/{MAX_LOOP} =====")

        # 1️⃣ 发送字符 'A'
        ser_main.reset_input_buffer()
        ser_main.write(b'A')
        ser_main.flush()
        print("[TX] 已发送字符 'A'")

    
        # 2️⃣ 等待 'down'
        if not wait_for_feedback(ser_main, b'down', timeout=READ_TIMEOUT):
            print("[STOP] 未收到 down，终止循环。")
            break

        # 3️⃣ 收到 down → 调用 position_mode_test()
        target_pos = 3 if loop_idx % 2 == 0 else 0
        print(f"[ACTION] 调用 position_mode_test({target_pos})")

        # 执行电机动作（内部使用 DMTPDriver 控制）

        # 执行电机动作（内部使用 DMTPDriver 控制）
        flag = position_mode_test(target_pos)

        # === 判断返回值 ===
        if not flag:
            print(f"[ERROR] 电机动作执行失败（position_mode_test 返回 False），系统卡主。")
            while True:
                time.sleep(0.5)  # 无限等待人工干预
                
        # 5️⃣ 一次闭环完成，继续下一轮
        print("[OK] 一次动作完成，准备进入下一轮。")
        time.sleep(0.5)

    ser_main.close()
    print("[END] 循环结束，串口关闭。")


if __name__ == "__main__":
    main_loop()
