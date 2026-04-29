"""
robot_motion.py
===============

封装：
1. SBUS-style 16 通道打包 → 25 字节协议帧  (pack_protocol_data)
2. 典型动作序列（前进 / 后退 / 左右转 / 静止）
3. 串口发送工具 send_frame / send_sequence

用法：
    python -m robot_motion  # 自测：连续前进 3 秒
"""

from __future__ import annotations
import time
import serial
from typing import List, Iterable, Optional, Tuple
# from .xunizhoudianji import *
# from xunizhoudianji import *
try:
    # 尝试作为包内模块导入 (当被其他脚本 import 时)
    from .xunizhoudianji import *
except ImportError:
    # 尝试作为普通脚本导入 (当直接运行时)
    from xunizhoudianji import *
__all__ = [
    "pack_protocol_data",
    "bytes_to_hexstr",
    "open_port",
    "send_frame",
    "send_sequence",
    "FORWARD",
    "BACKWARD",
    "TURN_LEFT",
    "TURN_RIGHT",
    "STOP",
    # 新增：可调构造器（只改一个通道）
    "build_forward",
    "build_backward",
    "build_turn_left",
    "build_turn_right",
]

# --------------------------------------------------
# 调试与工具
# --------------------------------------------------
DEBUG = True  # 统一调试开关，必要时可改为 False

def _dbg(*args):
    if DEBUG:
        print("[DEBUG]", *args)

def bytes_to_hexstr(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)

def _clamp_channel(val: int, ch_idx: int) -> Tuple[int, bool]:
    """限制通道取值到 [0, 2047]，返回(裁剪后数值, 是否发生裁剪)"""
    orig = val
    if val < 0:
        val = 0
    elif val > 2047:
        val = 2047
    clipped = (val != orig)
    if clipped:
        _dbg(f"通道 ch[{ch_idx}] 数值超界: {orig} -> {val} (已裁剪至 [0,2047])")
    return val, clipped

# --------------------------------------------------
# 低层：打包函数
# --------------------------------------------------
def pack_protocol_data(ch: List[int]) -> bytearray:
    """16 ch (0-2047) → 25-byte SBUS packet"""
    if len(ch) != 16:
        raise ValueError("需要 16 个通道数据")

    # 输入合法性与裁剪检测
    clipped_any = False
    for i in range(16):
        new_val, clipped = _clamp_channel(int(ch[i]), i)
        if clipped:
            clipped_any = True
        ch[i] = new_val
    # if clipped_any:
    #     _dbg("存在通道被裁剪，裁剪后的通道数组：", ch)
    # else:
    #     _dbg("通道数组校验通过：", ch)

    buf = bytearray(25)
    buf[0] = 0x0F  # header

    # 前 8 ch
    buf[1]  = (ch[0] >> 3) & 0xFF
    buf[2]  = ((ch[0] << 5) | (ch[1] >> 6)) & 0xFF
    buf[3]  = ((ch[1] << 2) | (ch[2] >> 9)) & 0xFF
    buf[4]  = (ch[2] >> 1) & 0xFF
    buf[5]  = ((ch[2] << 7) | (ch[3] >> 4)) & 0xFF
    buf[6]  = ((ch[3] << 4) | (ch[4] >> 7)) & 0xFF
    buf[7]  = ((ch[4] << 1) | (ch[5] >> 10)) & 0xFF
    buf[8]  = (ch[5] >> 2) & 0xFF
    buf[9]  = ((ch[5] << 6) | (ch[6] >> 5)) & 0xFF
    buf[10] = ((ch[6] << 3) | (ch[7] >> 8)) & 0xFF
    buf[11] = ch[7] & 0xFF

    # 后 8 ch
    buf[12] = (ch[8] >> 3) & 0xFF
    buf[13] = ((ch[8] << 5) | (ch[9] >> 6)) & 0xFF
    buf[14] = ((ch[9] << 2) | (ch[10] >> 9)) & 0xFF
    buf[15] = (ch[10] >> 1) & 0xFF
    buf[16] = ((ch[10] << 7) | (ch[11] >> 4)) & 0xFF
    buf[17] = ((ch[11] << 4) | (ch[12] >> 7)) & 0xFF
    buf[18] = ((ch[12] << 1) | (ch[13] >> 10)) & 0xFF
    buf[19] = (ch[13] >> 2) & 0xFF
    buf[20] = ((ch[13] << 6) | (ch[14] >> 5)) & 0xFF
    buf[21] = ((ch[14] << 3) | (ch[15] >> 8)) & 0xFF
    buf[22] = ch[15] & 0xFF

    # flag
    buf[23] = 0x00

    # # xor 校验 (1-23)
    # xor = 0
    # for i in range(1, 24):
    #     xor ^= buf[i]
    # buf[24] = xor
    # 改成和校验
    
    checksum = sum(buf[0:24]) & 0xFF
    buf[24] = checksum

    # _dbg("打包完成：len=25, header=0x0F, flag=0x00, xor=0x%02X" % xor)
    # _dbg("打包后的字节（HEX）：", bytes_to_hexstr(buf))
    return buf

# --------------------------------------------------
# 预设动作帧（保留原常量，便于兼容）
# --------------------------------------------------
def _ch(template: List[int]) -> bytearray:
    # _dbg("生成固定模板帧，模板通道：", template)
    return pack_protocol_data(template)



FORWARD    = _ch([1500,1730,1500,1500,1,1,1,0, 0,0,1050,1950,0,0,0,0])
BACKWARD   = _ch([1500,1270,1500,1500,1,1,1,0, 0,0,1050,1950,0,0,0,0])
TURN_LEFT  = _ch([1200,1500,1500,1500,1,1500,1,1500, 1500,1500,1050,1950,0,0,0,0])
TURN_RIGHT = _ch([1900,1500,1500,1500,1,1500,1,1500, 1500,1500,1050,1950,0,0,0,0])
STOP       = _ch([1500,1500,1500,1500,1,1,1,0, 0,0,1050,1900,0,0,0,0])  # 全通道居中
# STOP      = _ch([1500]*16)  # 全通道居中

# --------------------------------------------------
# 可调构造器：只改一个通道
#   - 前进/后退：改 ch[1]（第二个通道）
#   - 左转/右转：改 ch[0]（第一个通道）
# 其他通道保持与原模板一致
# --------------------------------------------------
def build_forward(ch2_value: Optional[int] = None) -> bytearray:
    tmpl = [1500,1270,1500,1500,1,1,1,0, 0,0,1050,1900,0,0,0,0]
    if ch2_value is not None:
        # _dbg(f"[FORWARD] 设定 ch[1] = {ch2_value} (原={tmpl[1]})")
        tmpl[1], _ = _clamp_channel(int(ch2_value), 1)
    return pack_protocol_data(tmpl)

def build_backward(ch2_value: Optional[int] = None) -> bytearray:
    tmpl = [1500,1730,1500,1500,1,1,1,0, 0,0,1050,1900,0,0,0,0]
    if ch2_value is not None:
        # _dbg(f"[BACKWARD] 设定 ch[1] = {ch2_value} (原={tmpl[1]})")
        tmpl[1], _ = _clamp_channel(int(ch2_value), 1)
    return pack_protocol_data(tmpl)

def build_turn_left(ch1_value: Optional[int] = None) -> bytearray:
    tmpl = [1150,1500,1500,1500,1,1500,1,1500, 1500,1500,1050,1900,0,0,0,0]
    if ch1_value is not None:
        # _dbg(f"[TURN_LEFT] 设定 ch[0] = {ch1_value} (原={tmpl[0]})")
        tmpl[0], _ = _clamp_channel(int(ch1_value), 0)
    return pack_protocol_data(tmpl)

def build_turn_right(ch1_value: Optional[int] = None) -> bytearray:
    tmpl = [1830,1500,1500,1500,1,1500,1,1500, 1500,1500,1050,1900,0,0,0,0]
    if ch1_value is not None:
        # _dbg(f"[TURN_RIGHT] 设定 ch[0] = {ch1_value} (原={tmpl[0]})")
        tmpl[0], _ = _clamp_channel(int(ch1_value), 0)
    return pack_protocol_data(tmpl)

# --------------------------------------------------
# 串口辅助
# --------------------------------------------------
def open_port(dev="/dev/ttyCH341USB0", baud=115200) -> serial.Serial:
    # _dbg(f"准备打开串口：dev={dev}, baud={baud}, 8N1, timeout=0")
    ser = serial.Serial(dev, baudrate=baud, bytesize=8,
                        parity=serial.PARITY_NONE, stopbits=1, timeout=0)
    # _dbg(f"串口已打开：{ser.port}, 实际波特率={ser.baudrate}, DSRTS={getattr(ser, 'rts', None)},{getattr(ser, 'dtr', None)}")
    return ser

def send_frame(ser: serial.Serial, frame: bytes, repeat:int=1, interval:float=0.02):
    """重复写同一帧，便于保持动作"""
    ser = open_port()
    # _dbg(f"send_frame: repeat={repeat}, interval={interval}s, frame_len={len(frame)}")
    # _dbg("frame(HEX)=", bytes_to_hexstr(frame))
    for idx in range(repeat):
        written = ser.write(frame)
        ser.flush()
        # _dbg(f"第 {idx+1}/{repeat} 次发送，写入字节={written}")
        time.sleep(interval)
    ser.close()

def send_sequence(ser: serial.Serial,
                  frames: Iterable[bytes],
                  interval: float = 0.02):
    """按序列发送多帧"""
    # _dbg("send_sequence: interval=", interval)
    for i, f in enumerate(frames, 1):
        _dbg(f"发送序列帧 #{i}, len={len(f)}, HEX={bytes_to_hexstr(f)}")
        written = ser.write(f)
        ser.flush()
        # _dbg(f"已写入字节={written}")
        time.sleep(interval)



def _demo():
    # 控制机器人底盘的串口（CH341）
    ser = open_port()
    print("串口已打开", ser.port)

    # # 配置电机参数（驱动器）
    control_system = SerialControlSystem(
        port='/dev/ttyACM0',      # 和 send_start_command_and_wait_for_down 用的那个口一致
        baudrate=115200,
        motor_port='/dev/ttyACM0',
        motor_baudrate=6000000,
        node_id=1
    )

    # 关键：先连接 /dev/ttyACM1
    if not control_system.connect():
        print("控制系统串口连接失败，退出 _demo")
        ser.close()
        return

    # 设零点
    set_current_position_as_zero('/dev/ttyACM0', 1, 6000000)

    try:

        # send_frame(ser, FORWARD, repeat=300, interval=0.02)
        # send_frame(ser, FORWARD, repeat=20, interval=0.02)
        # # 底盘先后退一小段
        send_frame(ser, BACKWARD, repeat=15, interval=0.02)
        # 然后跑电机 + 协议循环
        control_system.run_cycle(cycles=3)

        send_frame(ser, BACKWARD, repeat=15, interval=0.02)
        control_system.run_cycle(cycles=3)

        send_frame(ser, BACKWARD, repeat=15, interval=0.02)
        control_system.run_cycle(cycles=3)

        send_frame(ser, BACKWARD, repeat=15, interval=0.02)
        control_system.run_cycle(cycles=3)

        send_frame(ser, BACKWARD, repeat=15, interval=0.02)
        control_system.run_cycle(cycles=3)

        send_frame(ser, BACKWARD, repeat=15, interval=0.02)
        control_system.run_cycle(cycles=3)

        send_frame(ser, BACKWARD, repeat=15, interval=0.02)
        control_system.run_cycle(cycles=3)

        send_frame(ser, BACKWARD, repeat=15, interval=0.02)
        # 然后跑电机 + 协议循环
        control_system.run_cycle(cycles=3)

        send_frame(ser, BACKWARD, repeat=15, interval=0.02)
        control_system.run_cycle(cycles=3)

    finally:
        control_system.close()   # 关掉 /dev/ttyACM1
        ser.close()              # 关掉 CH341
        print("串口已关闭")




if __name__ == "__main__":
    _demo()
