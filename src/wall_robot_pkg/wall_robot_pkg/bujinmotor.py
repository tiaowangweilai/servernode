#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DMTP 驱动器控制 - 支持转矩模式、速度模式和位置模式
"""
import struct
import serial
import time
import math
from typing import Optional, Tuple, List


class DMTPDriver:
    # 控制指令
    FUNC_CONTROL = 0x21
    FUNC_ENABLE = 0x22
    FUNC_DISABLE = 0x23
    FUNC_CLEAR_ERROR = 0x28

    # 转矩控制
    FUNC_SET_TORQ = 0x31  # 设置轮廓转矩
    FUNC_SET_TORQ_ADV = 0x32  # 设置轮廓转矩(自定义斜率)

    # 速度控制
    FUNC_SET_VEL = 0x33  # 设置轮廓速度
    FUNC_SET_VEL_ADV = 0x34  # 设置轮廓速度(自定义加速度)

    # 位置控制
    FUNC_SET_POS = 0x35  # 设置轮廓位置
    FUNC_SET_POS_ADV = 0x36  # 设置轮廓位置(自定义参数)

    # 读取类
    FUNC_READ_STATUS = 0x42
    FUNC_READ_TORQ = 0x45
    FUNC_READ_VEL = 0x44
    FUNC_READ_POS = 0x43

    # 控制模式
    MODE_TORQUE = 0x01
    MODE_VELOCITY = 0x02
    MODE_POSITION = 0x03

    F32 = struct.Struct('<f')
    U16 = struct.Struct('<H')

    def __init__(self, port: str = 'COM6', node_id: int = 1,
                 baudrate: int = 1_000_000, timeout: float = 0.2):
        self.node = node_id
        self.ser = serial.Serial(port, baudrate,
                                 bytesize=8, parity='N', stopbits=1,
                                 timeout=timeout)
        if not self.ser.is_open:
            raise IOError(f'无法打开串口 {port}')

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    @staticmethod
    def _crc16_modbus(data: bytes) -> int:
        """MODBUS CRC16计算"""
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc = crc >> 1
        return crc

    def _build_tx_frame(self, func: int, payload: bytes = b'') -> bytes:
        """构建发送帧格式：节点ID + 功能码 + 数据 + CRC16"""
        frame = bytes([self.node, func]) + payload
        crc = self._crc16_modbus(frame)
        return frame + self.U16.pack(crc)

    def _parse_rx_frame(self, data: bytes) -> Optional[bytes]:
        """解析接收帧格式：节点ID + 功能码 + 数据长度 + 数据 + CRC16"""
        if len(data) < 5:  # 最小长度检查
            return None

        # 检查节点ID匹配
        if data[0] != self.node:
            # print(f"节点ID不匹配: 期望{self.node}, 收到{data[0]}")
            return None

        # 提取数据长度
        data_length = data[2]

        # 检查帧长度是否足够
        if len(data) < 5 + data_length:
            # print(f"帧长度不足: 需要{5 + data_length}, 实际{len(data)}")
            return None

        # 提取完整帧并验证CRC
        full_frame = data[:3 + data_length + 2]  # 头+数据+CRC
        frame_data = full_frame[:-2]  # 去掉CRC的部分
        received_crc = self.U16.unpack(full_frame[-2:])[0]
        calculated_crc = self._crc16_modbus(frame_data)

        if received_crc != calculated_crc:
            # print(f"CRC错误: 计算值={calculated_crc:04X}, 接收值={received_crc:04X}")
            return None

        # 返回数据部分
        return frame_data[3:3 + data_length]

    def _tx_rx(self, func: int, payload: bytes = b'', expect_response: bool = True) -> Optional[bytes]:
        """发送并接收数据"""
        tx_frame = self._build_tx_frame(func, payload)
        # print(f"TX: {tx_frame.hex(' ').upper()}")

        # 清空接收缓冲区
        self.ser.reset_input_buffer()

        # 发送数据
        self.ser.write(tx_frame)
        self.ser.flush()

        if not expect_response:
            time.sleep(0.01)  # 短暂延迟
            return None

        # 等待响应
        time.sleep(0.1)  # 给驱动器响应时间
        rx_data = self.ser.read_all()

        if not rx_data:
            print("RX: 无响应")
            return None

        # print(f"RX: {rx_data.hex(' ').upper()}")
        return self._parse_rx_frame(rx_data)

    # ==================== 基础控制功能 ====================
    def enable(self) -> bool:
        """使能电机"""
        result = self._tx_rx(self.FUNC_ENABLE, b'', expect_response=True)
        return result is not None and len(result) >= 1 and result[0] == 0x00

    def disable(self) -> bool:
        """禁用电机"""
        result = self._tx_rx(self.FUNC_DISABLE, b'', expect_response=True)
        return result is not None and len(result) >= 1 and result[0] == 0x00

    def clear_error(self) -> bool:
        """清除错误"""
        result = self._tx_rx(self.FUNC_CLEAR_ERROR, b'', expect_response=True)
        return result is not None and len(result) >= 1 and result[0] == 0x00

    def set_control_mode(self, mode: int) -> bool:
        """设置控制模式"""
        payload = bytes([mode])
        result = self._tx_rx(self.FUNC_CONTROL, payload, expect_response=True)
        time.sleep(0.1)
        return result is not None and len(result) >= 1 and result[0] == 0x00

    # ==================== 转矩控制功能 ====================
    def set_torque(self, torque_nm: float, slope_nm_per_s: float = 0.0) -> bool:
        """
        设置转矩值
        byte 1: echo (返回数据选择，0x00表示不返回)
        byte 2: sync (同步标志，0x00表示立即执行)
        byte 3-6: 目标转矩 (float32小端模式)
        byte 7-10: 转矩斜率 (可选，float32小端模式)
        """
        echo = 0x00  # 不返回额外数据
        sync = 0x00  # 立即执行

        if slope_nm_per_s == 0:
            # 简单转矩设置
            payload = bytes([echo, sync]) + self.F32.pack(torque_nm)
            self._tx_rx(self.FUNC_SET_TORQ, payload, expect_response=False)
        else:
            # 带斜率的转矩设置
            payload = bytes([echo, sync]) + self.F32.pack(torque_nm) + self.F32.pack(slope_nm_per_s)
            self._tx_rx(self.FUNC_SET_TORQ_ADV, payload, expect_response=False)

        return True

    # ==================== 速度控制功能 ====================
    def set_velocity(self, velocity_usr_s: float, acceleration_usr_s2: float = 0.0,
                     deceleration_usr_s2: float = 0.0) -> bool:
        """
        设置速度值
        byte 1: echo (返回数据选择，0x00表示不返回)
        byte 2: sync (同步标志，0x00表示立即执行)
        byte 3-6: 目标速度 (float32小端模式)
        byte 7-10: 加速度 (可选，float32小端模式)
        byte 11-14: 减速度 (可选，float32小端模式)
        """
        echo = 0x00  # 不返回额外数据
        sync = 0x00  # 立即执行

        if acceleration_usr_s2 == 0 and deceleration_usr_s2 == 0:
            # 简单速度设置
            payload = bytes([echo, sync]) + self.F32.pack(velocity_usr_s)
            self._tx_rx(self.FUNC_SET_VEL, payload, expect_response=False)
        else:
            # 带加速度的速度设置
            payload = (bytes([echo, sync]) +
                       self.F32.pack(velocity_usr_s) +
                       self.F32.pack(acceleration_usr_s2) +
                       self.F32.pack(deceleration_usr_s2))
            self._tx_rx(self.FUNC_SET_VEL_ADV, payload, expect_response=False)

        return True

    # ==================== 位置控制功能 ====================
    def set_position(self, position_usr: float, velocity_usr_s: float = 0.0,
                     acceleration_usr_s2: float = 0.0, deceleration_usr_s2: float = 0.0) -> bool:
        """
        设置位置值
        byte 1: echo (返回数据选择，0x00表示不返回)
        byte 2: sync (同步标志，0x00表示立即执行)
        byte 3-6: 目标位置 (float32小端模式)
        byte 7-10: 轮廓速度 (可选，float32小端模式)
        byte 11-14: 轮廓加速度 (可选，float32小端模式)
        byte 15-18: 轮廓减速度 (可选，float32小端模式)
        """
        echo = 0x00  # 不返回额外数据
        sync = 0x00  # 立即执行

        if velocity_usr_s == 0 and acceleration_usr_s2 == 0 and deceleration_usr_s2 == 0:
            # 简单位置设置
            payload = bytes([echo, sync]) + self.F32.pack(position_usr)
            self._tx_rx(self.FUNC_SET_POS, payload, expect_response=False)
        else:
            # 带参数的位置设置
            payload = (bytes([echo, sync]) +
                       self.F32.pack(position_usr) +
                       self.F32.pack(velocity_usr_s) +
                       self.F32.pack(acceleration_usr_s2) +
                       self.F32.pack(deceleration_usr_s2))
            self._tx_rx(self.FUNC_SET_POS_ADV, payload, expect_response=False)

        return True

    # ==================== 读取功能 ====================
    def get_actual_torque(self) -> float:
        """读取实际转矩"""
        data = self._tx_rx(self.FUNC_READ_TORQ, b'', expect_response=True)
        return self.F32.unpack(data)[0] if data and len(data) == 4 else 0.0

    def get_actual_velocity(self) -> float:
        """读取实际速度"""
        data = self._tx_rx(self.FUNC_READ_VEL, b'', expect_response=True)
        return self.F32.unpack(data)[0] if data and len(data) == 4 else 0.0

    def get_actual_position(self) -> float:
        """读取实际位置"""
        data = self._tx_rx(self.FUNC_READ_POS, b'', expect_response=True)
        return self.F32.unpack(data)[0] if data and len(data) == 4 else 0.0

    def get_status(self) -> Tuple[int, int]:
        """读取状态字和错误字"""
        data = self._tx_rx(self.FUNC_READ_STATUS, b'', expect_response=True)
        if not data or len(data) < 4:
            return 0, 0
        return self.U16.unpack(data[:2])[0], self.U16.unpack(data[2:4])[0]

    def check_motor_status(self) -> dict:
        """检查电机状态"""
        status, error = self.get_status()

        result = {
            'status_word': status,
            'error_word': error,
            'control_mode': (status >> 12) & 0x0F,
            'motor_enabled': bool(status & 0x0001),
            'target_reached': bool(status & 0x0002),
            'zero_speed': bool(status & 0x0004),
        }

        modes = {
            0x01: "轮廓转矩", 0x02: "轮廓速度", 0x03: "轮廓位置",
            0x04: "转矩跟随", 0x05: "速度跟随", 0x06: "位置跟随",
            0x0A: "原点回零"
        }

        # print(f"状态字: 0x{status:04X}, 错误字: 0x{error:04X}")
        # print(f"控制模式: {modes.get(result['control_mode'], '未知')}")
        # print(f"电机使能: {'是' if result['motor_enabled'] else '否'}")
        # print(f"目标到达: {'是' if result['target_reached'] else '否'}")
        # print(f"零速到达: {'是' if result['zero_speed'] else '否'}")

        if error:
            errors = []
            if error & 0x0001: errors.append("欠压保护")
            if error & 0x0002: errors.append("过压保护")
            if error & 0x0004: errors.append("过流保护")
            if error & 0x0008: errors.append("跟随错误")
            if error & 0x0010: errors.append("过载错误")
            print(f"错误: {', '.join(errors)}")

        return result

    def monitor_motor_short(self, duration: float = 3.0, interval: float = 0.5):
        """简化的电机状态监控"""
        start_time = time.time()
        # print("时间\t转矩(Nm)\t速度(usr/s)\t位置(usr)\t目标到达")
        # print("-" * 60)

        while time.time() - start_time < duration:
            torque = self.get_actual_torque()
            velocity = self.get_actual_velocity()
            position = self.get_actual_position()
            status, _ = self.get_status()
            target_reached = bool(status & 0x0002)

            elapsed = time.time() - start_time
            # print(
            #     f"{elapsed:5.1}f\t{torque:7.3f}\t{velocity:10.1f}\t{position:10.1f}\t{'是' if target_reached else '否'}")
            time.sleep(interval)

    def wait_for_target_reached(self, timeout: float =2, check_interval: float = 0.1) -> bool:
        """等待目标到达"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            status, _ = self.get_status()
            if status & 0x0002:  # 目标到达标志
                return True
            time.sleep(check_interval)
        return False

    def stop_motor(self):
        """安全停止电机"""
        # 根据当前模式选择停止方式
        status, _ = self.get_status()
        control_mode = (status >> 12) & 0x0F

        if control_mode == self.MODE_TORQUE:
            self.set_torque(0.0)
        elif control_mode == self.MODE_VELOCITY:
            self.set_velocity(0.0)
        elif control_mode == self.MODE_POSITION:
            # 位置模式下保持当前位置
            current_position = self.get_actual_position()
            self.set_position(current_position)

        time.sleep(0.1)
        self.disable()


def torque_mode_test(torque_value: float, port: str = 'COM5', node_id: int = 1, baudrate: int = 1_000_000):
    """转矩模式测试"""
    try:
        d = DMTPDriver(port, node_id, baudrate)

        # print("=== 初始化 ===")
        d.clear_error()
        d.check_motor_status()

        # print("\n=== 设置转矩模式 ===")
        d.set_control_mode(DMTPDriver.MODE_TORQUE)

        # print("\n=== 使能电机 ===")
        d.enable()
        d.check_motor_status()

        # print(f"\n=== 设置转矩: {torque_value} Nm ===")
        d.set_torque(torque_value)
        d.monitor_motor_short(5.0)

        return True

    except Exception as e:
        # print(f"测试过程中出现错误: {e}")
        import traceback
        traceback.print_exc()
        return False

    finally:
        # print("\n=== 停止电机 ===")
        if 'd' in locals():
            d.stop_motor()


def velocity_mode_test(velocity_value: float, port: str = 'COM5', node_id: int = 1, baudrate: int = 1_000_000, acceleration: float = 100.0):
    """速度模式测试"""
    try:
        d = DMTPDriver(port, node_id, baudrate)

        # print("=== 初始化 ===")
        d.clear_error()
        d.check_motor_status()

        # print("\n=== 设置速度模式 ===")
        d.set_control_mode(DMTPDriver.MODE_VELOCITY)

        # print("\n=== 使能电机 ===")
        d.enable()
        d.check_motor_status()

        # print(f"\n=== 设置速度: {velocity_value} usr/s, 加速度: {acceleration} usr/s² ===")
        d.set_velocity(velocity_value, acceleration, acceleration)
        d.monitor_motor_short(8.0)
        return True

    except Exception as e:
        # print(f"测试过程中出现错误: {e}")
        import traceback
        traceback.print_exc()
        return False

    finally:
        # print("\n=== 停止电机 ===")
        if 'd' in locals():
            d.stop_motor()

last_pos = 0.0   # 记录非零位移
def position_mode_test(target_position: float, port: str = 'COM5', node_id: int = 1, baudrate: int = 1_000_000,
                       velocity: float = 2, acceleration: float = 20.0, deceleration: float = 20.0):
    """位置模式测试"""
    global last_pos
    try:
        if target_position==0:
            pos = last_pos
        else:
            pos = target_position
        d = DMTPDriver(port, node_id, baudrate)

        # print("=== 初始化 ===")
        d.clear_error()
        d.check_motor_status()

        # 获取当前位置
        current_position = d.get_actual_position()
        # print(f"当前位置: {current_position:.1f} usr")

        # print("\n=== 设置位置模式 ===")
        d.set_control_mode(DMTPDriver.MODE_POSITION)

        # print("\n=== 使能电机 ===")
        d.enable()
        d.check_motor_status()

        # print(f"\n=== 移动到位置: {target_position} usr ===")
        # print(f"速度: {velocity} usr/s, 加速度: {acceleration} usr/s², 减速度: {deceleration} usr/s²")

        d.set_position(target_position, velocity, acceleration, deceleration)

        # 监控运动过程
        d.monitor_motor_short(0.0)
        
        if target_position != 0:
            # 传进来的是非零目标：用这个目标 & 更新 last_pos
            pos = target_position
            last_pos = target_position
        else:
            # 传进来的是 0：复用上次的非零目标
            pos = last_pos
            
        last_pos = target_position
        # 等待目标到达
        if d.wait_for_target_reached(timeout=math.fabs(pos)/velocity):
            print("目标位置已到达!")
        else:
            print("目标位置未在超时时间内到达")

        return True

    except Exception as e:
        # print(f"测试过程中出现错误: {e}")
        import traceback
        traceback.print_exc()
        return False

    finally:
        # print("\n=== 停止电机 ===")
        if 'd' in locals():
            d.stop_motor()


# ==================== 主程序 ====================
if __name__ == '__main__':
    # 默认配置（仅当直接运行此文件时使用）
    PORT = '/dev/ttyACM0'
    NODE_ID = 1
    BAUDRATE = 6_000_000

    # torque_mode_test(0.5)
    # velocity_mode_test(-1)

    while True:
        position_mode_test(1, PORT, NODE_ID, BAUDRATE)
        time.sleep(0.1)
        position_mode_test(4, PORT, NODE_ID, BAUDRATE)
        time.sleep(0.1)