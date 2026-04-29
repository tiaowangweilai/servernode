# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# """
# 步进电机驱动器控制 - 封装函数
# 提供两个核心功能：设置零点、运动到指定位置
# """
# import struct
# import serial
# import time
# import threading
# from typing import Optional, Dict, Any
# from enum import IntEnum


# class RegisterAddress(IntEnum):
#     """寄存器地址定义"""
#     HARDWARE_VERSION = 0x0000
#     SOFTWARE_VERSION = 0x0002
#     MOTOR_POSITION = 0x0004
#     STATUS_REGISTER = 0x0006

#     UART_TIMEOUT = 0x0008
#     BAUD_RATE = 0x0009
#     SMOOTH_CONSTANT = 0x000A
#     DYNAMIC_ERROR_THRESHOLD = 0x000B
#     STATIC_ERROR_THRESHOLD = 0x000C
#     RATED_CURRENT = 0x000D
#     IDLE_CURRENT_PERCENT = 0x000E
#     ENCODER_LINES = 0x000F
#     POSITION_WARNING = 0x0010
#     ACTUAL_POSITION_ERROR = 0x0011
#     ALARM_CONFIG = 0x0012
#     REAL_TIME_SPEED = 0x0019
#     REAL_TIME_CURRENT = 0x001A

#     # 运动参数
#     START_SPEED = 0x0096
#     STOP_SPEED = 0x0097
#     ACCEL_TIME = 0x0098
#     DECEL_TIME = 0x0099
#     RUN_SPEED = 0x009A

#     # 运行指令
#     RUN_STOP = 0x00C8
#     DISABLE_ENABLE = 0x00D4
#     MOTOR_RUN_TO_ABS = 0x00D0
#     SET_CURRENT_POSITION = 0x00D2
#     SAVE_COMMAND = 0x00DC


# class MotorDirection(IntEnum):
#     CW = 0
#     CCW = 1


# class DMTPDriver:
#     """步进电机驱动器控制类"""

#     U16 = struct.Struct('<H')
#     U32 = struct.Struct('<I')

#     def __init__(self, port: str = 'COM5', node_id: int = 1,
#                  baudrate: int = 115200, timeout: float = 0.5, debug: bool = False):
#         self.node = node_id
#         self.timeout = timeout
#         self.debug = debug
#         self.pulses_per_rev = 4000

#         import serial
#         self.ser = serial.Serial(port, baudrate,
#                                  bytesize=8, parity='N', stopbits=1,
#                                  timeout=timeout)
#         if not self.ser.is_open:
#             raise IOError(f'无法打开串口 {port}')

#         self._lock = threading.Lock()

#     def close(self):
#         if self.ser and self.ser.is_open:
#             self.ser.close()

#     def __enter__(self):
#         return self

#     def __exit__(self, exc_type, exc_val, exc_tb):
#         self.close()

#     @staticmethod
#     def _crc16_modbus(data: bytes) -> int:
#         crc = 0xFFFF
#         for b in data:
#             crc ^= b
#             for _ in range(8):
#                 if crc & 0x0001:
#                     crc = (crc >> 1) ^ 0xA001
#                 else:
#                     crc = crc >> 1
#         return crc

#     def _read_register(self, address: int) -> Optional[int]:
#         """读取单个寄存器 (16位)"""
#         with self._lock:
#             frame = bytes([self.node, 0x03, address >> 8, address & 0xFF, 0x00, 0x01])
#             crc = self._crc16_modbus(frame)
#             frame += self.U16.pack(crc)

#             self.ser.reset_input_buffer()
#             self.ser.write(frame)
#             self.ser.flush()

#             time.sleep(0.05)
#             rx_data = self.ser.read_all()

#             if not rx_data or len(rx_data) < 5:
#                 return None
#             if rx_data[0] != self.node or rx_data[1] != 0x03:
#                 return None

#             full_frame = rx_data[:-2] if len(rx_data) >= 2 else rx_data
#             received_crc = self.U16.unpack(rx_data[-2:])[0]
#             calculated_crc = self._crc16_modbus(full_frame)

#             if received_crc != calculated_crc:
#                 return None

#             return self.U16.unpack(rx_data[3:5])[0]

#     def _write_register(self, address: int, value: int) -> bool:
#         """写入单个寄存器 (16位)"""
#         with self._lock:
#             frame = bytes([self.node, 0x06, address >> 8, address & 0xFF,
#                            (value >> 8) & 0xFF, value & 0xFF])
#             crc = self._crc16_modbus(frame)
#             frame += self.U16.pack(crc)

#             self.ser.reset_input_buffer()
#             self.ser.write(frame)
#             self.ser.flush()

#             time.sleep(0.05)
#             rx_data = self.ser.read_all()

#             if not rx_data or len(rx_data) < 8:
#                 return False
#             if rx_data[0] != self.node or rx_data[1] != 0x06:
#                 return False

#             full_frame = rx_data[:-2] if len(rx_data) >= 2 else rx_data
#             received_crc = self.U16.unpack(rx_data[-2:])[0]
#             calculated_crc = self._crc16_modbus(full_frame)

#             return received_crc == calculated_crc

#     def _read_dword(self, address: int) -> Optional[int]:
#         """读取双字寄存器 (32位)，返回有符号整数"""
#         with self._lock:
#             frame = bytes([self.node, 0x03, address >> 8, address & 0xFF, 0x00, 0x02])
#             crc = self._crc16_modbus(frame)
#             frame += self.U16.pack(crc)

#             self.ser.reset_input_buffer()
#             self.ser.write(frame)
#             self.ser.flush()

#             time.sleep(0.05)
#             rx_data = self.ser.read_all()

#             if not rx_data or len(rx_data) < 7:
#                 return None
#             if rx_data[0] != self.node or rx_data[1] != 0x03:
#                 return None

#             full_frame = rx_data[:-2] if len(rx_data) >= 2 else rx_data
#             received_crc = self.U16.unpack(rx_data[-2:])[0]
#             calculated_crc = self._crc16_modbus(full_frame)

#             if received_crc != calculated_crc:
#                 return None

#             data_len = rx_data[2]
#             if data_len >= 4:
#                 value = self.U32.unpack(rx_data[3:7])[0]
#                 # 将无符号值转换为有符号整数 (32位补码)
#                 if value >= 0x80000000:
#                     value = value - 0x100000000
#                 return value
#             return None

#     def _write_dword(self, address: int, value: int) -> bool:
#         """写入双字寄存器 (32位)，支持有符号整数"""
#         with self._lock:
#             # 将有符号整数转换为无符号整数表示（32位补码）
#             if value < 0:
#                 value = value + (1 << 32)

#             data = self.U32.pack(value)
#             frame = bytes([self.node, 0x10, address >> 8, address & 0xFF, 0x00, 0x02, 0x04])
#             frame += data
#             crc = self._crc16_modbus(frame)
#             frame += self.U16.pack(crc)

#             self.ser.reset_input_buffer()
#             self.ser.write(frame)
#             self.ser.flush()

#             time.sleep(0.05)
#             rx_data = self.ser.read_all()

#             if not rx_data or len(rx_data) < 8:
#                 return False
#             if rx_data[0] != self.node or rx_data[1] != 0x10:
#                 return False

#             full_frame = rx_data[:-2] if len(rx_data) >= 2 else rx_data
#             received_crc = self.U16.unpack(rx_data[-2:])[0]
#             calculated_crc = self._crc16_modbus(full_frame)

#             return received_crc == calculated_crc

#     # ==================== 主要接口 ====================

#     def get_position(self) -> Optional[int]:
#         """获取电机实时位置 (脉冲数，有符号整数)"""
#         return self._read_dword(RegisterAddress.MOTOR_POSITION)

#     def get_speed(self) -> Optional[int]:
#         """获取实时速度 (rpm)"""
#         value = self._read_register(RegisterAddress.REAL_TIME_SPEED)
#         if value is not None:
#             if value >= 0x8000:
#                 return value - 0x10000
#             return value
#         return None

#     def get_status(self) -> Optional[Dict[str, Any]]:
#         """获取状态寄存器"""
#         value = self._read_register(RegisterAddress.STATUS_REGISTER)
#         if value is None:
#             return None

#         running_state = (value >> 8) & 0x03
#         running_names = {0: "空闲", 1: "启动中", 2: "停止中", 3: "运行中"}

#         return {
#             'raw': value,
#             'running_state': running_state,
#             'running_state_name': running_names.get(running_state, "未知"),
#             'running': running_state == 3,
#             'position_reached': bool(value & 0x1000),
#             'x0_input': bool(value & 0x0001),
#             'x1_input': bool(value & 0x0002),
#             'x2_input': bool(value & 0x0004),
#             'x3_input': bool(value & 0x0008),
#             'x4_input': bool(value & 0x0010),
#             'x5_input': bool(value & 0x0020),
#             'x6_input': bool(value & 0x0040),
#             'x7_input': bool(value & 0x0080),
#         }

#     def set_run_speed(self, speed_rpm: int) -> bool:
#         """设置运行速度 (rpm)"""
#         if speed_rpm < 0 or speed_rpm > 10000:
#             raise ValueError(f"速度范围: 0-10000 rpm")
#         return self._write_register(RegisterAddress.RUN_SPEED, speed_rpm)

#     def set_acceleration(self, accel_time_ms: int) -> bool:
#         """设置加速时间 (ms)"""
#         if accel_time_ms < 0 or accel_time_ms > 65535:
#             raise ValueError(f"加速时间范围: 0-65535 ms")
#         return self._write_register(RegisterAddress.ACCEL_TIME, accel_time_ms)

#     def set_deceleration(self, decel_time_ms: int) -> bool:
#         """设置减速时间 (ms)"""
#         if decel_time_ms < 0 or decel_time_ms > 65535:
#             raise ValueError(f"减速时间范围: 0-65535 ms")
#         return self._write_register(RegisterAddress.DECEL_TIME, decel_time_ms)

#     def set_start_speed(self, speed_rpm: int) -> bool:
#         """设置启动速度 (rpm)"""
#         if speed_rpm < 0 or speed_rpm > 65535:
#             raise ValueError(f"启动速度范围: 0-65535 rpm")
#         return self._write_register(RegisterAddress.START_SPEED, speed_rpm)

#     def enable(self) -> bool:
#         """使能电机"""
#         return self._write_register(RegisterAddress.DISABLE_ENABLE, 0)

#     def disable(self) -> bool:
#         """禁用电机"""
#         return self._write_register(RegisterAddress.DISABLE_ENABLE, 1)

#     def set_current_position(self, position_pulses: int) -> bool:
#         """设置当前电机位置 (将当前位置设置为指定值)"""
#         return self._write_dword(RegisterAddress.SET_CURRENT_POSITION, position_pulses)

#     def move_to_absolute(self, position_pulses: int) -> bool:
#         """移动到绝对位置"""
#         return self._write_dword(RegisterAddress.MOTOR_RUN_TO_ABS, position_pulses)

#     def stop_motor(self, emergency: bool = False) -> bool:
#         """停止电机"""
#         if emergency:
#             return self._write_register(RegisterAddress.RUN_STOP, 256)
#         else:
#             return self._write_register(RegisterAddress.RUN_STOP, 0)

#     def wait_for_stop(self, timeout: float = 10, interval: float = 0.1) -> bool:
#         """等待电机停止"""
#         start_time = time.time()
#         while time.time() - start_time < timeout:
#             status = self.get_status()
#             if status and not status.get('running', False):
#                 return True
#             time.sleep(interval)
#         return False

#     def wait_for_position_reached(self, timeout: float = 10, interval: float = 0.05) -> bool:
#         """等待位置到达"""
#         start_time = time.time()
#         while time.time() - start_time < timeout:
#             status = self.get_status()
#             if status and status.get('position_reached', False):
#                 return True
#             time.sleep(interval)
#         return False

#     def wait_for_position_stable(self, target_position: int,
#                                  timeout: float = 30,
#                                  tolerance: int = 10,
#                                  stable_samples: int = 3) -> bool:
#         """
#         等待位置稳定到达目标位置

#         Args:
#             target_position: 目标位置
#             timeout: 超时时间（秒）
#             tolerance: 位置容差（脉冲）
#             stable_samples: 需要稳定的采样次数
#         """
#         start_time = time.time()
#         positions = []

#         while time.time() - start_time < timeout:
#             current_pos = self.get_position()
#             if current_pos is None:
#                 time.sleep(0.05)
#                 continue

#             if abs(current_pos - target_position) <= tolerance:
#                 positions.append(current_pos)

#                 if len(positions) > stable_samples:
#                     positions.pop(0)

#                 if len(positions) >= stable_samples:
#                     pos_range = max(positions) - min(positions)
#                     if pos_range <= 5:
#                         return True
#             else:
#                 positions = []

#             time.sleep(0.05)

#         final_pos = self.get_position()
#         if final_pos is not None and abs(final_pos - target_position) <= tolerance:
#             return True

#         return False


# # ==================== 封装的两个核心函数 ====================

# def set_current_as_zero(driver: DMTPDriver, verbose: bool = True) -> bool:
#     """
#     将电机当前位置设置为零点

#     Args:
#         driver: DMTPDriver实例
#         verbose: 是否打印详细信息

#     Returns:
#         bool: 成功返回True，失败返回False
#     """
#     if verbose:
#         print("=" * 50)
#         print("设置当前为零点")
#         print("=" * 50)

#     # 1. 使能电机
#     if verbose:
#         print("[1] 使能电机...")
#     driver.enable()
#     time.sleep(0.5)

#     status = driver.get_status()
#     if verbose and status:
#         print(f"    电机状态: {status['running_state_name']}")

#     # 2. 获取当前位置
#     current_pos = driver.get_position()
#     if current_pos is None:
#         if verbose:
#             print("  错误: 无法读取当前位置")
#         return False

#     if verbose:
#         print(f"[2] 当前位置: {current_pos} pulses")

#     # 3. 设置当前位置为零点
#     if verbose:
#         print("[3] 设置当前位置为零点...")
#     success = driver.set_current_position(0)
#     if not success:
#         if verbose:
#             print("  错误: 设置零点失败")
#         return False

#     time.sleep(0.2)
#     new_pos = driver.get_position()

#     if verbose:
#         print(f"    零点设置完成，当前位置: {new_pos} pulses")
#         print("=" * 50)

#     return True


# def move_to_position(driver: DMTPDriver,
#                      target_position: int,
#                      speed_rpm: int = 80,
#                      accel_time_ms: int = 200,
#                      decel_time_ms: int = 200,
#                      start_speed_rpm: int = 30,
#                      wait_timeout: float = 5,
#                      tolerance: int = 50,
#                      stop_after_reach: bool = False,
#                      verbose: bool = True) -> bool:
#     """
#     控制电机运动到指定绝对位置

#     Args:
#         driver: DMTPDriver实例
#         target_position: 目标位置（脉冲数，支持负数）
#         speed_rpm: 运行速度 (rpm)
#         accel_time_ms: 加速时间 (ms)
#         decel_time_ms: 减速时间 (ms)
#         start_speed_rpm: 启动速度 (rpm)
#         wait_timeout: 等待运动完成的超时时间 (秒)
#         tolerance: 位置到达容差（脉冲）
#         stop_after_reach: 到达后是否停止电机（True: 发送停止命令，False: 到达后自然停止）
#         verbose: 是否打印详细信息

#     Returns:
#         bool: 成功到达目标位置返回True，失败返回False
#     """
#     if verbose:
#         print("=" * 50)
#         print("运动到目标位置")
#         print("=" * 50)
#         print(f"目标位置: {target_position} pulses")
#         print(f"速度: {speed_rpm} rpm")
#         print(f"加速时间: {accel_time_ms} ms")
#         print(f"减速时间: {decel_time_ms} ms")
#         print(f"启动速度: {start_speed_rpm} rpm")

#     # 1. 设置运动参数
#     driver.set_acceleration(accel_time_ms)
#     driver.set_deceleration(decel_time_ms)
#     driver.set_start_speed(start_speed_rpm)
#     driver.set_run_speed(speed_rpm)

#     # 2. 确保电机使能
#     driver.enable()
#     time.sleep(0.05)

#     # 3. 启动运动
#     if verbose:
#         print("启动运动...")
#     success = driver.move_to_absolute(target_position)
#     if not success:
#         if verbose:
#             print("  错误: 启动运动失败")
#         return False

#     # 4. 等待运动完成
#     if verbose:
#         print("等待运动完成...")

#     position_reached = driver.wait_for_position_stable(
#         target_position,
#         timeout=wait_timeout,
#         tolerance=tolerance
#     )

#     if not position_reached:
#         if verbose:
#             final_pos = driver.get_position()
#             print(f"  当前位置: {final_pos} pulses")
#         # return True

#     if verbose:
#         final_pos = driver.get_position()
#         print(f"运动完成! 最终位置: {final_pos} pulses")

#     # 5. 可选：停止电机
#     if stop_after_reach:
#         if verbose:
#             print("停止电机...")
#         driver.stop_motor(emergency=False)
#         time.sleep(0.1)

#     if verbose:
#         print("=" * 50)

#     return True


# def move_round_trip(driver: DMTPDriver,
#                     target_pulses: int,
#                     speed_rpm: int = 60,
#                     delay_sec: float = 2.0,
#                     accel_time_ms: int = 200,
#                     decel_time_ms: int = 200,
#                     start_speed_rpm: int = 10,
#                     keep_enabled: bool = True,
#                     verbose: bool = True) -> bool:
#     """
#     往返运动：设置零点 -> 运动到目标位置 -> 停留 -> 返回零点

#     Args:
#         driver: DMTPDriver实例
#         target_pulses: 目标位置脉冲数（支持负数）
#         speed_rpm: 运行速度 (rpm)
#         delay_sec: 在目标位置的停留时间 (秒)
#         accel_time_ms: 加速时间 (ms)
#         decel_time_ms: 减速时间 (ms)
#         start_speed_rpm: 启动速度 (rpm)
#         keep_enabled: 运动完成后是否保持使能状态
#         verbose: 是否打印详细信息

#     Returns:
#         bool: 成功返回True，失败返回False
#     """
#     if verbose:
#         print("=" * 60)
#         print("往返运动控制")
#         print("=" * 60)
#         print(f"目标位置: {target_pulses} pulses ({target_pulses / driver.pulses_per_rev:.2f} 转)")
#         print(f"速度: {speed_rpm} rpm, 停留时间: {delay_sec} 秒")

#     # 1. 设置当前位置为零点
#     if not set_current_as_zero(driver, verbose):
#         if verbose:
#             print("设置零点失败")
#         return False

#     # 2. 正向运动到目标位置
#     if verbose:
#         print("\n正向运动到目标位置...")
#     success = move_to_position(
#         driver=driver,
#         target_position=target_pulses,
#         speed_rpm=speed_rpm,
#         accel_time_ms=accel_time_ms,
#         decel_time_ms=decel_time_ms,
#         start_speed_rpm=start_speed_rpm,
#         wait_timeout=30,
#         tolerance=20,
#         stop_after_reach=False,
#         verbose=verbose
#     )
#     if not success:
#         if verbose:
#             print("正向运动失败")
#         return False

#     # 3. 停留
#     if verbose:
#         print(f"\n停留 {delay_sec} 秒...")
#     time.sleep(delay_sec)

#     # 4. 反向运动回零点
#     if verbose:
#         print("\n反向运动回零点...")
#     success = move_to_position(
#         driver=driver,
#         target_position=0,
#         speed_rpm=speed_rpm,
#         accel_time_ms=accel_time_ms,
#         decel_time_ms=decel_time_ms,
#         start_speed_rpm=start_speed_rpm,
#         wait_timeout=30,
#         tolerance=20,
#         stop_after_reach=False,
#         verbose=verbose
#     )
#     if not success:
#         if verbose:
#             print("反向运动失败")
#         if keep_enabled:
#             if verbose:
#                 print("电机保持使能，请检查机械系统")
#         else:
#             driver.disable()
#         return False

#     # 5. 完成后处理
#     if verbose:
#         print("\n运动完成后的处理...")
#     if keep_enabled:
#         if verbose:
#             print("电机保持使能状态，当前位置锁定")
#         driver.stop_motor(emergency=False)
#         time.sleep(0.1)
#     else:
#         if verbose:
#             print("电机已禁用")
#         driver.disable()

#     final_pos = driver.get_position()
#     if verbose:
#         print(f"最终位置: {final_pos} pulses")
#         print("=" * 60)
#         print("往返运动完成!")
#         print("=" * 60)

#     return True


# def initialize_driver(port: str = 'COM11', node_id: int = 1,
#                       baudrate: int = 115200, verbose: bool = True) -> Optional[DMTPDriver]:
#     """
#     初始化驱动器并执行初始设置

#     Args:
#         port: 串口号
#         node_id: 节点ID
#         baudrate: 波特率
#         verbose: 是否打印详细信息

#     Returns:
#         DMTPDriver实例，初始化失败返回None
#     """
#     try:
#         # 创建驱动器实例
#         driver = DMTPDriver(port, node_id, baudrate, debug=not verbose)

#         # 测试连接
#         pos = driver.get_position()
#         if pos is not None:
#             if verbose:
#                 print(f"✓ 连接成功! 当前位置: {pos} pulses")
#         else:
#             if verbose:
#                 print("✗ 连接失败!")
#             driver.close()
#             return None

#         # 执行初始运动
#         if verbose:
#             print("\n执行初始运动...")
#         success = move_to_position(
#             driver=driver,
#             target_position=-12,
#             speed_rpm=100,
#             accel_time_ms=100,
#             decel_time_ms=200,
#             wait_timeout=1,
#             verbose=verbose
#         )

#         if not success and verbose:
#             print("⚠ 初始运动失败，但继续执行零点设置...")

#         # 设置当前位置为零点
#         if verbose:
#             print("\n设置当前位置为零点")
#         set_current_as_zero(driver, verbose=verbose)

#         if verbose:
#             print("✓ 驱动器初始化完成!")

#         return driver

#     except serial.SerialException as e:
#         if verbose:
#             print(f"串口错误: {e}")
#         return None
#     except Exception as e:
#         if verbose:
#             print(f"初始化错误: {e}")
#         return None

# # ==================== 示例使用 ====================

# def main():
#     # 初始化驱动器
#     driver = initialize_driver(port='/dev/ttyACM0', verbose=True)

#     try:
#         if driver:
#             while True:
#                 # 运动到指定位置
#                 move_to_position(
#                     driver=driver,
#                     target_position=11,
#                     speed_rpm=100,
#                     accel_time_ms=100,
#                     decel_time_ms=200,
#                     wait_timeout=5,
#                     verbose=True
#                 )

#                 # 运动回零点
#                 move_to_position(
#                     driver=driver,
#                     target_position=0,
#                     speed_rpm=100,
#                     accel_time_ms=100,
#                     decel_time_ms=200,
#                     wait_timeout=3,
#                     verbose=True
#                 )
#     finally:
#         # 程序退出时关闭串口
#         if driver:
#             driver.close()

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
步进电机驱动器控制 - 封装函数
提供两个核心功能：设置零点、运动到指定位置
"""
import struct
import serial
import time
import threading
from typing import Optional, Dict, Any
from enum import IntEnum
import math


class RegisterAddress(IntEnum):
    """寄存器地址定义"""
    HARDWARE_VERSION = 0x0000
    SOFTWARE_VERSION = 0x0002
    MOTOR_POSITION = 0x0004
    STATUS_REGISTER = 0x0006

    UART_TIMEOUT = 0x0008
    BAUD_RATE = 0x0009
    SMOOTH_CONSTANT = 0x000A
    DYNAMIC_ERROR_THRESHOLD = 0x000B
    STATIC_ERROR_THRESHOLD = 0x000C
    RATED_CURRENT = 0x000D
    IDLE_CURRENT_PERCENT = 0x000E
    ENCODER_LINES = 0x000F
    POSITION_WARNING = 0x0010
    ACTUAL_POSITION_ERROR = 0x0011
    ALARM_CONFIG = 0x0012
    REAL_TIME_SPEED = 0x0019
    REAL_TIME_CURRENT = 0x001A

    # 运动参数
    START_SPEED = 0x0096
    STOP_SPEED = 0x0097
    ACCEL_TIME = 0x0098
    DECEL_TIME = 0x0099
    RUN_SPEED = 0x009A

    # 运行指令
    RUN_STOP = 0x00C8
    DISABLE_ENABLE = 0x00D4
    MOTOR_RUN_TO_ABS = 0x00D0
    SET_CURRENT_POSITION = 0x00D2
    SAVE_COMMAND = 0x00DC


class MotorDirection(IntEnum):
    CW = 0
    CCW = 1


class DMTPDriver:
    """步进电机驱动器控制类"""

    U16 = struct.Struct('<H')
    U32 = struct.Struct('<I')

    def __init__(self, port: str = 'COM5', node_id: int = 1,
                 baudrate: int = 115200, timeout: float = 0.5, debug: bool = False):
        self.node = node_id
        self.timeout = timeout
        self.debug = debug
        self.pulses_per_rev = 4000

        import serial
        self.ser = serial.Serial(port, baudrate,
                                 bytesize=8, parity='N', stopbits=1,
                                 timeout=timeout)
        if not self.ser.is_open:
            raise IOError(f'无法打开串口 {port}')

        self._lock = threading.Lock()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    @staticmethod
    def _crc16_modbus(data: bytes) -> int:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc = crc >> 1
        return crc

    def _read_register(self, address: int) -> Optional[int]:
        """读取单个寄存器 (16位)"""
        with self._lock:
            frame = bytes([self.node, 0x03, address >> 8, address & 0xFF, 0x00, 0x01])
            crc = self._crc16_modbus(frame)
            frame += self.U16.pack(crc)

            self.ser.reset_input_buffer()
            self.ser.write(frame)
            self.ser.flush()

            time.sleep(0.05)
            rx_data = self.ser.read_all()

            if not rx_data or len(rx_data) < 5:
                return None
            if rx_data[0] != self.node or rx_data[1] != 0x03:
                return None

            full_frame = rx_data[:-2] if len(rx_data) >= 2 else rx_data
            received_crc = self.U16.unpack(rx_data[-2:])[0]
            calculated_crc = self._crc16_modbus(full_frame)

            if received_crc != calculated_crc:
                return None

            return self.U16.unpack(rx_data[3:5])[0]

    def _write_register(self, address: int, value: int) -> bool:
        """写入单个寄存器 (16位)"""
        with self._lock:
            frame = bytes([self.node, 0x06, address >> 8, address & 0xFF,
                           (value >> 8) & 0xFF, value & 0xFF])
            crc = self._crc16_modbus(frame)
            frame += self.U16.pack(crc)

            self.ser.reset_input_buffer()
            self.ser.write(frame)
            self.ser.flush()

            time.sleep(0.05)
            rx_data = self.ser.read_all()

            if not rx_data or len(rx_data) < 8:
                return False
            if rx_data[0] != self.node or rx_data[1] != 0x06:
                return False

            full_frame = rx_data[:-2] if len(rx_data) >= 2 else rx_data
            received_crc = self.U16.unpack(rx_data[-2:])[0]
            calculated_crc = self._crc16_modbus(full_frame)

            return received_crc == calculated_crc

    def _read_dword(self, address: int) -> Optional[int]:
        """读取双字寄存器 (32位)，返回有符号整数"""
        with self._lock:
            frame = bytes([self.node, 0x03, address >> 8, address & 0xFF, 0x00, 0x02])
            crc = self._crc16_modbus(frame)
            frame += self.U16.pack(crc)

            self.ser.reset_input_buffer()
            self.ser.write(frame)
            self.ser.flush()

            time.sleep(0.05)
            rx_data = self.ser.read_all()

            if not rx_data or len(rx_data) < 7:
                return None
            if rx_data[0] != self.node or rx_data[1] != 0x03:
                return None

            full_frame = rx_data[:-2] if len(rx_data) >= 2 else rx_data
            received_crc = self.U16.unpack(rx_data[-2:])[0]
            calculated_crc = self._crc16_modbus(full_frame)

            if received_crc != calculated_crc:
                return None

            data_len = rx_data[2]
            if data_len >= 4:
                value = self.U32.unpack(rx_data[3:7])[0]
                # 将无符号值转换为有符号整数 (32位补码)
                if value >= 0x80000000:
                    value = value - 0x100000000
                return value
            return None

    def _write_dword(self, address: int, value: int) -> bool:
        """写入双字寄存器 (32位)，支持有符号整数"""
        with self._lock:
            # 将有符号整数转换为无符号整数表示（32位补码）
            if value < 0:
                value = value + (1 << 32)

            data = self.U32.pack(value)
            frame = bytes([self.node, 0x10, address >> 8, address & 0xFF, 0x00, 0x02, 0x04])
            frame += data
            crc = self._crc16_modbus(frame)
            frame += self.U16.pack(crc)

            self.ser.reset_input_buffer()
            self.ser.write(frame)
            self.ser.flush()

            time.sleep(0.05)
            rx_data = self.ser.read_all()

            if not rx_data or len(rx_data) < 8:
                return False
            if rx_data[0] != self.node or rx_data[1] != 0x10:
                return False

            full_frame = rx_data[:-2] if len(rx_data) >= 2 else rx_data
            received_crc = self.U16.unpack(rx_data[-2:])[0]
            calculated_crc = self._crc16_modbus(full_frame)

            return received_crc == calculated_crc

    # ==================== 主要接口 ====================

    def get_position(self) -> Optional[int]:
        """获取电机实时位置 (脉冲数，有符号整数)"""
        return self._read_dword(RegisterAddress.MOTOR_POSITION)

    def get_speed(self) -> Optional[int]:
        """获取实时速度 (rpm)"""
        value = self._read_register(RegisterAddress.REAL_TIME_SPEED)
        if value is not None:
            if value >= 0x8000:
                return value - 0x10000
            return value
        return None

    def get_status(self) -> Optional[Dict[str, Any]]:
        """获取状态寄存器"""
        value = self._read_register(RegisterAddress.STATUS_REGISTER)
        if value is None:
            return None

        running_state = (value >> 8) & 0x03
        running_names = {0: "空闲", 1: "启动中", 2: "停止中", 3: "运行中"}

        return {
            'raw': value,
            'running_state': running_state,
            'running_state_name': running_names.get(running_state, "未知"),
            'running': running_state == 3,
            'position_reached': bool(value & 0x1000),
            'x0_input': bool(value & 0x0001),
            'x1_input': bool(value & 0x0002),
            'x2_input': bool(value & 0x0004),
            'x3_input': bool(value & 0x0008),
            'x4_input': bool(value & 0x0010),
            'x5_input': bool(value & 0x0020),
            'x6_input': bool(value & 0x0040),
            'x7_input': bool(value & 0x0080),
        }

    def set_run_speed(self, speed_rpm: int) -> bool:
        """设置运行速度 (rpm)"""
        if speed_rpm < 0 or speed_rpm > 10000:
            raise ValueError(f"速度范围: 0-10000 rpm")
        return self._write_register(RegisterAddress.RUN_SPEED, speed_rpm)

    def set_acceleration(self, accel_time_ms: int) -> bool:
        """设置加速时间 (ms)"""
        if accel_time_ms < 0 or accel_time_ms > 65535:
            raise ValueError(f"加速时间范围: 0-65535 ms")
        return self._write_register(RegisterAddress.ACCEL_TIME, accel_time_ms)

    def set_deceleration(self, decel_time_ms: int) -> bool:
        """设置减速时间 (ms)"""
        if decel_time_ms < 0 or decel_time_ms > 65535:
            raise ValueError(f"减速时间范围: 0-65535 ms")
        return self._write_register(RegisterAddress.DECEL_TIME, decel_time_ms)

    def set_start_speed(self, speed_rpm: int) -> bool:
        """设置启动速度 (rpm)"""
        if speed_rpm < 0 or speed_rpm > 65535:
            raise ValueError(f"启动速度范围: 0-65535 rpm")
        return self._write_register(RegisterAddress.START_SPEED, speed_rpm)

    def enable(self) -> bool:
        """使能电机"""
        return self._write_register(RegisterAddress.DISABLE_ENABLE, 0)

    def disable(self) -> bool:
        """禁用电机"""
        return self._write_register(RegisterAddress.DISABLE_ENABLE, 1)

    def set_current_position(self, position_pulses: int) -> bool:
        """设置当前电机位置 (将当前位置设置为指定值)"""
        return self._write_dword(RegisterAddress.SET_CURRENT_POSITION, position_pulses)

    def move_to_absolute(self, position_pulses: int) -> bool:
        """移动到绝对位置"""
        return self._write_dword(RegisterAddress.MOTOR_RUN_TO_ABS, position_pulses)

    def stop_motor(self, emergency: bool = False) -> bool:
        """停止电机"""
        if emergency:
            return self._write_register(RegisterAddress.RUN_STOP, 256)
        else:
            return self._write_register(RegisterAddress.RUN_STOP, 0)

    def wait_for_stop(self, timeout: float = 10, interval: float = 0.1) -> bool:
        """等待电机停止"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            status = self.get_status()
            if status and not status.get('running', False):
                return True
            time.sleep(interval)
        return False

    def wait_for_position_reached(self, timeout: float = 10, interval: float = 0.05) -> bool:
        """等待位置到达"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            status = self.get_status()
            if status and status.get('position_reached', False):
                return True
            time.sleep(interval)
        return False

    def wait_for_position_stable(self, target_position: int,
                                 timeout: float = 30,
                                 tolerance: int = 10,
                                 stable_samples: int = 3) -> bool:
        """
        等待位置稳定到达目标位置

        Args:
            target_position: 目标位置
            timeout: 超时时间（秒）
            tolerance: 位置容差（脉冲）
            stable_samples: 需要稳定的采样次数
        """
        start_time = time.time()
        positions = []

        while time.time() - start_time < timeout:
            current_pos = self.get_position()
            if current_pos is None:
                time.sleep(0.05)
                continue

            if abs(current_pos - target_position) <= tolerance:
                positions.append(current_pos)

                if len(positions) > stable_samples:
                    positions.pop(0)

                if len(positions) >= stable_samples:
                    pos_range = max(positions) - min(positions)
                    if pos_range <= 5:
                        return True
            else:
                positions = []

            time.sleep(0.05)

        final_pos = self.get_position()
        if final_pos is not None and abs(final_pos - target_position) <= tolerance:
            return True

        return False


# ==================== 封装的两个核心函数 ====================

def set_current_as_zero(driver: DMTPDriver, verbose: bool = True) -> bool:
    """
    将电机当前位置设置为零点

    Args:
        driver: DMTPDriver实例
        verbose: 是否打印详细信息

    Returns:
        bool: 成功返回True，失败返回False
    """
    if verbose:
        print("=" * 50)
        print("设置当前为零点")
        print("=" * 50)

    # 1. 使能电机
    if verbose:
        print("[1] 使能电机...")
    driver.enable()
    time.sleep(0.5)

    status = driver.get_status()
    if verbose and status:
        print(f"    电机状态: {status['running_state_name']}")

    # 2. 获取当前位置
    current_pos = driver.get_position()
    if current_pos is None:
        if verbose:
            print("  错误: 无法读取当前位置")
        return False

    if verbose:
        print(f"[2] 当前位置: {current_pos} pulses")

    # 3. 设置当前位置为零点
    if verbose:
        print("[3] 设置当前位置为零点...")
    success = driver.set_current_position(0)
    if not success:
        if verbose:
            print("  错误: 设置零点失败")
        return False

    time.sleep(0.2)
    new_pos = driver.get_position()

    if verbose:
        print(f"    零点设置完成，当前位置: {new_pos} pulses")
        print("=" * 50)

    return True

last_pos = 0.0
def move_to_position(driver: DMTPDriver,
                     target_position: int,
                     speed_rpm: int = 80,
                     accel_time_ms: int = 200,
                     decel_time_ms: int = 200,
                     start_speed_rpm: int = 30,
                     wait_timeout: float = 5,
                     tolerance: int = 50,
                     stop_after_reach: bool = False,
                     verbose: bool = True) -> bool:
    """
    控制电机运动到指定绝对位置

    Args:
        driver: DMTPDriver实例
        target_position: 目标位置（脉冲数，支持负数）
        speed_rpm: 运行速度 (rpm)
        accel_time_ms: 加速时间 (ms)
        decel_time_ms: 减速时间 (ms)
        start_speed_rpm: 启动速度 (rpm)
        wait_timeout: 等待运动完成的超时时间 (秒)
        tolerance: 位置到达容差（脉冲）
        stop_after_reach: 到达后是否停止电机（True: 发送停止命令，False: 到达后自然停止）
        verbose: 是否打印详细信息

    Returns:
        bool: 成功到达目标位置返回True，失败返回False
    """
    global last_pos
    if target_position == 0:
        pos = last_pos
    else:
        pos = target_position

    # 1. 设置运动参数
    driver.set_acceleration(accel_time_ms)
    driver.set_deceleration(decel_time_ms)
    driver.set_start_speed(start_speed_rpm)
    driver.set_run_speed(speed_rpm)

    # 2. 确保电机使能
    driver.enable()
    time.sleep(0.05)

    # 3. 启动运动
    if verbose:
        print("启动运动...")
    success = driver.move_to_absolute(target_position)
    if not success:
        if verbose:
            print("  错误: 启动运动失败")
        return False

    # 4. 等待运动完成
    if verbose:
        print("等待运动完成...")
    if target_position != 0:
        # 传进来的是非零目标：用这个目标 & 更新 last_pos
        pos = target_position
        last_pos = target_position
    else:
        # 传进来的是 0：复用上次的非零目标
        pos = last_pos

    last_pos = target_position
    position_reached = driver.wait_for_position_stable(
        target_position,
        timeout=math.fabs(pos)/speed_rpm * 15,
        tolerance=tolerance
    )

    if not position_reached:
        if verbose:
            final_pos = driver.get_position()
            print(f"  当前位置: {final_pos} pulses")
        # return True

    if verbose:
        final_pos = driver.get_position()
        print(f"运动完成! 最终位置: {final_pos} pulses")

    # 5. 可选：停止电机
    if stop_after_reach:
        if verbose:
            print("停止电机...")
        driver.stop_motor(emergency=False)
        time.sleep(0.1)

    if verbose:
        print("=" * 50)

    return True


def move_round_trip(driver: DMTPDriver,
                    target_pulses: int,
                    speed_rpm: int = 60,
                    delay_sec: float = 2.0,
                    accel_time_ms: int = 200,
                    decel_time_ms: int = 200,
                    start_speed_rpm: int = 10,
                    keep_enabled: bool = True,
                    verbose: bool = True) -> bool:
    """
    往返运动：设置零点 -> 运动到目标位置 -> 停留 -> 返回零点

    Args:
        driver: DMTPDriver实例
        target_pulses: 目标位置脉冲数（支持负数）
        speed_rpm: 运行速度 (rpm)
        delay_sec: 在目标位置的停留时间 (秒)
        accel_time_ms: 加速时间 (ms)
        decel_time_ms: 减速时间 (ms)
        start_speed_rpm: 启动速度 (rpm)
        keep_enabled: 运动完成后是否保持使能状态
        verbose: 是否打印详细信息

    Returns:
        bool: 成功返回True，失败返回False
    """
    if verbose:
        print("=" * 60)
        print("往返运动控制")
        print("=" * 60)
        print(f"目标位置: {target_pulses} pulses ({target_pulses / driver.pulses_per_rev:.2f} 转)")
        print(f"速度: {speed_rpm} rpm, 停留时间: {delay_sec} 秒")

    # 1. 设置当前位置为零点
    if not set_current_as_zero(driver, verbose):
        if verbose:
            print("设置零点失败")
        return False

    # 2. 正向运动到目标位置
    if verbose:
        print("\n正向运动到目标位置...")
    success = move_to_position(
        driver=driver,
        target_position=target_pulses,
        speed_rpm=speed_rpm,
        accel_time_ms=accel_time_ms,
        decel_time_ms=decel_time_ms,
        start_speed_rpm=start_speed_rpm,
        wait_timeout=30,
        tolerance=20,
        stop_after_reach=False,
        verbose=verbose
    )
    if not success:
        if verbose:
            print("正向运动失败")
        return False

    # 3. 停留
    if verbose:
        print(f"\n停留 {delay_sec} 秒...")
    time.sleep(delay_sec)

    # 4. 反向运动回零点
    if verbose:
        print("\n反向运动回零点...")
    success = move_to_position(
        driver=driver,
        target_position=0,
        speed_rpm=speed_rpm,
        accel_time_ms=accel_time_ms,
        decel_time_ms=decel_time_ms,
        start_speed_rpm=start_speed_rpm,
        wait_timeout=30,
        tolerance=20,
        stop_after_reach=False,
        verbose=verbose
    )
    if not success:
        if verbose:
            print("反向运动失败")
        if keep_enabled:
            if verbose:
                print("电机保持使能，请检查机械系统")
        else:
            driver.disable()
        return False

    # 5. 完成后处理
    if verbose:
        print("\n运动完成后的处理...")
    if keep_enabled:
        if verbose:
            print("电机保持使能状态，当前位置锁定")
        driver.stop_motor(emergency=False)
        time.sleep(0.1)
    else:
        if verbose:
            print("电机已禁用")
        driver.disable()

    final_pos = driver.get_position()
    if verbose:
        print(f"最终位置: {final_pos} pulses")
        print("=" * 60)
        print("往返运动完成!")
        print("=" * 60)

    return True


def clear_alarm(driver: DMTPDriver, verbose: bool = True) -> bool:
    """
    清除驱动器报警状态

    Args:
        driver: DMTPDriver实例
        verbose: 是否打印详细信息

    Returns:
        bool: 成功清除返回True，失败返回False

    说明:
        清除报警状态包括:
        - 电机相位过流
        - 供电电压过高/过低
        - 电机A/B相开路
        - 位置超差
        - 编码器错误等
    """
    if verbose:
        print("=" * 50)
        print("清除报警状态")
        print("=" * 50)

    # 读取当前报警状态
    alarm_status = driver._read_register(RegisterAddress.ALARM_CONFIG)
    if alarm_status is not None and verbose:
        print(f"清除前报警状态: 0x{alarm_status:04X}")

        # 解析报警状态位 (根据手册第26页)
        alarm_bits = {
            0: "电机相位过流",
            1: "供电电压过高",
            2: "供电电压过低",
            3: "电机A相开路",
            4: "电机B相开路",
            5: "其他报警或位置超差",
            6: "内部24V电压偏移",
            7: "AI电压错误",
            8: "BI电压错误",
            9: "编码器错误"
        }

        active_alarms = []
        for bit, name in alarm_bits.items():
            if alarm_status & (1 << bit):
                active_alarms.append(name)

        if active_alarms:
            print(f"当前报警: {', '.join(active_alarms)}")
        else:
            print("无报警状态")

    # 写入任意值清除报警 (根据手册，写入0x0001清除报警)
    success = driver._write_register(RegisterAddress.ALARM_CONFIG, 0x0001)

    if verbose:
        if success:
            # 验证清除结果
            new_status = driver._read_register(RegisterAddress.ALARM_CONFIG)
            if new_status is not None:
                print(f"清除后报警状态: 0x{new_status:04X}")
                if new_status == 0:
                    print("✓ 报警状态已清除")
                else:
                    print(f"⚠ 仍有报警: 0x{new_status:04X}")
            else:
                print("✓ 报警清除指令已发送")
        else:
            print("✗ 清除报警失败")
        print("=" * 50)

    return success

def initialize_driver(port: str = 'COM11', node_id: int = 1,
                      baudrate: int = 115200, verbose: bool = True) -> Optional[DMTPDriver]:
    """
    初始化驱动器并执行初始设置
    流程：清除警告 -> 运动到机械零点 -> 设置当前位置为零点
    """
    try:
        # 创建驱动器实例
        driver = DMTPDriver(port, node_id, baudrate, debug=not verbose)

        # 测试连接
        pos = driver.get_position()
        if pos is not None:
            if verbose:
                print(f"✓ 连接成功! 当前位置: {pos} pulses")
        else:
            if verbose:
                print("✗ 连接失败!")
            driver.close()
            return None

        # ========== 步骤1: 清除所有警告 ==========
        if verbose:
            print("\n" + "=" * 50)
            print("步骤1: 清除驱动器警告")
            print("=" * 50)

        # 读取报警状态（只读寄存器 0x00A3）
        alarm_status = driver._read_register(0x00A3)
        if alarm_status is not None:
            if verbose:
                print(f"清除前报警状态: 0x{alarm_status:04X}")

                # 解析报警类型
                if alarm_status != 0:
                    alarm_bits = {
                        0: "电机相位过流", 1: "供电电压过高", 2: "供电电压过低",
                        3: "电机A相开路", 4: "电机B相开路", 5: "位置超差",
                        6: "内部24V电压偏移", 7: "AI电压错误", 8: "BI电压错误", 9: "编码器错误"
                    }
                    active_alarms = []
                    for bit, name in alarm_bits.items():
                        if alarm_status & (1 << bit):
                            active_alarms.append(name)
                    if active_alarms:
                        print(f"报警详情: {', '.join(active_alarms)}")
                else:
                    print("无报警状态")

        # 清除报警（写入寄存器 0x00A4，值为 0x0000）
        if verbose:
            print("正在清除报警...")
        driver._write_register(0x00A4, 0x0000)
        time.sleep(0.2)

        # 验证清除结果（再次读取报警状态 0x00A3）
        new_alarm = driver._read_register(0x00A3)
        if verbose:
            if new_alarm == 0:
                print("✓ 报警已清除")
            else:
                print(f"⚠ 报警清除后状态: 0x{new_alarm:04X}")
                # 如果还有报警，再尝试清除一次
                if new_alarm != 0:
                    if verbose:
                        print("尝试再次清除报警...")
                    driver._write_register(0x00A4, 0x0000)
                    time.sleep(0.2)
                    final_alarm = driver._read_register(0x00A3)
                    if final_alarm == 0:
                        print("✓ 第二次清除成功")
                    else:
                        print(f"⚠ 清除后仍有报警: 0x{final_alarm:04X}")

        # ========== 步骤2: 运动到机械零点 ==========
        move_to_position(
            driver=driver,
            target_position=-12,
            speed_rpm=100,
            accel_time_ms=100,
            decel_time_ms=200,
            wait_timeout=5,
            verbose=True
        )
        set_current_as_zero(driver, verbose=verbose)

        return driver

    except serial.SerialException as e:
        if verbose:
            print(f"串口错误: {e}")
        return None
    except Exception as e:
        if verbose:
            print(f"初始化错误: {e}")
            import traceback
            traceback.print_exc()
        return None
# ==================== 示例使用 ====================

def main():
    # 初始化驱动器
    driver = initialize_driver(port='COM11', verbose=True)

    try:
        if driver:
            while True:
                # 运动到指定位置
                move_to_position(
                    driver=driver,
                    target_position=11,
                    speed_rpm=60,
                    accel_time_ms=100,
                    decel_time_ms=200,
                    verbose=True
                )

                # 运动回零点
                move_to_position(
                    driver=driver,
                    target_position=0,
                    speed_rpm=100,
                    accel_time_ms=100,
                    decel_time_ms=200,
                    verbose=True
                )
    finally:
        # 程序退出时关闭串口
        if driver:
            driver.close()

if __name__ == '__main__':
    main()