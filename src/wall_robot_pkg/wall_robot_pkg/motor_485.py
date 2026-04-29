#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import struct
import time

class MD2202Controller:
    def __init__(self, port='/dev/ttyACM0', baudrate=9600):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.2)
            print(f"✅ 成功打开电机串口: {port} (波特率 {baudrate})")
        except Exception as e:
            print(f"❌ 打开电机串口失败: {e}")
            self.ser = None

    def _modbus_crc16(self, data: bytearray) -> bytes:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1: crc = (crc >> 1) ^ 0xA001
                else: crc >>= 1
        return struct.pack('<H', crc)

    def _send_cmd(self, payload_list: list, cmd_name: str=""):
        if self.ser is None: return
        self.ser.reset_input_buffer()
        data = bytearray(payload_list)
        data.extend(self._modbus_crc16(data)) 
        self.ser.write(data)
        self.ser.flush() 
        time.sleep(0.05) 
        if self.ser.in_waiting > 0: self.ser.read(self.ser.in_waiting)

    def set_param_m1(self):
        payload = [
            0x02, 0x10, 0x00, 0x10, 0x00, 0x10, 0x20, 0x00, 
            0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 
            0x00, 0x05, 0xdc, 0x00, 0x00, 0x00, 0x0A, 0x00, 
            0x01, 0x00, 0x00, 0x01, 0xF4, 0x03, 0xE8, 0x00, 
            0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x20
        ]
        self._send_cmd(payload)

    def set_param_m2(self):
        payload = [
            0x02, 0x10, 0x00, 0x30, 0x00, 0x10, 0x20, 0x00, 
            0x01, 0x00, 0x01, 0x00, 0x06, 0x00, 0x01, 0x00, 
            0x00, 0x05, 0xdc, 0x00, 0x00, 0x00, 0x0A, 0x00, 
            0x01, 0x00, 0x00, 0x01, 0xF4, 0x03, 0xE8, 0x00, 
            0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x20
        ]
        self._send_cmd(payload)

    def reset_m1(self): self._send_cmd([0x02, 0x06, 0x00, 0x25, 0x00, 0x01])
    def reset_m2(self): self._send_cmd([0x02, 0x06, 0x00, 0x45, 0x00, 0x01])

    def set_pos_m1(self, pulse: int):
        high_b = (pulse >> 8) & 0xFF
        low_b  = pulse & 0xFF
        self._send_cmd([0x02, 0x10, 0x00, 0x22, 0x00, 0x02, 0x04, 0x00, 0x00, high_b, low_b])

    def set_pos_m2(self, pulse: int):
        high_b = (pulse >> 8) & 0xFF
        low_b  = pulse & 0xFF
        self._send_cmd([0x02, 0x10, 0x00, 0x42, 0x00, 0x02, 0x04, 0x00, 0x00, high_b, low_b])

    def close(self):
        if self.ser: self.ser.close()


# ==========================================
# 供主节点调用的外部执行逻辑 (解耦)
# ==========================================
def scan_sequence(motor_sys, m1_pulse=1584, m2_pulse=1513):
    """执行标准的一整套扫查动作：下移->伸出->收回->上升"""
    if motor_sys is None or motor_sys.ser is None:
        print("❌ 串口未连接，无法执行扫查序列！")
        return
    
    print(f"🔄 启动标准探测序列 (M1: {m1_pulse}, M2: {m2_pulse})")
    try:
        motor_sys.set_pos_m1(m1_pulse)
        time.sleep(2.0)
        
        motor_sys.set_pos_m2(m2_pulse)
        time.sleep(2.5)
        
        motor_sys.set_pos_m2(0)
        time.sleep(2.5)
        
        motor_sys.set_pos_m1(0)
        time.sleep(2.0)
        print("✅ 扫查序列执行完毕")
    except Exception as e:
        print(f"❌ 序列执行中断: {e}")

# 兼容独立运行测试
if __name__ == '__main__':
    motor = MD2202Controller(port='/dev/ttyACM0', baudrate=9600)
    if motor.ser is not None:
        try:
            print("--- 初始化与寻零 ---")
            motor.reset_m1(); time.sleep(2.0)
            motor.reset_m2(); time.sleep(4.0)
            
            print("--- 开始循环测试 ---")
            while True:
                scan_sequence(motor, m1_pulse=200, m2_pulse=500)
                time.sleep(1.0)
        except KeyboardInterrupt: pass
        finally: motor.close()