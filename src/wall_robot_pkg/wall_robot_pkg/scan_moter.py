# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import serial
# import time
# import sys
# import os

# # ❌ 删除那些错误的 src 导入
# # ❌ 删除那些错误的 src 导入

# # ==========================================
# # 🔥 核心修复：强制添加当前目录到系统路径
# # 解决 ROS 调用时找不到 bujinmotor 和 GPIO 的问题
# # ==========================================
# current_dir = os.path.dirname(os.path.abspath(__file__))
# if current_dir not in sys.path:
#     sys.path.append(current_dir)

# try:
#     # ✅ 正确的导入方式：依赖上面的 sys.path
#     from bujinmotor import *
#     from GPIO import * # 这句会把 push_rod_forward_time 等函数全部导入进来
#     print(f"✅ [scan_moter] 成功导入驱动，路径: {current_dir}")
# except ImportError as e:
#     print(f"❌ [scan_moter] 驱动导入失败: {e}")
#     print(f"请检查 {current_dir} 下是否有 bujinmotor.py 和 GPIO.py")

# # ==========================================

# class SerialControlSystem:
#     def __init__(self, port='/dev/ttyACM1', baudrate=115200, motor_port='/dev/ttyACM0', motor_baudrate=6000000, node_id=1):
#         self.port = port
#         self.baudrate = baudrate
#         self.motor_port = motor_port
#         self.motor_baudrate = motor_baudrate
#         self.node_id = node_id
#         self.ser = None

#     def connect(self):
#         """连接串口"""
#         try:
#             if self.ser and self.ser.is_open:
#                 return True
#             self.ser = serial.Serial(
#                 port=self.port,
#                 baudrate=self.baudrate,
#                 bytesize=8,
#                 parity='N',
#                 stopbits=1,
#                 timeout=2
#             )
#             time.sleep(0.1)
#             return self.ser.is_open
#         except Exception as e:
#             print(f"连接串口失败: {e}")
#             return False

#     def position_mode_test(self, value):
#         """封装调用底层函数"""
#         try:
#             # 强制转换为浮点数或整数，防止类型错误
#             val = float(value) 
#             position_mode_test(
#                 val,
#                 self.motor_port,
#                 self.node_id,
#                 self.motor_baudrate
#             )
#             return True
#         except Exception as e:
#             print(f"执行动作失败: {e}")
#             return False

#     def close(self):
#         if self.ser and self.ser.is_open:
#             try: self.ser.close()
#             except: pass

# def set_current_position_as_zero(port, node_id=1, baudrate=1000000):
#     try:
#         from bujinmotor import DMTPDriver
#         d = DMTPDriver(port, node_id, baudrate)
#         d.clear_error()
#         d._tx_rx(0x27, b'', expect_response=True)
#         d.close()
#         print("✓ 零点设置成功")
#     except:
#         print("⚠️ 零点设置跳过 (驱动未连接或错误)")

# def push_rod_backward_time_wrapper(run_time_ms):
#     try:
#         # 尝试获取全局 pusher 对象
#         import bujinmotor
#         if hasattr(bujinmotor, 'global_pusher'):
#             # 注意：这里假设 GPIO 里有 push_rod_backward_time
#             # 如果没有，可能需要检查 GPIO.py
#             push_rod_backward_time(run_time_ms, bujinmotor.global_pusher)
#         else:
#             pass 
#     except: pass

# # ==================== 核心执行函数 ====================
# def scan(distence, speed, control_system=None):
#     """
#     执行扫查动作
#     """
#     print(f"🔄 [Scan] 启动动作: 距离={distence}, 速度={speed}")
    
#     try:
#         dist = float(distence)
#         spd = float(speed)
#     except:
#         print("❌ 参数类型错误")
#         return

#     local_system = False
    
#     # 如果没传 hardware，自己初始化 (独立测试用)
#     if control_system is None:
#         try: gpio_init()
#         except: pass
#         control_system = SerialControlSystem(motor_port='/dev/ttyACM1', motor_baudrate=6000000, node_id=1)
#         local_system = True
#         set_current_position_as_zero('/dev/ttyACM1', 1, 6000000)
#         push_rod_backward_time_wrapper(1200)

#     # 检查连接
#     if control_system.ser is None or not control_system.ser.is_open:
#         if not control_system.connect():
#             print("❌ 串口未连接，无法执行动作")
#             return

#     # 执行动作序列
#     try:
#         mp = control_system.motor_port
#         nid = control_system.node_id
#         mb = control_system.motor_baudrate
        
#         # 动作 1: 伸出 -> 推杆进
#         print(f" -> 伸出 {dist}")
#         position_mode_test(dist, mp, nid, mb, spd)
#         push_rod_forward_time(250)

#         # 动作 2: 收回 -> 推杆进
#         print(f" -> 收回 0")
#         position_mode_test(0, mp, nid, mb, spd)
#         push_rod_forward_time(250)

#         # 动作 3: 伸出
#         print(f" -> 伸出 {dist}")
#         position_mode_test(dist, mp, nid, mb, spd)
#         push_rod_forward_time(250)

#         # 动作 4: 收回
#         print(f" -> 收回 1")
#         position_mode_test(0, mp, nid, mb, spd)
#         push_rod_forward_time(250)

#         # 动作 5: 伸出
#         print(f" -> 伸出 {dist}")
#         position_mode_test(dist, mp, nid, mb, spd)
#         push_rod_forward_time(250)

#         # 动作 6: 收回 -> 复位
#         print(f" -> 收回 0 (复位)")
#         position_mode_test(0, mp, nid, mb, spd)
#         push_rod_backward_time_wrapper(1200)
        
#         print("✅ [Scan] 动作序列完成")

#     except Exception as e:
#         print(f"❌ [Scan] 执行出错: {e}")
#         import traceback
#         traceback.print_exc()
#     finally:
#         if local_system:
#             control_system.close()
#             try: 
#                 push_rod_stop()
#                 image_motor_stop()
#                 GPIO.cleanup()
#             except: pass

# if __name__ == "__main__":
#     gpio_init()
#     d_val = 4.0
#     s_val = 3.0
#     if len(sys.argv) > 1: d_val = sys.argv[1]
#     if len(sys.argv) > 2: s_val = sys.argv[2]
    
#     print("🚀 正在运行独立测试模式...")
#     scan(d_val, s_val)


#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

# ==========================================
# 🔥 核心修复：安全导入，避免和 Jetson.GPIO 冲突
# ==========================================
try:
    import bujinmotor
    import GPIO as custom_gpio  # 给你的 GPIO.py 起个别名
    print(f"✅ [scan_moter] 成功导入驱动，路径: {current_dir}")
except ImportError as e:
    print(f"❌ [scan_moter] 驱动导入失败: {e}")
    print(f"请检查 {current_dir} 下是否有 bujinmotor.py 和 GPIO.py")

class SerialControlSystem:
    def __init__(self, port='/dev/ttyACM1', baudrate=115200, motor_port='/dev/ttyACM0', motor_baudrate=6000000, node_id=1):
        self.port = port
        self.baudrate = baudrate
        self.motor_port = motor_port
        self.motor_baudrate = motor_baudrate
        self.node_id = node_id
        self.ser = None

    def connect(self):
        try:
            if self.ser and self.ser.is_open:
                return True
            self.ser = serial.Serial(
                port=self.port, baudrate=self.baudrate, bytesize=8,
                parity='N', stopbits=1, timeout=2
            )
            time.sleep(0.1)
            return self.ser.is_open
        except Exception as e:
            print(f"连接串口失败: {e}")
            return False

    def position_mode_test(self, value):
        try:
            val = float(value) 
            bujinmotor.position_mode_test(val, self.motor_port, self.node_id, self.motor_baudrate)
            return True
        except Exception as e:
            print(f"执行动作失败: {e}")
            return False

    def close(self):
        if self.ser and self.ser.is_open:
            try: self.ser.close()
            except: pass

def set_current_position_as_zero(port, node_id=1, baudrate=1000000):
    try:
        from bujinmotor import DMTPDriver
        d = DMTPDriver(port, node_id, baudrate)
        d.clear_error()
        d._tx_rx(0x27, b'', expect_response=True)
        d.close()
        print("✓ 零点设置成功")
    except:
        print("⚠️ 零点设置跳过 (驱动未连接或错误)")


# ==================== 核心执行函数 ====================
def scan(distence, speed, control_system=None):
    """
    执行扫查动作
    """
    print(f"🔄 [Scan] 启动动作: 距离={distence}, 速度={speed}")
    
    try:
        dist = float(distence)
        spd = float(speed)
    except:
        print("❌ 参数类型错误")
        return

    local_system = False
    
    # 1. 独立测试环境初始化
    if control_system is None:
        try: 
            custom_gpio.gpio_init() 
        except Exception as e: 
            print(f"GPIO初始化失败: {e}")
            
        control_system = SerialControlSystem(motor_port='/dev/ttyACM1', motor_baudrate=6000000, node_id=1)
        local_system = True
        set_current_position_as_zero('/dev/ttyACM1', 1, 6000000)
        
        # 🔥 核心修复：直接调用正确的后退函数，抛弃有 Bug 的 wrapper
        print(" -> 执行初始后退动作...")
        custom_gpio.push_rod_backward(1200)

    # 2. 检查串口连接
    if control_system.ser is None or not control_system.ser.is_open:
        if not control_system.connect():
            print("❌ 串口未连接，无法执行动作")
            return

    # 3. 动作流执行
    try:
        mp = control_system.motor_port
        nid = control_system.node_id
        mb = control_system.motor_baudrate
        
        print(f" -> 伸出 {dist}")
        bujinmotor.position_mode_test(dist, mp, nid, mb, spd)
        custom_gpio.push_rod_forward_time(250)

        print(f" -> 收回 0")
        bujinmotor.position_mode_test(0, mp, nid, mb, spd)
        custom_gpio.push_rod_forward_time(250)

        print(f" -> 伸出 {dist}")
        bujinmotor.position_mode_test(dist, mp, nid, mb, spd)
        custom_gpio.push_rod_forward_time(250)

        print(f" -> 收回 1")
        bujinmotor.position_mode_test(0, mp, nid, mb, spd)
        custom_gpio.push_rod_forward_time(250)

        print(f" -> 伸出 {dist}")
        bujinmotor.position_mode_test(dist, mp, nid, mb, spd)
        custom_gpio.push_rod_forward_time(250)

        print(f" -> 收回 0 (复位)")
        bujinmotor.position_mode_test(0, mp, nid, mb, spd)
        custom_gpio.push_rod_backward(1200)
        
        print("✅ [Scan] 动作序列完成")

    except Exception as e:
        print(f"❌ [Scan] 执行出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理独立测试占用的资源
        if local_system:
            control_system.close()
            try: 
                custom_gpio.push_rod_stop()
                custom_gpio.image_motor_stop()
                custom_gpio.GPIO.cleanup()
            except: pass

def scanL(distence, speed, control_system=None):
    """
    执行扫查动作
    """
    print(f"🔄 [Scan] 启动动作: 距离={distence}, 速度={speed}")
    
    try:
        dist = float(distence)
        spd = float(speed)
    except:
        print("❌ 参数类型错误")
        return

    local_system = False
    
    # 1. 独立测试环境初始化
    if control_system is None:
        try: 
            custom_gpio.gpio_init() 
        except Exception as e: 
            print(f"GPIO初始化失败: {e}")
            
        control_system = SerialControlSystem(motor_port='/dev/ttyACM1', motor_baudrate=6000000, node_id=1)
        local_system = True
        set_current_position_as_zero('/dev/ttyACM1', 1, 6000000)
        
        # 🔥 核心修复：直接调用正确的后退函数，抛弃有 Bug 的 wrapper
        # print(" -> 执行初始后退动作...")
        # custom_gpio.push_rod_backward(1200)

    # 2. 检查串口连接
    if control_system.ser is None or not control_system.ser.is_open:
        if not control_system.connect():
            print("❌ 串口未连接，无法执行动作")
            return

    # 3. 动作流执行
    try:
        mp = control_system.motor_port
        nid = control_system.node_id
        mb = control_system.motor_baudrate
        
        print(f" -> 伸出 {dist}")
        bujinmotor.position_mode_test(dist, mp, nid, mb, spd)
        custom_gpio.image_forward(250)
        # custom_gpio.push_rod_forward_time(250)

        # print(f" -> 收回 0")
        # bujinmotor.position_mode_test(0, mp, nid, mb, spd)
        # custom_gpio.push_rod_forward_time(250)

        # print(f" -> 伸出 {dist}")
        # bujinmotor.position_mode_test(dist, mp, nid, mb, spd)
        # custom_gpio.push_rod_forward_time(250)

        # print(f" -> 收回 1")
        # bujinmotor.position_mode_test(0, mp, nid, mb, spd)
        # custom_gpio.push_rod_forward_time(250)

        # print(f" -> 伸出 {dist}")
        # bujinmotor.position_mode_test(dist, mp, nid, mb, spd)
        # custom_gpio.push_rod_forward_time(250)

        # print(f" -> 收回 0 (复位)")
        # bujinmotor.position_mode_test(0, mp, nid, mb, spd)
        # custom_gpio.push_rod_backward(1200)
        
        # print("✅ [Scan] 动作序列完成")

    except Exception as e:
        print(f"❌ [Scan] 执行出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理独立测试占用的资源
        if local_system:
            control_system.close()
            try: 
                custom_gpio.push_rod_stop()
                custom_gpio.image_motor_stop()
                custom_gpio.GPIO.cleanup()
            except: pass

def scanR(distence, speed, control_system=None):
    """
    执行扫查动作
    """
    print(f"🔄 [Scan] 启动动作: 距离={distence}, 速度={speed}")
    
    try:
        dist = float(distence)
        spd = float(speed)
    except:
        print("❌ 参数类型错误")
        return

    local_system = False
    
    # 1. 独立测试环境初始化
    if control_system is None:
        try: 
            custom_gpio.gpio_init() 
        except Exception as e: 
            print(f"GPIO初始化失败: {e}")
            
        control_system = SerialControlSystem(motor_port='/dev/ttyACM1', motor_baudrate=6000000, node_id=1)
        local_system = True
        # set_current_position_as_zero('/dev/ttyACM1', 1, 6000000)
        
        # 🔥 核心修复：直接调用正确的后退函数，抛弃有 Bug 的 wrapper
        # print(" -> 执行初始后退动作...")
        # custom_gpio.push_rod_backward(1200)

    # 2. 检查串口连接
    if control_system.ser is None or not control_system.ser.is_open:
        if not control_system.connect():
            print("❌ 串口未连接，无法执行动作")
            return

    # 3. 动作流执行
    try:
        mp = control_system.motor_port
        nid = control_system.node_id
        mb = control_system.motor_baudrate
        
        print(f" -> 伸出 {dist}")
        bujinmotor.position_mode_test(0, mp, nid, mb, spd)
        custom_gpio.image_forward(250)
        # custom_gpio.push_rod_forward_time(250)

        # print(f" -> 收回 0")
        # bujinmotor.position_mode_test(0, mp, nid, mb, spd)
        # custom_gpio.push_rod_forward_time(250)

        # print(f" -> 伸出 {dist}")
        # bujinmotor.position_mode_test(dist, mp, nid, mb, spd)
        # custom_gpio.push_rod_forward_time(250)

        # print(f" -> 收回 1")
        # bujinmotor.position_mode_test(0, mp, nid, mb, spd)
        # custom_gpio.push_rod_forward_time(250)

        # print(f" -> 伸出 {dist}")
        # bujinmotor.position_mode_test(dist, mp, nid, mb, spd)
        # custom_gpio.push_rod_forward_time(250)

        # print(f" -> 收回 0 (复位)")
        # bujinmotor.position_mode_test(0, mp, nid, mb, spd)
        # custom_gpio.push_rod_backward(1200)
        
        # print("✅ [Scan] 动作序列完成")

    except Exception as e:
        print(f"❌ [Scan] 执行出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理独立测试占用的资源
        if local_system:
            control_system.close()
            try: 
                custom_gpio.push_rod_stop()
                custom_gpio.image_motor_stop()
                custom_gpio.GPIO.cleanup()
            except: pass
# ============================ 主函数 ============================
if __name__ == "__main__":
    # 初始化
    custom_gpio.gpio_init()
    
    # 接收终端参数，比如执行: python3 scan_moter.py 4.0 3.0
    d_val = 4.0
    s_val = 3.0
    if len(sys.argv) > 1: d_val = sys.argv[1]
    if len(sys.argv) > 2: s_val = sys.argv[2]
    
    print("🚀 正在运行独立测试模式...")
    # 🔥 核心修复：把你刚才注释掉的代码解开了！
    scan(d_val, s_val)