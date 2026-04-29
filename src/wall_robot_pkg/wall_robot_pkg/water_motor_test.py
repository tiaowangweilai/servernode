# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import json
# import time
# import serial

# # 导入真实电机驱动
# try:
#     from .motor_485 import MD2202Controller
# except ImportError:
#     MD2202Controller = None

# class WaterMotorTestNode(Node):
#     def __init__(self):
#         super().__init__('water_motor_test_node')
        
#         # 1. 抽水电机 (STM32) 无线串口
#         # 协议格式: $PUMP,<PWM>\r\n
#         self.STM32_SERIAL_PORT = '/dev/ttyUSB_STM32' 
#         self.stm32_ser = None
#         self.init_stm32_serial()

#         # 2. M1 下压电机驱动
#         self.m1_motor = None
#         self.init_m1_motor()

#         # 3. 核心状态逻辑变量
#         self.coupling_in_progress = False
#         self.coupling_start_time = 0.0
#         self.extra_press_triggered = False

#         # 4. 订阅总线参数流
#         self.params_sub = self.create_subscription(String, '/mission/params', self.params_callback, 10)
        
#         # 5. 逻辑轮询定时器 (10Hz)
#         self.logic_timer = self.create_timer(0.1, self.timer_callback)
        
#         self.get_logger().info("🚀 [联动逻辑] water_motor_test 启动 (500->等5s->加300 模式)")

#     def init_stm32_serial(self):
#         try:
#             self.stm32_ser = serial.Serial(self.STM32_SERIAL_PORT, 115200, timeout=0.1)
#             self.get_logger().info(f"✅ STM32 串口就绪: {self.STM32_SERIAL_PORT}")
#         except:
#             self.get_logger().warn("⚠️ STM32 串口未连接 (模拟中)")

#     def init_m1_motor(self):
#         if MD2202Controller:
#             try:
#                 self.m1_motor = MD2202Controller(port='/dev/ttyACM0', baudrate=115200)
#                 if self.m1_motor: self.m1_motor.set_param_m1()
#                 self.get_logger().info("✅ M1 物理驱动加载成功")
#             except: self.get_logger().error("❌ M1 串口占用或未发现")

#     def send_pump_pwm(self, pwm_val):
#         cmd = f"water{pwm_val}\r\n"
#         if self.stm32_ser and self.stm32_ser.is_open:
#             self.stm32_ser.write(cmd.encode())
#             self.get_logger().info(f"📡 串口发送 -> {repr(cmd)}")
#         else:
#             self.get_logger().info(f"💻 [模拟] 下发 PWM: {pwm_val}")

#     def params_callback(self, msg):
#         try:
#             data = json.loads(msg.data)[0]
#             command = data.get("data", {}).get("command", "")

#             if command == "coupling_failed":
#                 self.get_logger().warn("⚠️ 耦合失败 -> 启动【初步下压500 + 5s监控】")
#                 # 动作1: 初步下压
#                 if self.m1_motor: self.m1_motor.set_pos_m1(500)
#                 # 动作2: 大水注水
#                 self.send_pump_pwm(1000)
#                 # 启动计时逻辑
#                 self.coupling_in_progress = True
#                 self.extra_press_triggered = False
#                 self.coupling_start_time = time.time()

#             elif command == "coupling_success":
#                 self.get_logger().info("🟢 耦合成功 -> 切换【维持模式】")
#                 self.coupling_in_progress = False # 停止计时监控
#                 self.send_pump_pwm(200) # 水关小

#         except: pass

#     def timer_callback(self):
#         """ 核心监控：处理 5 秒无响应补偿逻辑 """
#         if self.coupling_in_progress and not self.extra_press_triggered:
#             elapsed = time.time() - self.coupling_start_time
#             if elapsed >= 10.0:
#                 self.get_logger().error("⏰ 5秒超时！未收到成功信号 -> 执行【二次补偿下压300】")
#                 # 动作：追加 300，即移动到绝对位置 800
#                 if self.m1_motor:
#                     try: self.m1_motor.set_pos_m1(800)
#                     except Exception as e: self.get_logger().error(f"M1 执行失败: {e}")
                
#                 self.extra_press_triggered = True
#                 # 注意：此时水泵依然保持 1000 大水量，直到收到 success 才会变小

# def main(args=None):
#     rclpy.init(args=args)
#     node = WaterMotorTestNode()
#     try: rclpy.spin(node)
#     except KeyboardInterrupt: pass
#     finally: node.destroy_node(); rclpy.shutdown()

# if __name__ == '__main__': main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import serial

# 导入真实电机驱动
try:
    from .motor_485 import MD2202Controller
except ImportError:
    MD2202Controller = None

class WaterMotorTestNode(Node):
    def __init__(self):
        super().__init__('water_motor_test_node')
        
        # 1. 抽水电机 (STM32) 无线串口
        # 🌟 端口请根据实际情况修改，例如 /dev/ttyACM1 或 /dev/ttyUSB0
        self.STM32_SERIAL_PORT = '/dev/ttyACM0' 
        self.stm32_ser = None
        self.init_stm32_serial()

        # 2. M1 下压电机驱动
        self.m1_motor = None
        self.init_m1_motor()

        # 3. 核心状态逻辑变量
        self.coupling_in_progress = False
        self.coupling_start_time = 0.0
        self.extra_press_triggered = False

        # 4. 订阅总线参数流
        self.params_sub = self.create_subscription(String, '/mission/params', self.params_callback, 10)
        
        # 5. 逻辑轮询定时器 (10Hz)
        self.logic_timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info("🚀 [联动逻辑] water_motor_test 启动 (支持收发双向协议)")

    def init_stm32_serial(self):
        try:
            # timeout=0.1 保证读取时不会死锁阻塞
            self.stm32_ser = serial.Serial(self.STM32_SERIAL_PORT, 115200, timeout=0.1)
            self.get_logger().info(f"✅ STM32 串口就绪: {self.STM32_SERIAL_PORT}")
        except:
            self.get_logger().warn(f"⚠️ STM32 串口 {self.STM32_SERIAL_PORT} 未连接 (模拟中)")

    def init_m1_motor(self):
        if MD2202Controller:
            try:
                # 假设 M1 电机在 ACM0
                self.m1_motor = MD2202Controller(port='/dev/ttyACM1', baudrate=115200)
                if self.m1_motor: self.m1_motor.set_param_m1()
                self.get_logger().info("✅ M1 物理驱动加载成功")
            except: self.get_logger().error("❌ M1 串口占用或未发现")

    def send_pump_pwm(self, pwm_val):
        """
        构造新协议：AA 55 + 'water' + PWM值 + \r\n
        """
        # 1. 帧头 0xAA 0x55
        header = bytes([0xAA, 0x55])
        # 2. 内容：water + 数值 + \r\n
        content = f"water{pwm_val}\r\n".encode('ascii')
        # 3. 拼接最终字节流
        full_packet = header + content

        if self.stm32_ser and self.stm32_ser.is_open:
            self.stm32_ser.write(full_packet)
            # 以十六进制格式打印日志，方便核对数据
            self.get_logger().info(f"📡 串口发送(Hex) -> {full_packet.hex(' ').upper()}")
        else:
            self.get_logger().info(f"💻 [模拟发送] 协议包 -> {full_packet.hex(' ').upper()}")

    def params_callback(self, msg):
        try:
            data_list = json.loads(msg.data)
            if not data_list: return
            item = data_list[0]
            command = item.get("data", {}).get("command", "")

            if command == "coupling_failed":
                self.get_logger().warn("🔴 耦合失败 -> 启动【初步下压500 + 5s监控】")
                # 动作1: 初步下压
                if self.m1_motor: self.m1_motor.set_pos_m1(500)
                
                # 动作2: 开启注水 (按之前逻辑给 1000)
                self.send_pump_pwm(90)
                
                # 启动计时逻辑
                self.coupling_in_progress = True
                self.extra_press_triggered = False
                self.coupling_start_time = time.time()

            elif command == "coupling_success":
                self.get_logger().info("🟢 耦合成功 -> 切换【维持模式】")
                self.coupling_in_progress = False 
                # 动作：将水关小 (给 200)
                self.send_pump_pwm(40)

        except Exception as e:
            self.get_logger().error(f"回调解析错误: {e}")

    def timer_callback(self):
        """ 10Hz 核心定时器：处理串口接收 与 超时补偿逻辑 """
        
        # ==============================================================
        # 🌟 1. 轮询读取 STM32 串口返回的数据 (非阻塞方式)
        # ==============================================================
        if self.stm32_ser and self.stm32_ser.is_open:
            try:
                # 只要缓存区里有数据，就一直读，直到读完
                while self.stm32_ser.in_waiting > 0:
                    # 读取一行数据并解码，去除首尾的 \r\n 等空白符
                    response = self.stm32_ser.readline().decode('utf-8', errors='ignore').strip()
                    if response:
                        self.get_logger().info(f"📥 [下位机回复] 收到 32 应答: {response}")
            except Exception as e:
                self.get_logger().error(f"读取 STM32 串口异常: {e}")
        # ==============================================================

        # 2. 处理 5 秒无响应补偿逻辑
        if self.coupling_in_progress and not self.extra_press_triggered:
            elapsed = time.time() - self.coupling_start_time
            # 严格按照要求：5秒未收到好指令则追加下压
            if elapsed >= 5.0:
                self.get_logger().error("⏰ 5秒超时！未收到成功信号 -> 执行【二次补偿下压300】")
                # 追加 300，到达绝对位置 800
                if self.m1_motor:
                    try: self.m1_motor.set_pos_m1(800)
                    except Exception as e: self.get_logger().error(f"M1 追加下压失败: {e}")
                
                self.extra_press_triggered = True

def main(args=None):
    rclpy.init(args=args)
    node = WaterMotorTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()