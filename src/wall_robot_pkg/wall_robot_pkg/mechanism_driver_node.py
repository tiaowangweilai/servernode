#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import time

# === 导入底层硬件库 (只在这个节点里导入！) ===
try:
    from .GPIO import gpio_init, push_rod_forward_time
except ImportError:
    gpio_init, push_rod_forward_time = None, None

try:
    from .IG35 import initialize_driver as init_ig35, move_to_position as move_ig35
except ImportError:
    init_ig35, move_ig35 = None, None

try:
    from .motor_485 import MD2202Controller
except ImportError:
    MD2202Controller = None


class MechanismDriverNode(Node):
    def __init__(self):
        super().__init__('mechanism_driver_node')
        
        # 🌟 1. 订阅标准控制话题
        self.m1_sub = self.create_subscription(Int32, '/mech/m1_target', self.m1_callback, 10)
        self.m2_sub = self.create_subscription(Int32, '/mech/m2_target', self.m2_callback, 10)
        self.ig35_sub = self.create_subscription(Int32, '/mech/ig35_target', self.ig35_callback, 10)
        self.pushrod_sub = self.create_subscription(Int32, '/mech/push_rod_time', self.pushrod_callback, 10)
        
        # 🌟 2. 硬件统一初始化
        self.ig35_motor = None
        self.m1m2_motor = None
        self.init_motor_hardware()
        
        if gpio_init:
            try: 
                gpio_init()
                self.get_logger().info("✅ GPIO 推杆初始化成功")
            except Exception as e: 
                self.get_logger().warn(f"⚠️ GPIO 初始化失败: {e}")

        self.get_logger().info("⚙️ [机构驱动节点] 就绪，专职伺服底层硬件！")

    def init_motor_hardware(self):
        if init_ig35 is None or MD2202Controller is None: 
            return
        UNIFIED_BAUDRATE = 115200 
        SHARED_PORT = '/dev/ttyACM1' 
        try: 
            shared_ser = serial.Serial(SHARED_PORT, UNIFIED_BAUDRATE, timeout=0.2)
        except Exception as e: 
            self.get_logger().error(f"❌ 共享串口打开失败: {e}")
            return
            
        original_serial = serial.Serial
        serial.Serial = lambda *args, **kwargs: shared_ser
        try:
            self.ig35_motor = init_ig35(port=SHARED_PORT, baudrate=UNIFIED_BAUDRATE, verbose=False)
            self.m1m2_motor = MD2202Controller(port=SHARED_PORT, baudrate=UNIFIED_BAUDRATE)
            if self.m1m2_motor and self.m1m2_motor.ser:
                self.m1m2_motor.set_param_m1()
                self.m1m2_motor.set_param_m2()
                time.sleep(0.5)
                self.m1m2_motor.reset_m1()
                self.m1m2_motor.reset_m2()
            self.get_logger().info("✅ IG35 与 M1/M2 串口初始化成功")
        except Exception as e: 
            self.get_logger().error(f"电机初始化失败: {e}")
        finally: 
            serial.Serial = original_serial

    # ==========================================
    # 🌟 3. 回调执行函数 (执行具体物理动作)
    # ==========================================
    def m1_callback(self, msg):
        pulse = msg.data
        if self.m1m2_motor:
            self.m1m2_motor.set_pos_m1(pulse)
            self.get_logger().info(f"🔽 M1 执行目标脉冲: {pulse}")

    def m2_callback(self, msg):
        pulse = msg.data
        if self.m1m2_motor:
            self.m1m2_motor.set_pos_m2(pulse)
            self.get_logger().info(f"◀️ M2 执行目标脉冲: {pulse}")

    def ig35_callback(self, msg):
        target = msg.data
        if self.ig35_motor:
            self.get_logger().info(f"↔️ IG35 横杆开始移动至: {target}")
            try:
                # 这里的阻塞不再影响全车主控！
                move_ig35(driver=self.ig35_motor, target_position=target, speed_rpm=20, accel_time_ms=100, decel_time_ms=200, wait_timeout=3, verbose=False)
                self.get_logger().info(f"✅ IG35 到达目标: {target}")
            except Exception as e:
                self.get_logger().error(f"❌ IG35 移动异常: {e}")

    def pushrod_callback(self, msg):
        duration_ms = msg.data
        if push_rod_forward_time:
            self.get_logger().info(f"⚙️ 推杆动作触发，持续 {duration_ms} ms")
            push_rod_forward_time(duration_ms)

    def destroy_node(self):
        if self.ig35_motor: 
            self.ig35_motor.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MechanismDriverNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()