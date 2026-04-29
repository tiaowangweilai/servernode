#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import time
import os
import numpy as np
import json

# === 导入底层驱动 ===
try:
    from . import robot_motion
except ImportError:
    try:
        import robot_motion
    except ImportError:
        from . import robot_control as robot_motion

class ChassisDriverNode(Node):
    def __init__(self):
        super().__init__('chassis_driver_node')
        
        self.declare_parameter('serial_port', '/dev/ttyCH341USB0') 
        self.declare_parameter('baud_rate', 115200)
        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        
        self.status_pub = self.create_publisher(String, '/chassis/serial_status', 10)
        self.ser = None
        self.connect_serial()

        # --- 订阅者 ---
        # 手动控制依然保持 Twist
        self.sub_manual = self.create_subscription(Twist, '/cmd_vel_manual', self.manual_callback, 10)
        # 自动控制修改为 String，以接收完整的 JSON 导航指令
        self.sub_auto = self.create_subscription(String, '/cmd_vel_auto', self.auto_callback, 10)
        
        self.last_manual_ts = 0.0
        self.last_write_time = time.time()
        self.MIN_WRITE_INTERVAL = 0.008 

        self.PWM_MID, self.PWM_RANGE = 1500, 500
        self.MAX_LINEAR_SPEED, self.MAX_ANGULAR_SPEED = 0.5, 1.0
        self.UPPER_BOUND, self.LOWER_BOUND = 1745, 1255
        self.current_vx, self.current_wz = 0.0, 0.0
        self.MAX_ACCEL_X, self.MAX_ACCEL_WZ = 0.2, 1.0

        self.create_timer(1.0, self.check_health_logic)
        self.get_logger().info(f">>> 底盘驱动已启动，自动驾驶订阅器已切换为 JSON(String) 模式")

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1, write_timeout=0.1)
            self.get_logger().info(f"✅ 成功连接到底盘串口: {self.port}")
        except Exception as e:
            self.get_logger().error(f"❌ 无法打开串口 {self.port}: {e}")
            self.ser = None

    def check_health_logic(self):
        file_exists = os.path.exists(self.port)
        if file_exists and self.ser and self.ser.is_open:
            msg = String()
            msg.data = "ALIVE"
            self.status_pub.publish(msg)
        else:
            if file_exists:
                self.connect_serial()
            else:
                self.get_logger().warn(f"🚨 找不到设备 {self.port}", throttle_duration_sec=5.0)

    def manual_callback(self, msg):
        self.last_manual_ts = self.get_clock().now().nanoseconds / 1e9
        self.process_twist(msg, "MANUAL")

    def auto_callback(self, msg):
        """处理 JSON 格式的导航指令"""
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self.last_manual_ts) < 1.0:
            return # 手动优先

        try:
            data = json.loads(msg.data)
            # 兼容性：从 JSON 中提取速度或目标位姿
            # 这里的逻辑可以根据你的具体需求扩展
            twist = Twist()
            # 尝试解析速度字段
            twist.linear.x = data.get('vx', 0.0)
            twist.linear.y = data.get('vy', 0.0)
            twist.angular.z = data.get('wz', 0.0)
            
            # 也可以在这里记录 target_x, target_y 等参数供其他逻辑使用
            # self.get_logger().info(f"执行自动导航指令, 目标: ({data.get('target_x')}, {data.get('target_y')})")

            self.process_twist(twist, "AUTO")
        except Exception as e:
            self.get_logger().error(f"解析 JSON 指令失败: {e}")

    def jump_deadband(self, pwm_val):
        if pwm_val == self.PWM_MID: return self.PWM_MID
        if pwm_val > self.PWM_MID and pwm_val < self.UPPER_BOUND: return self.UPPER_BOUND
        if pwm_val < self.PWM_MID and pwm_val > self.LOWER_BOUND: return self.LOWER_BOUND
        return pwm_val

    def process_twist(self, msg, source):
        if not self.ser or not self.ser.is_open:
            return
        now = time.time()
        dt = now - self.last_write_time
        target_vx, target_wz = msg.linear.x, msg.angular.z
        if dt < self.MIN_WRITE_INTERVAL:
            if not (abs(target_vx) < 0.01 and abs(target_wz) < 0.01): return 
        self.last_write_time = now

        if abs(target_vx) < 0.01 and abs(target_wz) < 0.01:
            self.current_vx, self.current_wz = 0.0, 0.0
            self.send_to_hardware(robot_motion.STOP, "停止", source, 0.0, 0.0)
            return
            
        step_x = self.MAX_ACCEL_X * dt
        self.current_vx = float(np.clip(target_vx, self.current_vx - step_x, self.current_vx + step_x))
        step_wz = self.MAX_ACCEL_WZ * dt
        self.current_wz = float(np.clip(target_wz, self.current_wz - step_wz, self.current_wz + step_wz))

        vx, wz = self.current_vx, self.current_wz
        if abs(wz) > 0.01:
            delta = int((abs(wz) / self.MAX_ANGULAR_SPEED) * self.PWM_RANGE)
            pwm = self.jump_deadband(self.PWM_MID - delta if wz > 0 else self.PWM_MID + delta)
            frame = robot_motion.build_turn_left(pwm) if wz > 0 else robot_motion.build_turn_right(pwm)
        else:
            delta = int((abs(vx) / self.MAX_LINEAR_SPEED) * self.PWM_RANGE)
            pwm = self.jump_deadband(self.PWM_MID + delta if vx > 0 else self.PWM_MID - delta)
            frame = robot_motion.build_forward(pwm) if vx > 0 else robot_motion.build_backward(pwm)

        self.send_to_hardware(frame, "MOVING", source, vx, wz)

    def send_to_hardware(self, frame, info, source, vx, wz):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(frame)
            except Exception as e:
                self.get_logger().warn(f"⚠️ 串口写入异常: {e}")

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(robot_motion.STOP)
                self.ser.close()
            except: pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ChassisDriverNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()