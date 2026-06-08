#!/usr/bin/env python3
"""
电机循环测试脚本 (ROS2 版本)
循环: M1下压(300) → 推杆前推(500ms) → M1抬起(0) → 推杆后退(500ms)

用法:
  终端1: ros2 run wall_robot_pkg mechanism_driver_node
  终端2: python3 test_motor_loop.py

需要先启动 mechanism_driver_node (负责硬件驱动)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time


class MotorLoopTest(Node):
    def __init__(self):
        super().__init__("motor_loop_test")

        # 发布器
        self.m1_pub = self.create_publisher(Int32, "/mech/m1_target", 10)
        self.pushrod_fwd_pub = self.create_publisher(Int32, "/mech/push_rod_time", 10)
        self.pushrod_bwd_pub = self.create_publisher(Int32, "/mech/push_rod_backward", 10)

        self.get_logger().info("=" * 50)
        self.get_logger().info("  电机循环测试 (ROS2)")
        self.get_logger().info("  循环: M1下压 -> 推杆前推 -> M1抬起 -> 推杆后退")
        self.get_logger().info("=" * 50)
        self.get_logger().info("  确保 mechanism_driver_node 正在运行!")
        self.get_logger().info("=" * 50)

    def run_loop(self):
        cycle = 0
        try:
            while rclpy.ok():
                cycle += 1
                self.get_logger().info(f"\n===== 第 {cycle} 次循环 =====")

                # Step 1: M1 下压 (300)
                self.get_logger().info("  [1/4] M1 下压至 300...")
                self.m1_pub.publish(Int32(data=300))
                time.sleep(1.5)

                # Step 2: 推杆前推 (500ms)
                self.get_logger().info("  [2/4] 推杆前推 500ms...")
                self.pushrod_fwd_pub.publish(Int32(data=500))
                time.sleep(0.8)

                # Step 3: M1 抬起 (0)
                self.get_logger().info("  [3/4] M1 抬起至 0...")
                self.m1_pub.publish(Int32(data=0))
                time.sleep(1.0)

                # Step 4: 推杆后退 (500ms)
                self.get_logger().info("  [4/4] 推杆后退 500ms...")
                self.pushrod_bwd_pub.publish(Int32(data=500))
                time.sleep(0.8)

                self.get_logger().info(f"  ✅ 第 {cycle} 次循环完成")

        except KeyboardInterrupt:
            self.get_logger().info("\n测试终止")


def main(args=None):
    rclpy.init(args=args)
    node = MotorLoopTest()
    try:
        node.run_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
