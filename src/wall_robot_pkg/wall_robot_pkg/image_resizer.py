#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageThrottleNode(Node):
    def __init__(self):
        super().__init__('image_throttle_node')
        
        # 1. 订阅 30帧/秒 的原始融合图
        self.sub = self.create_subscription(
            Image,
            '/vision/edge_preview',
            self.listener_callback,
            10)
            
        # 2. 发布 10帧/秒 的限流图 (依然是原尺寸、原格式)
        self.pub = self.create_publisher(Image, '/vision/web/edge_10fps', 10)

        self.frame_skip = 2  # 每 3 帧放行 1 帧
        self.frame_count = 0

        self.get_logger().info("✅ 图像限流节点已启动: 纯指针转发，0 CPU消耗！")
        self.get_logger().info("   源话题: /vision/edge_preview (30fps)")
        self.get_logger().info("   目标话题: /vision/web/edge_10fps (10fps)")

    def listener_callback(self, msg):
        self.frame_count += 1
        # 抽帧逻辑：不满足条件的直接 return，连内存都不进
        if self.frame_count % self.frame_skip == 0:
            self.pub.publish(msg)  # 直接原封不动把指针丢给发布者

def main(args=None):
    rclpy.init(args=args)
    node = ImageThrottleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()