#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np
import json
from rclpy.qos import qos_profile_sensor_data

class VisionProcessorNode(Node):
    def __init__(self):
        super().__init__('vision_processor_node')
        self.bridge = CvBridge()

        # 1. D405 内参
        self.declare_parameter('fx', 434.664)
        self.declare_parameter('fy', 433.539)
        self.declare_parameter('cx', 421.903)
        self.declare_parameter('cy', 243.643)
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value

        # 自检心跳
        self.camera_status_pub = self.create_publisher(String, 'camera_status', 10)
        self.heartbeat_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            self.heartbeat_callback, qos_profile_sensor_data)

        # 2. 订阅与发布
        self.color_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_rect_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.preview_pub = self.create_publisher(Image, '/vision/edge_preview', 10)
        self.compressed_pub = self.create_publisher(CompressedImage, "/vision/edge_preview/compressed", 10)
        self.safety_pub = self.create_publisher(String, '/vision/safety_status', 10)

        # 3. 时间同步
        self.ts = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.image_callback)

        # 阈值 (mm)
        self.GAP_MODERATE = 20
        self.GAP_STRONG = 50
        self.MAX_RANGE = 800


    def heartbeat_callback(self, msg):
        hb_msg = String()
        hb_msg.data = "online"
        self.camera_status_pub.publish(hb_msg)

    def image_callback(self, color_msg, depth_msg):
        try:
            cv_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")

            depth_smooth = cv2.medianBlur(cv_depth, 5)
            valid_mask = (depth_smooth > 0).astype(np.uint8)
            kernel = np.ones((3, 3), np.uint8)
            strict_valid_mask = cv2.erode(valid_mask, kernel, iterations=1)
            depth_grad = cv2.morphologyEx(depth_smooth, cv2.MORPH_GRADIENT, kernel)
            depth_grad[strict_valid_mask == 0] = 0

            is_gap_moderate = depth_grad > self.GAP_MODERATE
            is_gap_strong = depth_grad > self.GAP_STRONG

            gray = cv2.cvtColor(cv_color, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (7, 7), 0)
            canny_raw = cv2.Canny(blurred, 60, 160)
            is_visual_edge = canny_raw > 0
            is_near = (cv_depth > 0) & (cv_depth <= self.MAX_RANGE)

            final_edge_mask = ((is_gap_moderate & is_visual_edge) | is_gap_strong) & is_near
            final_edge_u8 = (final_edge_mask.astype(np.uint8) * 255)
            num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(final_edge_u8, connectivity=8)
            clean_mask = np.zeros_like(final_edge_u8)
            for i in range(1, num_labels):
                if stats[i, cv2.CC_STAT_AREA] > 40:
                    clean_mask[labels == i] = 255

            min_dist = 9999.0
            edge_pixels = np.where(clean_mask > 0)
            if len(edge_pixels[0]) > 0:
                valid_depths = cv_depth[edge_pixels]
                actual_depths = valid_depths[valid_depths > 0]
                if len(actual_depths) > 0:
                    min_dist = float(np.min(actual_depths))

            status = "SAFE"
            if min_dist < 200: status = "DANGER"
            elif min_dist < 450: status = "WARN"

            preview_img = cv_color.copy()
            preview_img[clean_mask > 0] = [0, 255, 0]
            dist_text = f"Obstacle Dist: {min_dist:.1f} mm" if min_dist != 9999.0 else "Path Clear"
            color_txt = (0, 255, 0) if status == "SAFE" else (0, 0, 255)
            cv2.putText(preview_img, dist_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_txt, 2)
            cv2.putText(preview_img, f"Status: {status}", (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_txt, 2)

            self.preview_pub.publish(self.bridge.cv2_to_imgmsg(preview_img, "bgr8", color_msg.header))
            # 预压缩为 JPEG 并发布到 compressed 话题（mjpeg_server 直接转发，不再重新编码）
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 40]
            _result, _encimg = cv2.imencode(".jpg", preview_img, encode_param)
            if _result:
                from sensor_msgs.msg import CompressedImage as _CI
                _compressed = _CI()
                _compressed.header = color_msg.header
                _compressed.format = "jpeg"
                _compressed.data = _encimg.tobytes()
                self.compressed_pub.publish(_compressed)

            # 安全状态
            safety_info = {"status": status, "min_dist_mm": min_dist if min_dist != 9999.0 else -1}
            self.safety_pub.publish(String(data=json.dumps(safety_info)))

        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
