#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import time

# 导入你提供的驱动封装 (注意这里使用相对导入)
from . import rgbd

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # === 发布者 ===
        # 彩色图话题: /camera/color/image_raw
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        # 深度图话题: /camera/depth/image_raw
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        # 相机内参信息 (可选，用于 Rviz 叠加显示)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        
        # 工具类：用于将 OpenCV 图像转为 ROS 消息
        self.bridge = CvBridge()
        
        # === 获取相机参数 ===
        self.get_logger().info("正在初始化 Orbbec 相机...")
        try:
            
            # 尝试获取一次参数，确保相机连接
            self.cam_params = rgbd.get_camera_params()
            self.get_logger().info("相机初始化成功！")
        except Exception as e:
            self.get_logger().error(f"相机初始化失败: {e}")
            self.cam_params = None

        # === 定时采集 (30Hz) ===
        self.timer = self.create_timer(1.0/60.0, self.timer_callback)

    def timer_callback(self):
        # 1. 从 rgbd.py 获取一帧
        # get_frame_once 返回: (color_img, depth_raw, scale)
        color_img, depth_raw, scale = rgbd._CAPTURE.get_frame_once(timeout_ms=30)
        
        if color_img is None and depth_raw is None:
            return

        timestamp = self.get_clock().now().to_msg()
        frame_id = "camera_link" # 坐标系名称

        # 2. 发布彩色图
        if color_img is not None:
            try:
                # 转换: numpy(BGR) -> ROS Msg
                msg = self.bridge.cv2_to_imgmsg(color_img, encoding="bgr8")
                msg.header.stamp = timestamp
                msg.header.frame_id = frame_id
                self.color_pub.publish(msg)
                
                # 顺便发布相机内参 (只需在有图时发布)
                if self.cam_params:
                    info_msg = self.create_camera_info(timestamp, frame_id)
                    self.info_pub.publish(info_msg)
            except Exception as e:
                self.get_logger().warn(f"发布彩色图出错: {e}")

        # 3. 发布深度图
        if depth_raw is not None:
            try:
                # 转换: numpy(uint16 mm) -> ROS Msg
                # encoding="mono16" 对应 16位深度数据
                msg = self.bridge.cv2_to_imgmsg(depth_raw, encoding="mono16")
                msg.header.stamp = timestamp
                msg.header.frame_id = frame_id
                self.depth_pub.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"发布深度图出错: {e}")

    def create_camera_info(self, timestamp, frame_id):
        """构建 CameraInfo 消息，Rviz 需要这个才能正确对齐图像"""
        info = CameraInfo()
        info.header.stamp = timestamp
        info.header.frame_id = frame_id
        
        c_intr = self.cam_params['color_intrinsic']
        c_dist = self.cam_params['color_distortion']
        
        info.width = c_intr['width']
        info.height = c_intr['height']
        
        # 畸变模型
        info.distortion_model = "plumb_bob"
        info.d = [float(x) for x in c_dist['coeffs'][:5]] # 取前5个畸变系数
        
        # 内参矩阵 K (3x3)
        fx, fy = c_intr['fx'], c_intr['fy']
        cx, cy = c_intr['cx'], c_intr['cy']
        info.k = [fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0]
        
        # 投影矩阵 P (3x4) - 简化版，假设无校正旋转
        info.p = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
                  
        return info

    def destroy_node(self):
        # 关闭时释放相机资源
        rgbd.close_device()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()