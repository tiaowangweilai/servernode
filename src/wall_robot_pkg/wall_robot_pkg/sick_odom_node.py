#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String  # 新增：用于发状态
import socket
import struct
import math
import json
import time

class SickOdomNode(Node):
    def __init__(self):
        super().__init__('sick_odom_node')
        
        # === 网络配置 ===
        self.UDP_IP = "0.0.0.0"
        self.UDP_PORT = 5010
        self.BUFFER_SIZE = 1024
        
        # === 建立UDP Socket ===
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))
        self.sock.setblocking(False) # 设置非阻塞模式，防止卡死ROS
        
        # === 发布者 ===
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        # 新增：发布雷达详细状态 (JSON格式)
        self.status_pub = self.create_publisher(String, '/lidar_status', 10)
        
        # === 定时器 (100Hz 高频读取) ===
        self.timer = self.create_timer(0.01, self.read_udp_data)
        self.get_logger().info(f"SICK 雷达节点已启动，监听 UDP {self.UDP_PORT}")

    def euler_to_quaternion(self, yaw):
        """将角度转换为四元数 (用于发布 /odom)"""
        yaw_rad = math.radians(yaw)
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw_rad / 2)
        qw = math.cos(yaw_rad / 2)
        return [qx, qy, qz, qw]

    def read_udp_data(self):
        try:
            # 尝试接收数据
            data, addr = self.sock.recvfrom(self.BUFFER_SIZE)
            
            # === 解析逻辑 (基于你提供的协议) ===
            if len(data) >= 56:
                # 跳过前16字节报文头
                payload = data[16:56]
                
                # 大端格式解析: >QQqqiBBH
                fmt = '>QQqqiBBH'
                unpacked = struct.unpack(fmt, payload)
                
                # 提取数据
                x_mm = unpacked[2]
                y_mm = unpacked[3]
                yaw_mdeg = unpacked[4]
                localization_stat = unpacked[5] # 定位状态 (0=OK)
                mapmatching_stat  = unpacked[6] # 地图匹配度 (0-100)
                
                # 单位转换
                x_m = x_mm / 1000.0
                y_m = y_mm / 1000.0
                yaw_deg = yaw_mdeg / 1000.0
                
                # --- 1. 发布 /odom (给主控导航用) ---
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_link"
                
                odom_msg.pose.pose.position.x = x_m
                odom_msg.pose.pose.position.y = y_m
                
                q = self.euler_to_quaternion(yaw_deg)
                odom_msg.pose.pose.orientation.x = q[0]
                odom_msg.pose.pose.orientation.y = q[1]
                odom_msg.pose.pose.orientation.z = q[2]
                odom_msg.pose.pose.orientation.w = q[3]
                
                self.odom_pub.publish(odom_msg)
                
                # --- 2. 发布 /lidar_status (给网页显示用) ---
                # 打包成 JSON 字符串
                status_data = {
                    "loc_stat": localization_stat,
                    "map_stat": mapmatching_stat
                }
                status_msg = String()
                status_msg.data = json.dumps(status_data)
                self.status_pub.publish(status_msg)
                
        except BlockingIOError:
            # 没有数据时不报错
            pass
        except Exception as e:
            self.get_logger().error(f"解析错误: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SickOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()