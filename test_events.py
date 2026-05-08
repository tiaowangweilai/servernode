#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class EventTestNode(Node):
    def __init__(self):
        super().__init__('event_test_node')
        self.event_pub = self.create_publisher(String, '/mission/events', 10)
        self.sys_sub = self.create_subscription(String, '/mission/sys_command', self.sys_callback, 10)
        self.get_logger().info('=' * 50)
        self.get_logger().info('事件测试工具已启动')
        self.get_logger().info('=' * 50)
        self.get_logger().info('')
        self.get_logger().info('可用命令:')
        self.get_logger().info('1 -> 发 capture (开始采集)')
        self.get_logger().info('2 -> 发 save (保存文件)')
        self.get_logger().info('3 -> 发 work_complete (作业完成)')
        self.get_logger().info('q -> 退出')

    def sys_callback(self, msg):
        self.get_logger().info(f"<<< [sys_command] 收到: {msg.data}")

    def send_event(self, event_str):
        msg = String()
        msg.data = event_str
        self.event_pub.publish(msg)
        self.get_logger().info(f">>> [events] 已上发: {event_str}")

def main():
    rclpy.init()
    node = EventTestNode()
    import threading
    def input_thread():
        while rclpy.ok():
            try:
                cmd = input()
                if cmd == 'q': break
                elif cmd == '1': node.send_event('capture')
                elif cmd == '2': node.send_event('save')
                elif cmd == '3': node.send_event('work_complete')
            except: break
        # input线程结束时触发关闭
        if rclpy.ok():
            rclpy.shutdown()
            
    t = threading.Thread(target=input_thread, daemon=True)
    t.start()
    
    try: 
        rclpy.spin(node)
    except: 
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()