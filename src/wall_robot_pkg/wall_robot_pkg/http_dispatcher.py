import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Point, Twist

from std_msgs.msg import String

import json



class InteractionDispatcher(Node):

    def __init__(self):

        super().__init__('ws_dispatcher_node')

        # ROS 2 发布者（向下位机内部发指令）

        self.params_pub = self.create_publisher(String, '/mission/params', 10)

        self.click_pub = self.create_publisher(Point, '/mission/click', 10)

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)

       

        # 🌟 核心：订阅 9090 (rosbridge) 已经拆好信封、转发过来的原始 JSON 字符串

        self.web_cmd_sub = self.create_subscription(String, '/web_to_dispatcher', self.web_cmd_callback, 10)

       

        self.get_logger().info("🚀 [通讯中枢] 就绪 (监听 9090 标准 ROSBridge 话题转发)")



    def web_cmd_callback(self, msg):

        try:

            # msg.data 里面就是你那串原汁原味的 [{"target": "agv", "data": {...}}]

            raw_data = json.loads(msg.data)

            if isinstance(raw_data, list) and len(raw_data) > 0:

                item = raw_data[0]

                target = item.get("target", "")

                data_payload = item.get("data", {})

               

                # 🌟 拦截：放行 agv 以及需要自检的 Lidar 和 Camera

                if target in ["agv", "Lidar", "Camera"]:

                    if "command" in data_payload:

                        cmd_type = data_payload["command"]

                       

                        # ==============================================================

                        # 🌟 修改点：拦截到耦合指令后，记录日志，并放行转发

                        # ==============================================================

                        if cmd_type in ["coupling_success", "coupling_failed"]:

                            if cmd_type == "coupling_success":

                                self.get_logger().info("🟢 [耦合判定] 收到上位机：耦合【成功】！(正在转发给抽水节点)")

                            else:

                                self.get_logger().warn("🔴 [耦合判定] 收到上位机：耦合【失败】！(正在转发给抽水节点)")

                           

                            # 🌟 删除了原来的 return！

                            # 代码会继续往下走，执行 self.params_pub.publish 广播到整个下位机网络

                        # ==============================================================



                        if cmd_type == "emergency_stop" and target == "agv":

                            twist = Twist()

                            self.vel_pub.publish(twist)

                            self.get_logger().warn("🚨 收到紧急停止！全车锁死。")

                       

                        # 透传包含 command 的 JSON 给网络

                        msg_out = String()

                        msg_out.data = msg.data

                        self.params_pub.publish(msg_out)

                        self.get_logger().info(f"📩 转发上位机指令 -> Target: {target}, Cmd: {cmd_type}")

                       

                    elif target == "agv" and ("vx" in data_payload or "wz" in data_payload):

                        vx = float(data_payload.get("vx", 0.0))

                        wz = float(data_payload.get("wz", 0.0))

                        twist = Twist()

                        twist.linear.x = vx; twist.angular.z = wz

                        self.vel_pub.publish(twist)

                       

                # 拦截像素点击

                elif target == "click":

                    px = float(data_payload.get("x", 0.0))

                    py = float(data_payload.get("y", 0.0))

                    self.click_pub.publish(Point(x=px, y=py, z=0.0))

                   

        except Exception as e:

            self.get_logger().error(f"JSON 解析错误: {e}")



def main(args=None):

    rclpy.init(args=args)

    node = InteractionDispatcher()

    try: rclpy.spin(node)

    except KeyboardInterrupt: pass

    finally: node.destroy_node(); rclpy.shutdown()



if __name__ == '__main__': main()