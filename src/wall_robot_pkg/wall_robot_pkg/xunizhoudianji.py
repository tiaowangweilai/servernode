import serial
import time
# from .bujinmotor import *  # 导入修改后的函数
try:
    from .bujinmotor import *
except ImportError:
    from bujinmotor import *
class SerialControlSystem:
    def __init__(self, port='/dev/ttyACM1', baudrate=115200, motor_port='/dev/ttyACM0', motor_baudrate=6000000, node_id=1):
        self.port = port
        self.baudrate = baudrate
        self.motor_port = motor_port
        self.motor_baudrate = motor_baudrate
        self.node_id = node_id
        self.ser = None

    def connect(self):
        """连接串口"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=2
            )
            time.sleep(0.1)
            return self.ser.is_open
        except Exception as e:
            print(f"连接串口失败: {e}")
            return False

    def position_mode_test(self, value):
        """
        位置模式测试
        Args:
            value: 测试值（30 或 0）
        Returns:
            bool: 执行结果
        """
        print(f"\n=== 执行 position_mode_test({value}) ===")
        # 调用修改后的函数，传入所有必要参数
        result = position_mode_test(
            target_position=value,
            port=self.motor_port,
            node_id=self.node_id,
            baudrate=self.motor_baudrate
        )
        print(f"position_mode_test({value}) 执行完成，结果: {result}")
        return result

    def send_start_command_and_wait_for_down(self, max_retries=5, timeout=10):
        """
        发送启动指令，等待成功响应，然后等待down指令
        """
        if not self.ser or not self.ser.is_open:
            print("串口未连接")
            return False

        start_command = [0xAA, 0x55, 0x73, 0x74, 0x61, 0x72, 0x74, 0x0D, 0x0A]
        success_response = "Protocol parsed successfully: start command received"
        down_command = "down"

        retry_count = 0
        data_to_send = bytes(start_command)

        # 第一阶段：发送启动指令
        while retry_count <= max_retries:
            self.ser.reset_input_buffer()

            # 发送启动指令
            self.ser.write(data_to_send)
            print(f"» {[hex(x) for x in start_command]}")

            # 读取响应
            time.sleep(0.1)
            response = self.ser.readline().decode('utf-8', errors='ignore').strip()

            if response:
                # print(f"« {response}")

                if success_response in response:
                    # print("✓ 启动指令执行成功！")
                    break
                else:
                    print("✗ 响应不正确")
            else:
                print("« 无响应")

            retry_count += 1
            if retry_count <= max_retries:
                print(f"第 {retry_count} 次重试...")
                time.sleep(1)
            else:
                print(f"经过 {max_retries} 次重试后启动指令失败")
                return False

        # 第二阶段：等待down指令
        # print("等待接收 'down' 指令...")
        start_time = time.time()

        while time.time() - start_time < timeout:
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    # print(f"« {response}")

                    if down_command in response.lower():
                        # print("✓ 成功接收到 'down' 指令！")
                        return True
            time.sleep(0.1)

        print(f"✗ 在 {timeout} 秒内未收到 'down' 指令")
        return False

    def run_cycle(self, cycles=None):
        """
        运行完整的循环流程
        Args:
            cycles: 循环次数，None表示无限循环
        """
        cycle_count = 0

        try:
            while cycles is None or cycle_count < cycles:
                cycle_count += 1
                # print(f"\n{'=' * 50}")
                # print(f"开始第 {cycle_count} 次循环")
                # print(f"{'=' * 50}")

                # 第一步：position_mode_test(30)
                # print("\n--- 第一步 ---")
                result1 = self.position_mode_test(3.6)
                # if not result1:
                #     print("✗ 第一步失败，终止循环")
                #     break
                # print("✓ 第一步完成")

                # 第二步：send_start_command_and_wait_for_down
                # print("\n--- 第二步 ---")
                result2 = self.send_start_command_and_wait_for_down(max_retries=5, timeout=10)
                # if not result2:
                #     print("✗ 第二步失败，终止循环")
                #     break
                # print("✓ 第二步完成")

                # 第三步：position_mode_test(0)
                # print("\n--- 第三步 ---")
                result3 = self.position_mode_test(0)
                # if not result3:
                #     print("✗ 第三步失败，终止循环")
                #     break
                # print("✓ 第三步完成")

                # 第四步：send_start_command_and_wait_for_down
                # print("\n--- 第四步 ---")
                result4 = self.send_start_command_and_wait_for_down(max_retries=5, timeout=10)
                # if not result4:
                #     print("✗ 第四步失败，终止循环")
                #     break
                # print("✓ 第四步完成")

                # print(f"\n✓ 第 {cycle_count} 次循环完成！")

                # 如果不是无限循环，询问是否继续
                # if cycles is not None and cycle_count < cycles:
                #     print("准备下一次循环...")
                    # time.sleep(1)  # 循环间隔

        except KeyboardInterrupt:
            print("\n用户中断循环")
        except Exception as e:
            print(f"循环执行出错: {e}")

        # print(f"\n循环结束，共完成 {cycle_count} 次循环")

    def close(self):
        """关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("串口已关闭")

    def run_cycle_new(self):
        print("\n--- 第一步 ---")
        result1 = self.position_mode_test(4.2)
        if not result1:
            print("✗ 第一步失败，终止循环")
            # 第三步：position_mode_test(0)
        print("\n--- 第二步 ---")
        result2 = self.position_mode_test(0)
        if not result2:
            print("✗ 第二步失败，终止循环")
        time.sleep(0.1)
        print("\n--- 第三步 ---")
        result3 = self.send_start_command_and_wait_for_down(max_retries=5, timeout=10)
        if not result3:
            print("✗ 第三步失败，终止循环")
            return False
        print("✓ 第三步完成")
        return True

def set_current_position_as_zero(port: str = 'COM6', node_id: int = 1, baudrate: int = 1_000_000) -> bool:
    """设置当前位置为零点"""
    try:
        d = DMTPDriver(port, node_id, baudrate)

        # 清除错误
        d.clear_error()

        # 读取当前位置
        current_position = d.get_actual_position()
        print(f"当前位置: {current_position:.3f} usr")

        # 设置当前位置为零点
        # 使用功能码 0x27 设置当前位置为原点
        result = d._tx_rx(0x27, b'', expect_response=True)

        if result and len(result) >= 1 and result[0] == 0x00:
            print("✓ 零点设置成功")

            # 验证零点设置
            time.sleep(0.1)
            new_position = d.get_actual_position()
            print(f"设置后位置: {new_position:.3f} usr")
            return True
        else:
            print(" 零点设置成功")
            return False

    except Exception as e:
        print(f"设置零点时出现错误: {e}")
        return False
    finally:
        if 'd' in locals():
            d.close()
# 使用示例
# if __name__ == "__main__":
#     # 配置电机参数
#     control_system = SerialControlSystem(
#         port='/dev/ttyACM1',
#         baudrate=115200,
#         motor_port='/dev/ttyACM0',  # 电机串口号
#         motor_baudrate=6000000,  # 电机波特率
#         node_id=1  # 电机节点ID
#     )

#     set_current_position_as_zero('/dev/ttyACM0',1,6000000)

#     if control_system.connect():
#         try:
#             # 选择循环模式
#             print("请选择运行模式:")
#             print("1. 无限循环")
#             print("2. 指定循环次数")

#             choice = input("请输入选择 (1 或 2): ").strip()

#             if choice == "1":
#                 print("开始无限循环... (按 Ctrl+C 停止)")
#                 control_system.run_cycle(cycles=None)
#             elif choice == "2":
#                 try:
#                     cycles = int(input("请输入循环次数: "))
#                     control_system.run_cycle(cycles=cycles)
#                 except ValueError:
#                     print("输入无效，使用默认循环1次")
#                     control_system.run_cycle(cycles=1)
#             else:
#                 print("输入无效，使用默认循环1次")
#                 control_system.run_cycle(cycles=1)

#         except KeyboardInterrupt:
#             print("\n用户中断程序")
#         finally:
#             control_system.close()
#     else:
#         print("无法连接串口")

if __name__ == "__main__":
    # 配置电机参数
    control_system = SerialControlSystem(
        port='/dev/ttyACM1',
        baudrate=115200,
        motor_port='/dev/ttyACM0',      # 电机串口号
        motor_baudrate=6000000,         # 电机波特率
        node_id=1                       # 电机节点ID
    )

    # 归零当前位置
    set_current_position_as_zero('/dev/ttyACM0', 1, 6000000)
    # time.sleep(5)
    if control_system.connect():
        try:
            print("开始运行循环 1 次...")
            control_system.run_cycle(cycles=3)
            # control_system.run_cycle_new()
            # control_system.run_cycle_new()
            # control_system.run_cycle_new()
        except KeyboardInterrupt:
            print("\n用户中断程序")
        finally:
            control_system.close()
    else:
        print("无法连接串口")
