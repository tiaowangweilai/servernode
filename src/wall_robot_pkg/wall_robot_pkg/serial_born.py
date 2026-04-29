def generate_motor_command(speeds, angles, directions, push_params):
    """
    生成电机控制指令(包含4个电机和1个推杆)

    参数:
        speeds:  list - 4个电机的速度值 (0-65535)
        angles:  list - 4个电机的角度值 (0-65535)
        directions: list - 4个电机的方向 (0或1)
        push_params: list - 推杆参数 [运动距离(0-255), 运动方向(0或1)]

    返回:
        bytes: 生成的27字节指令
    """
    # 电机2->虚拟轴  2->1（io）
    # 
    # 协议帧头
    header = bytes([0xAA, 0x55])

    # 打包电机数据
    motor_data = bytearray()
    for i in range(4):
        # 速度(2字节，大端序)
        motor_data.extend(speeds[i].to_bytes(2, 'big'))
        # 角度(2字节，大端序)
        motor_data.extend(angles[i].to_bytes(2, 'big'))
        # 方向(1字节)
        motor_data.append(directions[i])

    # 打包推杆数据
    # 运动距离(1字节)
    motor_data.append(push_params[0])
    # 运动方向(1字节)
    motor_data.append(push_params[1])

    # 计算校验和(不包括校验和本身)
    checksum = sum(header) + sum(motor_data)
    checksum = checksum & 0xFF  # 取低8位

    # 帧尾
    footer = bytes([0x0D, 0x0A])

    # 组合完整指令
    full_command = header + motor_data + bytes([checksum]) + footer

    return full_command


def print_command_hex(command):
    """以十六进制格式打印指令"""
    print("生成指令(十六进制):")
    print(' '.join(f"{b:02X}" for b in command))


def print_command_details(command):
    """打印指令详细解析"""
    print("\n指令解析:")
    print(f"HEADER: {command[0]:02X} {command[1]:02X}")

    # 解析4个电机数据
    for i in range(4):
        start = 2 + i * 5
        speed = int.from_bytes(command[start:start + 2], 'big')
        angle = int.from_bytes(command[start + 2:start + 4], 'big')
        direction = command[start + 4]
        print(f"电机{i + 1}: 速度={speed}, 角度={angle}, 方向={direction}")

    # 解析推杆数据
    push_start = 22
    push_distance = command[push_start]
    push_direction = command[push_start + 1]
    print(f"推杆: 距离={push_distance}, 方向={push_direction}")

    print(f"CHECKSUM: {command[24]:02X}")
    print(f"CRLF: {command[25]:02X} {command[26]:02X}")


# 示例使用
if __name__ == "__main__":
    # 示例1: 正常运动测试
    # 2虚拟轴 4是推杆 
    speeds = [0, 250, 0, 0]
    angles = [90, 90, 90, 90]
    directions = [0, 1, 0, 1]
    push_params = [0, 0]  # [距离, 方向]

    print("示例1: 正常运动测试")
    cmd = generate_motor_command(speeds, angles, directions, push_params)
    print_command_hex(cmd)
    print_command_details(cmd)

    # # 示例2: 边界值测试
    # speeds = [0, 0, 0, 0]
    # angles = [0, 0, 0, 0]
    # directions = [0, 1, 0, 1]
    # push_params = [99, 0]  # 最大距离值

    # print("\n示例2")
    # cmd = generate_motor_command(speeds, angles, directions, push_params)
    # print_command_hex(cmd)
    # print_command_details(cmd)