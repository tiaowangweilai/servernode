import time
import serial


class YKPPWM103TController:

    def __init__(self, port="/dev/ttyCH341USB0", baudrate=115200, device_addr=0x01):
        """底层的 Modbus 驱动控制器"""
        self.addr = device_addr
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.5,
        )

    def calculate_crc(self, data: bytes) -> bytes:
        """计算CRC16校验码"""
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    def send_cmd(self, func_code: int, reg_addr: int, data_val: int):
        """发送写单个寄存器指令"""
        packet = bytes([
            self.addr,
            func_code,
            (reg_addr >> 8) & 0xFF,
            reg_addr & 0xFF,
            (data_val >> 8) & 0xFF,
            data_val & 0xFF,
        ])
        packet += self.calculate_crc(packet)
        self.ser.write(packet)
        response = self.ser.read(8)
        return response

    def init_channels_mode(self):
        """初始化：通道1设为PWM模式(3)，通道2设为普通模式(0)"""
        self.send_cmd(0x06, 0x0004, 3)
        time.sleep(0.1)
        self.send_cmd(0x06, 0x0005, 0)
        time.sleep(0.1)

    def close(self):
        """💡 补上了这个漏掉的关闭串口函数"""
        if hasattr(self, "ser") and self.ser.is_open:
            self.ser.close()


# =====================================================================
#                      全局初始化与对外暴露的接口
# =====================================================================

# 1. 创建全局唯一的控制器实例
_controller = YKPPWM103TController(
    port="/dev/ttyCH341USB0", baudrate=115200, device_addr=0x01
)
_controller.init_channels_mode()


def control_channel1_pwm(frequency: int, duty_cycle: int, enable: bool = True):
    """接口1：调节通道一的 PWM 输出

    :param frequency: 频率 (0 ~ 10000 Hz)
    :param duty_cycle: 占空比 (0 ~ 100 %)
    :param enable: True 为开启PWM输出，False 为直接关闭该通道输出
    """
    if not enable:
        _controller.send_cmd(0x06, 0x0002, 0)
        print("接口调用：通道1已物理关闭。")
        return

    # 1. 设置频率
    _controller.send_cmd(0x06, 0x000B, frequency)
    time.sleep(0.05)

    # 2. 设置占空比
    _controller.send_cmd(0x06, 0x0008, duty_cycle)
    time.sleep(0.05)

    # 3. 通道1输出使能
    _controller.send_cmd(0x06, 0x0002, 1)
    print(
        f"接口调用：通道1 PWM 调节成功 -> 频率: {frequency}Hz, 占空比: {duty_cycle}%"
    )


def control_channel2_switch(turn_on: bool):
    """接口2：控制通道二的普通启停开关

    :param turn_on: True 为开启(输出高电平)，False 为关闭(输出低电平)
    """
    val = 1 if turn_on else 0
    _controller.send_cmd(0x06, 0x0003, val)
    status = "开启(高电平)" if turn_on else "关闭(低电平)"
    print(f"接口调用：通道2开关状态切换 -> {status}")


def close_controller():
    """接口3：显式关闭串口（程序彻底退出时调用）"""
    _controller.close()


# =====================================================================
# 当直接运行本脚本时进行的内部自我测试
if __name__ == "__main__":
    try:
        print("正在进行接口自测...")
        control_channel2_switch(True)  # 打开通道2
        time.sleep(2)
        control_channel1_pwm(2000, 80)  # 打开通道1并给 2000Hz, 40%
        time.sleep(2)
        control_channel2_switch(False)  # 关闭通道2
        control_channel1_pwm(0, 0, enable=False)  # 关闭通道1
    finally:
        close_controller()