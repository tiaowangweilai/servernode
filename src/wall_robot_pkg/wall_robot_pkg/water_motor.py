import time
import serial


class YKPPWM103TController:

    def __init__(self, port="/dev/ttyCH341USB0", baudrate=115200, device_addr=0x01):
        """底层的 Modbus 驱动控制器"""
        self.addr = device_addr
        self.port = port
        self.baud = baudrate
        self.ser = None

    def _ensure_open(self):
        """确保串口已打开"""
        if self.ser is None or not self.ser.is_open:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.3,
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

    def send_cmd(self, func_code: int, reg_addr: int, data_val: int, retries: int = 3):
        """发送写单个寄存器指令（带重试）"""
        if self.ser is None:
            self._ensure_open()
        packet = bytes([
            self.addr,
            func_code,
            (reg_addr >> 8) & 0xFF,
            reg_addr & 0xFF,
            (data_val >> 8) & 0xFF,
            data_val & 0xFF,
        ])
        packet += self.calculate_crc(packet)

        for attempt in range(retries):
            try:
                # 先清空读缓冲区（丢弃 SBUS 残留数据）
                self.ser.reset_input_buffer()
                self.ser.write(packet)
                response = self.ser.read(8)
                if response and len(response) == 8:
                    return response
                # 没收到响应，等一会重试
                time.sleep(0.05)
            except serial.SerialException:
                time.sleep(0.1)
        return b''

    def init_channels_mode(self, retries=3):
        """初始化：通道1设为PWM模式(3)，通道2设为普通模式(0)"""
        self.send_cmd(0x06, 0x0004, 3, retries=retries)
        time.sleep(0.05)
        self.send_cmd(0x06, 0x0005, 0, retries=retries)
        time.sleep(0.05)

    def close(self):
        """关闭串口"""
        if self.ser is not None and self.ser.is_open:
            self.ser.close()


# =====================================================================
#                      延迟初始化的对外接口
# =====================================================================


def _create_controller():
    """创建一个新的控制器，初始化后返回（用完需 close）"""
    ctrl = YKPPWM103TController(
        port="/dev/ttyCH341USB0", baudrate=115200, device_addr=0x01
    )
    ctrl._ensure_open()
    ctrl.init_channels_mode(retries=3)
    return ctrl


def control_channel1_pwm(frequency: int, duty_cycle: int, enable: bool = True):
    """接口1：调节通道一的 PWM 输出

    :param frequency: 频率 (0 ~ 10000 Hz)
    :param duty_cycle: 占空比 (0 ~ 100 %)
    :param enable: True 为开启PWM输出，False 为直接关闭该通道输出
    """
    ctrl = _create_controller()
    try:
        if not enable:
            ctrl.send_cmd(0x06, 0x0002, 0, retries=3)
            print("接口调用：通道1已物理关闭。")
            return

        # 1. 设置频率
        ctrl.send_cmd(0x06, 0x000B, frequency, retries=3)
        time.sleep(0.05)

        # 2. 设置占空比
        ctrl.send_cmd(0x06, 0x0008, duty_cycle, retries=3)
        time.sleep(0.05)

        # 3. 通道1输出使能
        ctrl.send_cmd(0x06, 0x0002, 1, retries=3)
        print(
            f"接口调用：通道1 PWM 调节成功 -> 频率: {frequency}Hz, 占空比: {duty_cycle}%"
        )
    finally:
        ctrl.close()


def control_channel2_switch(turn_on: bool):
    """接口2：控制通道二的普通启停开关

    :param turn_on: True 为开启(输出高电平)，False 为关闭(输出低电平)
    """
    ctrl = _create_controller()
    try:
        val = 1 if turn_on else 0
        ctrl.send_cmd(0x06, 0x0003, val, retries=3)
        status = "开启(高电平)" if turn_on else "关闭(低电平)"
        print(f"接口调用：通道2开关状态切换 -> {status}")
    finally:
        ctrl.close()


def close_controller():
    """接口3：保持向后兼容，无操作（现在每个操作自动管理串口生命周期）"""
    pass


# =====================================================================
# 当直接运行本脚本时进行的内部自我测试
if __name__ == "__main__":
    try:
        print("正在进行接口自测...")
        control_channel2_switch(True)  # 打开通道2
        time.sleep(2)
        control_channel1_pwm(2000, 80)  # 打开通道1并给 2000Hz, 80%
        time.sleep(2)
        control_channel2_switch(False)  # 关闭通道2
        control_channel1_pwm(0, 0, enable=False)  # 关闭通道1
    finally:
        close_controller()