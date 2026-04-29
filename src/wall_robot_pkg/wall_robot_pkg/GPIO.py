import Jetson.GPIO as GPIO
import serial
import time
import threading
import math
from enum import Enum


# ============================ GPIO引脚定义 ============================
# Jetson Nano引脚定义（根据实际连接调整）
class Pins:
    # 推杆电机控制引脚
    PUSH_ROD_IN1 = 29  # GPIO27 - 对应STM32的PB1
    PUSH_ROD_IN2 = 31  # GPIO22 - 对应STM32的PB11

    # 虚拟轴电机控制引脚
    IMAGE_MOTOR_IN1 = 32  # GPIO17 - 对应STM32的PB0
    IMAGE_MOTOR_IN2 = 33  # GPIO18 - 对应STM32的PB10


# ============================ 全局变量和状态 ============================
class MotorState(Enum):
    STOP = 0
    FORWARD = 1
    BACKWARD = 2


class Pusher:
    def __init__(self):
        self.position_mm = 0.0  # 当前位置 (mm)
        self.target_mm = 0.0  # 目标位置 (mm)
        self.running = False  # 是否正在运行
        self.position_push = 0  # 推杆位置计数

        # 编码器相关
        self.encoder_last = 0
        self.encoder_value = 0
        self.mm_per_pulse = 4.0 / 7.0  # 每脉冲位移 ≈ 0.5714 mm

        # 控制参数
        self.tolerance_mm = 1.0  # 到位容差
        self.timeout_ms = 5000  # 超时时间
        self.still_window_ms = 200  # 无进展判定窗口
        self.still_threshold = 1  # 无进展阈值

global_pusher = Pusher()
# ============================ 初始化函数 ============================
def gpio_init():
    """初始化GPIO"""
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    # 设置推杆电机引脚
    GPIO.setup(Pins.PUSH_ROD_IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(Pins.PUSH_ROD_IN2, GPIO.OUT, initial=GPIO.LOW)

    # 设置虚拟轴电机引脚
    GPIO.setup(Pins.IMAGE_MOTOR_IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(Pins.IMAGE_MOTOR_IN2, GPIO.OUT, initial=GPIO.LOW)

# ============================ 电机控制函数 ============================
def push_rod_stop():
    """停止推杆电机"""
    GPIO.output(Pins.PUSH_ROD_IN1, GPIO.LOW)
    GPIO.output(Pins.PUSH_ROD_IN2, GPIO.LOW)
    time.sleep(0.05)  # 延时50ms

def push_rod_forward_time(run_time_ms):
    """推杆前进指定时间"""
    GPIO.output(Pins.PUSH_ROD_IN1, GPIO.HIGH)
    GPIO.output(Pins.PUSH_ROD_IN2, GPIO.LOW)

    GPIO.output(Pins.IMAGE_MOTOR_IN1, GPIO.HIGH)
    GPIO.output(Pins.IMAGE_MOTOR_IN2, GPIO.LOW)
    
    time.sleep(run_time_ms / 1000.0)  # 转换为秒

    push_rod_stop()
    image_motor_stop()
    time.sleep(0.2)

def image_forward(run_time_ms):
    GPIO.output(Pins.IMAGE_MOTOR_IN1, GPIO.HIGH)
    GPIO.output(Pins.IMAGE_MOTOR_IN2, GPIO.LOW)
    
    time.sleep(run_time_ms / 1000.0)  # 转换为秒
    image_motor_stop()

def push_rod_backward_time(run_time_ms, pusher):
    """推杆后退指定时间"""
    GPIO.output(Pins.PUSH_ROD_IN1, GPIO.LOW)
    GPIO.output(Pins.PUSH_ROD_IN2, GPIO.HIGH)

    time.sleep(run_time_ms / 1000.0)

    push_rod_stop()
    time.sleep(0.2)


def push_rod_backward(run_time_ms):
    """推杆后退指定时间"""
    GPIO.output(Pins.PUSH_ROD_IN1, GPIO.LOW)
    GPIO.output(Pins.PUSH_ROD_IN2, GPIO.HIGH)

    time.sleep(run_time_ms / 1000.0)

    push_rod_stop()
    time.sleep(0.2)
    
# ============================ 虚拟轴电机控制 ============================
def image_motor_stop():
    """停止图像电机"""
    GPIO.output(Pins.IMAGE_MOTOR_IN1, GPIO.LOW)
    GPIO.output(Pins.IMAGE_MOTOR_IN2, GPIO.LOW)

def image_motor_backward():
    """图像电机后退"""
    GPIO.output(Pins.IMAGE_MOTOR_IN1, GPIO.LOW)
    GPIO.output(Pins.IMAGE_MOTOR_IN2, GPIO.HIGH)

def image_motor_forward():
    """图像电机前进"""
    GPIO.output(Pins.IMAGE_MOTOR_IN1, GPIO.HIGH)
    GPIO.output(Pins.IMAGE_MOTOR_IN2, GPIO.LOW)

def image_motor_forward_time(run_time_ms):
    """图像电机前进指定时间"""
    image_motor_forward()
    time.sleep(run_time_ms / 1000.0)
    image_motor_stop()

def image_motor_backward_time(run_time_ms):
    """图像电机后退指定时间"""
    image_motor_backward()
    time.sleep(run_time_ms / 1000.0)
    image_motor_stop()

# ============================ 主控制循环 ============================
def main_control_loop():
    """主控制循环"""
    # 初始化
    gpio_init()

    # 创建对象
    pusher = Pusher()
    push_rod_backward_time(1500, pusher)  # 后退3秒
    # 初始后退动作（模拟STM32的初始化动作）
    print("执行初始后退动作...")
    while True:
        # image_motor_forward_time(300)
        push_rod_forward_time(300)  # 后退3秒

        # push_rod_forward_time(260)
        
        # push_rod_forward_time(260)

        # push_rod_forward_time(260)

        # push_rod_forward_time(260)

        # push_rod_forward_time(260)

        # push_rod_backward_time(1500, pusher)  # 后退3秒


    # try:
    #     while True:
    #         # 更新推杆位置计数
    #         pusher.position_push += 1
    #         print(f"收到新命令，位置计数: {pusher.position_push}")

    #         # 设置运行标志
    #         pusher.running = True

    #         # 运行控制逻辑
    #         if pusher.running:
    #             # 图像电机前进3秒
    #             print("虚拟轴电机前进...")
    #             image_motor_forward_time(300)  # 前进0.3秒

    #             # 停止图像电机
    #             image_motor_stop()
    #             pusher.running = False

    #             # 根据位置计数执行不同动作
    #             if pusher.position_push != 6:
    #                 # 推杆前进3秒
    #                 print("推杆前进...")
    #                 push_rod_forward_time(300, pusher)
    #             else:
    #                 # 重置计数器并后退
    #                 pusher.position_push = 0
    #                 print("推杆后退...")
    #                 push_rod_backward_time(3000, pusher)

    #             print("动作完成")

    #         time.sleep(0.1)  # 降低CPU使用率

    # except KeyboardInterrupt:
    #     print("\n程序被用户中断")
    # finally:
    #     # 清理
    #     print("清理GPIO...")
    #     push_rod_stop()
    #     image_motor_stop()
    #     GPIO.cleanup()
    #     print("程序退出")


# ============================ 主函数 ============================
if __name__ == "__main__":
    print("=== 推杆电机控制系统 ====")
    print("基于Jetson Nano的Python实现")
    print("模拟STM32的推杆控制逻辑")
    print("=" * 30)

    main_control_loop()
