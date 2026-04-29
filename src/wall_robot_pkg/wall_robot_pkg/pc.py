#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
简易机器人上位机：
- 方向按键：按住一直发运动命令，松开停止
- 摄像头：勾选“开启实时预览”后，在界面中显示画面
- 推杆：勾选“推杆运行”后周期性发送推杆运动命令（可选伸/缩），取消勾选则发送停止命令
"""

import tkinter as tk
from tkinter import ttk
import sys
import cv2
from PIL import Image, ImageTk

# ==================== 配置区 ====================

# 机器人命令发送间隔（毫秒）
ROBOT_INTERVAL_MS = 300

# 推杆命令发送间隔（毫秒）
PUSHROD_INTERVAL_MS = 500

# 摄像头配置
CAM_INDEX = 0          # 摄像头编号（0/1/2...）
CAM_UPDATE_MS = 33     # 约 30 FPS 刷新

# =================================================
# 这里用“函数”占位，实际使用时改成你自己的串口发送逻辑
# =================================================

def send_robot_cmd(action: str):
    """
    发送机器人运动命令
    action: "forward" / "backward" / "left" / "right" / "rotate" / "stop"
    """
    print(f"[ROBOT] action = {action}")
    # TODO: 替换为你自己的代码，例如：
    # from robot_motion import send_frame, FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT, STOP
    # if action == "forward":
    #     send_frame(FORWARD)
    # elif action == "backward":
    #     send_frame(BACKWARD)
    # ...


def send_pushrod_cmd(mode: str):
    """
    发送推杆控制命令
    mode: "extend" / "retract" / "stop"
    """
    print(f"[PUSHR] mode = {mode}")
    # TODO: 替换为你自己的代码，例如：
    # if mode == "extend":
    #     send_pushrod_pwm(某个通道, 伸出的 PWM 值)
    # elif mode == "retract":
    #     send_pushrod_pwm(某个通道, 收回的 PWM 值)
    # elif mode == "stop":
    #     send_pushrod_pwm(某个通道, 中位 PWM)


# ==================== Tk & 全局状态 ====================

root = tk.Tk()
root.title("机器人控制上位机（示例）")
root.geometry("900x600")

# 机器人按键“按住状态”
robot_holding = {
    "forward": False,
    "backward": False,
    "left": False,
    "right": False,
    "rotate": False,
}

# 摄像头相关
cam_var = tk.BooleanVar(value=False)
cap = None
cam_running = False
cam_photo = None  # 防止被垃圾回收
camera_label = None  # 先占个位置

# 推杆相关
pushrod_var = tk.BooleanVar(value=False)
pushrod_dir_var = tk.StringVar(value="extend")  # 默认伸出
pushrod_running = False
pushrod_status_label = None


# ==================== 摄像头逻辑 ====================

def start_camera():
    global cap, cam_running
    if cam_running:
        return
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        camera_label.config(text="无法打开摄像头")
        cam_var.set(False)
        return
    cam_running = True
    camera_label.config(text="摄像头已开启")
    update_camera_frame()


def stop_camera():
    global cap, cam_running, cam_photo
    cam_running = False
    if cap is not None:
        cap.release()
        cap = None
    cam_photo = None
    camera_label.config(image="", text="预览已关闭")


def update_camera_frame():
    """
    Tk 的 after 回调，从摄像头读取一帧并显示到 Label
    """
    global cap, cam_running, cam_photo
    if not cam_running or cap is None:
        return

    ret, frame = cap.read()
    if not ret:
        camera_label.config(text="读取失败")
    else:
        # BGR -> RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # 可以缩放一下，防止太大
        frame = cv2.resize(frame, (400, 300))
        img = Image.fromarray(frame)
        cam_photo = ImageTk.PhotoImage(img)
        camera_label.config(image=cam_photo, text="")

    # 计划下一帧
    root.after(CAM_UPDATE_MS, update_camera_frame)


def on_cam_toggle():
    if cam_var.get():
        start_camera()
    else:
        stop_camera()


# ==================== 机器人方向控制逻辑 ====================

def robot_loop(action: str):
    """
    按住方向键时周期性发送命令
    """
    if not robot_holding[action]:
        return
    # 每次循环发送一次该方向命令
    send_robot_cmd(action)
    # 下一轮
    root.after(ROBOT_INTERVAL_MS, lambda: robot_loop(action))


def on_robot_press(action: str, event=None):
    """
    按下方向按钮：开始循环发送该方向命令
    """
    if robot_holding[action]:
        return
    robot_holding[action] = True
    robot_loop(action)


def on_robot_release(action: str, event=None):
    """
    松开方向按钮：停止循环，并发送一次 STOP
    """
    robot_holding[action] = False
    send_robot_cmd("stop")


# ==================== 推杆控制逻辑 ====================

def pushrod_loop():
    """
    推杆运行时，周期性发送“伸/缩”命令
    """
    global pushrod_running
    if not pushrod_running:
        return
    mode = pushrod_dir_var.get()  # "extend" / "retract"
    send_pushrod_cmd(mode)
    root.after(PUSHROD_INTERVAL_MS, pushrod_loop)


def on_pushrod_toggle():
    """
    勾选/取消“推杆运行”
    """
    global pushrod_running
    pushrod_running = pushrod_var.get()

    if pushrod_running:
        pushrod_status_label.config(text="推杆：运行中")
        pushrod_loop()
    else:
        pushrod_status_label.config(text="推杆：已停止")
        # 停止时发一次停止命令
        send_pushrod_cmd("stop")


# ==================== 搭界面 ====================

# ---- 机器人方向控制框 ----
frame_robot = ttk.LabelFrame(root, text="机器人运动控制")
frame_robot.place(x=20, y=20, width=300, height=260)

btn_forward = tk.Button(frame_robot, text="前进", width=8, height=2)
btn_backward = tk.Button(frame_robot, text="后退", width=8, height=2)
btn_left = tk.Button(frame_robot, text="左转", width=8, height=2)
btn_right = tk.Button(frame_robot, text="右转", width=8, height=2)
btn_rotate = tk.Button(frame_robot, text="原地旋转", width=10, height=2)

btn_forward.grid(row=0, column=1, padx=5, pady=5)
btn_left.grid(   row=1, column=0, padx=5, pady=5)
btn_backward.grid(row=1, column=1, padx=5, pady=5)
btn_right.grid(  row=1, column=2, padx=5, pady=5)
btn_rotate.grid( row=2, column=1, padx=5, pady=10)

# 绑定按下/松开事件
btn_forward.bind("<ButtonPress-1>",  lambda e: on_robot_press("forward", e))
btn_forward.bind("<ButtonRelease-1>", lambda e: on_robot_release("forward", e))

btn_backward.bind("<ButtonPress-1>",  lambda e: on_robot_press("backward", e))
btn_backward.bind("<ButtonRelease-1>", lambda e: on_robot_release("backward", e))

btn_left.bind("<ButtonPress-1>",  lambda e: on_robot_press("left", e))
btn_left.bind("<ButtonRelease-1>", lambda e: on_robot_release("left", e))

btn_right.bind("<ButtonPress-1>",  lambda e: on_robot_press("right", e))
btn_right.bind("<ButtonRelease-1>", lambda e: on_robot_release("right", e))

btn_rotate.bind("<ButtonPress-1>",  lambda e: on_robot_press("rotate", e))
btn_rotate.bind("<ButtonRelease-1>", lambda e: on_robot_release("rotate", e))


# ---- 摄像头框 ----
frame_cam = ttk.LabelFrame(root, text="摄像头预览")
frame_cam.place(x=340, y=20, width=530, height=340)

cam_check = tk.Checkbutton(frame_cam, text="开启实时预览", variable=cam_var,
                           command=on_cam_toggle)
cam_check.pack(anchor="w", padx=10, pady=5)

camera_label = tk.Label(frame_cam, text="预览已关闭", width=50, height=18,
                        bg="#333333", fg="white")
camera_label.pack(padx=10, pady=5)


# ---- 推杆控制框 ----
frame_pushrod = ttk.LabelFrame(root, text="推杆控制")
frame_pushrod.place(x=20, y=300, width=300, height=200)

# 方向选择（伸出 / 收回）
tk.Label(frame_pushrod, text="运动方向:").grid(row=0, column=0, padx=5, pady=5, sticky="w")

rb_extend = tk.Radiobutton(frame_pushrod, text="伸出", variable=pushrod_dir_var,
                           value="extend")
rb_retract = tk.Radiobutton(frame_pushrod, text="收回", variable=pushrod_dir_var,
                            value="retract")
rb_extend.grid(row=0, column=1, padx=5, pady=5, sticky="w")
rb_retract.grid(row=0, column=2, padx=5, pady=5, sticky="w")

# 开关：推杆是否运动
pushrod_check = tk.Checkbutton(frame_pushrod, text="推杆运行",
                               variable=pushrod_var, command=on_pushrod_toggle)
pushrod_check.grid(row=1, column=0, columnspan=3, padx=5, pady=10, sticky="w")

pushrod_status_label = tk.Label(frame_pushrod, text="推杆：已停止")
pushrod_status_label.grid(row=2, column=0, columnspan=3, padx=5, pady=5, sticky="w")

# 退出按钮
btn_quit = tk.Button(root, text="退出程序", command=root.destroy, width=12)
btn_quit.place(x=750, y=540)

root.mainloop()
