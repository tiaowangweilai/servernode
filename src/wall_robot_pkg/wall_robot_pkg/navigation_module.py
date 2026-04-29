#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
import numpy as np
import socket
import struct
# from .robot_control import * # 假设这是你的底层串口通讯库
from . import robot_control
# ===================== 全局可调参数 (保持不变) =====================
UDP_IP = "0.0.0.0"
UDP_PORT = 5010
BUFFER_SIZE = 1024

SERIAL_DEV = "/dev/ttyACM0"
SERIAL_BAUD = 115200

XY_TARGET = 40       # 到达阈值 (mm)
AXIS_ALIGN_THRESH = 4
YAW_THRESH = 4       # 角度阈值 (度)
ARRIVED_WAIT_SEC = 1.0

CMD_REPEAT_PER_FRAME = 2
CMD_INTERVAL = 0.02
CONTROL_DT = 0.05
PRINT_DT = 1.0

YAW_ALLOW_FOR_STRAIGHT = 5
YAW_TURN_ENTER = 10
YAW_TURN_EXIT  = 3

# PID 参数
DIST_KP = 10.0; DIST_KI = 0.0; DIST_KD = 0.0; DIST_I_CLAMP = 400.0
YAW_KP = 10.0; YAW_KI = 0.0; YAW_KD = 0.0; YAW_I_CLAMP = 400.0

# PWM 映射参数 (保持不变)
CH1_MID = 1500; CH1_MIN = 1050; CH1_MAX = 1950; CH1_DEADZONE_MIN = 1150; CH1_DEADZONE_MAX = 1800
CH2_MID = 1500; CH2_MIN = 1250; CH2_MAX = 1800; CH2_DEADZONE_MIN = 1300; CH2_DEADZONE_MAX = 1700

# ===================== 辅助与映射函数 (保持不变) =====================

def build_ch1_from_pid(u_yaw: float) -> int:
    raw = CH1_MID - u_yaw
    if u_yaw > 0: pwm = int(np.clip(raw, CH1_MIN, CH1_DEADZONE_MIN))
    elif u_yaw < 0: pwm = int(np.clip(raw, CH1_DEADZONE_MAX, CH1_MAX))
    else: pwm = CH1_MID
    return pwm

def build_ch2_from_pid(u_v: float) -> int:
    if u_v > 0:
        target = CH2_MID + abs(u_v)
        if CH2_MID < target <= CH2_DEADZONE_MAX: target = CH2_DEADZONE_MAX + 1
        pwm = int(np.clip(target, CH2_MID + 1, CH2_MAX))
    elif u_v < 0:
        target = CH2_MID - abs(u_v)
        if CH2_DEADZONE_MIN <= target < CH2_MID: target = CH2_DEADZONE_MIN - 1
        pwm = int(np.clip(target, CH2_MIN, CH2_MID - 1))
    else: pwm = CH2_MID
    return pwm

def compute_signed_yaw_error(current_yaw, target_yaw):
    return (target_yaw - current_yaw + 540) % 360 - 180

def compute_signed_dist_and_heading(x, y, yaw, target_xy):
    dx = target_xy[0] - x; dy = target_xy[1] - y
    dist = np.hypot(dx, dy)
    heading = np.degrees(np.arctan2(dy, dx))
    yaw_err = compute_signed_yaw_error(yaw, heading)
    return dist, yaw_err, heading

def _normalize_deg(a): return (a + 180) % 360 - 180

def decide_turn_direction_from_pwm(ch1: int):
    if ch1 > CH1_MID: return "RIGHT"
    if ch1 < CH1_MID: return "LEFT"
    return "STOP"

def choose_axis_from_yaw(tgt_yaw: float):
    yaw_norm = (_normalize_deg(tgt_yaw) + 360) % 360
    candidates = [0, 90, 180, 270]
    nearest = min(candidates, key=lambda a: min(abs(yaw_norm - a), 360 - abs(yaw_norm - a)))
    axis = "x" if nearest in (0, 180) else "y"
    return axis, nearest

def angle_diff_abs(a, b): return abs(_normalize_deg(b - a))

# ===================== PID 类 =====================
class PID:
    def __init__(self, kp, ki, kd, i_clamp=1e9):
        self.kp = kp; self.ki = ki; self.kd = kd; self.i_clamp = abs(i_clamp)
        self.reset()
    def reset(self):
        self.i_term = 0.0; self.prev_e = None
    def step(self, e, dt):
        p = self.kp * e
        self.i_term += self.ki * e * dt
        self.i_term = float(np.clip(self.i_term, -self.i_clamp, self.i_clamp))
        d = 0.0
        if self.prev_e is not None: d = self.kd * (e - self.prev_e)
        self.prev_e = e
        return p + self.i_term + d

# ===================== 导航核心类 =====================

class RobotNavigator:
    def __init__(self):
        self.pose_result = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.pose_lock = threading.Lock()
        self.stop_flag = threading.Event()
        self.thread = None
        self.ser = None
        
        # PID控制器
        self.dist_pid = PID(DIST_KP, DIST_KI, DIST_KD, i_clamp=DIST_I_CLAMP)
        self.yaw_pid = PID(YAW_KP, YAW_KI, YAW_KD, i_clamp=YAW_I_CLAMP)

    def start(self):
        """启动：打开串口，开启监听线程"""
        if self.ser is None:
            try:
                print(f"[NAV] 打开底盘串口 {SERIAL_DEV} ...")
                self.ser = robot_control.open_port(SERIAL_DEV, SERIAL_BAUD)
            except Exception as e:
                print(f"[NAV][ERROR] 串口打开失败: {e}")
                raise e
        
        if self.thread is None or not self.thread.is_alive():
            self.stop_flag.clear()
            self.thread = threading.Thread(target=self._pose_listener, daemon=True)
            self.thread.start()
            print("[NAV] 导航系统已就绪 (UDP监听中)")

    def close(self):
        """关闭：停止线程，关闭串口"""
        print("[NAV] 系统关闭中...")
        self.stop_flag.set()
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.ser:
            try:
                robot_control.send_frame(self.ser, robot_control.STOP)
                self.ser.close()
            except:
                pass
            self.ser = None
        print("[NAV] 系统已关闭")

    def get_current_pose(self):
        with self.pose_lock:
            return self.pose_result.copy()

    def _pose_listener(self):
        """内部方法：后台UDP监听"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
        sock.settimeout(1.0) # 避免死锁
        
        while not self.stop_flag.is_set():
            try:
                data, _ = sock.recvfrom(BUFFER_SIZE)
                if len(data) < 56: continue
                payload = data[16:56]
                x_mm, y_mm, yaw_deg_raw = struct.unpack(">QQqqiBBH", payload)[2:5]
                yaw_deg = yaw_deg_raw / 1000.0
                
                with self.pose_lock:
                    self.pose_result = {"x": float(x_mm), "y": float(y_mm), "yaw": float(yaw_deg)}
            except socket.timeout:
                pass
            except Exception as e:
                print(f"[POSE][ERROR] {e}")
        sock.close()

    def navigate_to(self, tgt_x, tgt_y, tgt_yaw):
        """
        核心导航函数：阻塞直到到达目标点
        返回: True (成功到达), False (被中断或失败)
        """
        if not self.ser:
            print("[NAV][ERROR] 请先调用 start()!")
            return False

        tgt_xy = (tgt_x, tgt_y)
        print("\n" + "=" * 60)
        print(f"[NAV] 开始导航 -> xy=({tgt_x:.1f},{tgt_y:.1f}) yaw={tgt_yaw:.1f}°")
        print("=" * 60)

        self.dist_pid.reset()
        self.yaw_pid.reset()

        # FSM 初始状态
        state = "APPROACH"
        approach_submode = "ROTATE"
        locked_near_target = False
        last_t = time.time()
        last_print = 0.0

        # 头尾自适应判断 (Head/Tail Logic)
        with self.pose_lock:
            curr = self.pose_result
        
        dx0 = tgt_x - curr['x']
        dy0 = tgt_y - curr['y']
        heading0 = np.degrees(np.arctan2(dy0, dx0))
        heading0 = _normalize_deg(heading0)
        heading_tail0 = _normalize_deg(heading0 + 180.0)
        
        cost_head = angle_diff_abs(curr['yaw'], heading0) + angle_diff_abs(heading0, tgt_yaw)
        cost_tail = angle_diff_abs(curr['yaw'], heading_tail0) + angle_diff_abs(heading_tail0, tgt_yaw)
        approach_mode = "HEAD" if cost_head <= cost_tail else "TAIL"
        
        print(f"[NAV] 模式: {approach_mode} (代价 H:{cost_head:.0f} vs T:{cost_tail:.0f})")

        # ---------------- 导航主循环 ----------------
        while not self.stop_flag.is_set():
            loop_start = time.time()
            now = time.time()
            dt = max(1e-3, now - last_t)
            last_t = now

            with self.pose_lock:
                x = self.pose_result["x"]
                y = self.pose_result["y"]
                yaw = self.pose_result["yaw"]

            # 计算误差
            dist, yaw_err_head, heading = compute_signed_dist_and_heading(x, y, yaw, tgt_xy)
            yaw_err_final = compute_signed_yaw_error(yaw, tgt_yaw)

            if approach_mode == "TAIL":
                yaw_err_to_target = compute_signed_yaw_error(yaw, _normalize_deg(heading + 180.0))
            else:
                yaw_err_to_target = yaw_err_head

            frame = robot_control.STOP

            # ================= FSM 状态机 =================
            if state == "APPROACH" and not locked_near_target:
                if dist <= XY_TARGET:
                    state = "YAW_ALIGN"
                    self.yaw_pid.reset()
                    continue

                abs_yaw = abs(yaw_err_to_target)
                
                # 迟滞切换
                if approach_submode == "ROTATE":
                    if abs_yaw < YAW_TURN_EXIT: approach_submode = "MOVE"
                else:
                    if abs_yaw > YAW_TURN_ENTER: approach_submode = "ROTATE"

                if approach_submode == "ROTATE":
                    u_yaw = self.yaw_pid.step(yaw_err_to_target, dt)
                    ch1 = build_ch1_from_pid(u_yaw)
                    turn_dir = decide_turn_direction_from_pwm(ch1)
                    if turn_dir == "RIGHT": frame = robot_control.build_turn_right(ch1)
                    elif turn_dir == "LEFT": frame = robot_control.build_turn_left(ch1)
                else:
                    # MOVE
                    u_v = self.dist_pid.step(dist, dt)
                    if approach_mode == "TAIL": u_v = -abs(u_v) # 倒车
                    ch2 = build_ch2_from_pid(u_v)
                    if ch2 > CH2_MID: frame = robot_control.build_forward(ch2)
                    elif ch2 < CH2_MID: frame = robot_control.build_backward(ch2)

            elif state == "YAW_ALIGN":
                if abs(yaw_err_final) <= YAW_THRESH:
                    state = "AXIS_ALIGN"
                    self.axis_choice, self.axis_nearest = choose_axis_from_yaw(tgt_yaw)
                    # 预计算前进符号
                    if self.axis_choice == "x":
                        self.axis_fwd_sign = +1 if self.axis_nearest == 0 else -1
                    else:
                        self.axis_fwd_sign = +1 if self.axis_nearest == 90 else -1
                    self.dist_pid.reset()
                    continue
                
                u_yaw = self.yaw_pid.step(yaw_err_final, dt)
                ch1 = build_ch1_from_pid(u_yaw)
                turn_dir = decide_turn_direction_from_pwm(ch1)
                if turn_dir == "RIGHT": frame = robot_control.build_turn_right(ch1)
                elif turn_dir == "LEFT": frame = robot_control.build_turn_left(ch1)

            elif state == "AXIS_ALIGN":
                # 计算单轴误差
                err_x = tgt_x - x
                err_y = tgt_y - y
                axis_err = err_x if self.axis_choice == "x" else err_y

                if abs(axis_err) <= AXIS_ALIGN_THRESH:
                    state = "YAW_FINE"
                    self.yaw_pid.reset()
                    continue

                # 决定是物理前进还是后退
                sign_err = 1 if axis_err > 0 else -1
                desired_motion = "FWD" if sign_err == self.axis_fwd_sign else "BWD"
                
                u_mag = abs(self.dist_pid.step(abs(axis_err), dt))
                u_v = +u_mag if desired_motion == "FWD" else -u_mag
                ch2 = build_ch2_from_pid(u_v)
                
                if ch2 > CH2_MID: frame = robot_control.build_forward(ch2)
                elif ch2 < CH2_MID: frame = robot_control.build_backward(ch2)

            elif state == "YAW_FINE":
                if abs(yaw_err_final) <= YAW_THRESH:
                    # 1. 先停车
                    robot_control.send_frame(self.ser, robot_control.STOP, repeat=CMD_REPEAT_PER_FRAME)
                    
                    # 2. 暂停确认
                    print(f"[NAV] 预到达，等待确认... (err={yaw_err_final:.2f})")
                    time.sleep(ARRIVED_WAIT_SEC)
                    
                    # 3. 二次检查
                    with self.pose_lock:
                        x2, y2, yaw2 = self.pose_result["x"], self.pose_result["y"], self.pose_result["yaw"]
                    
                    dist2, _, _ = compute_signed_dist_and_heading(x2, y2, yaw2, tgt_xy)
                    yaw_final2 = compute_signed_yaw_error(yaw2, tgt_yaw)
                    
                    if dist2 < XY_TARGET and abs(yaw_final2) < YAW_THRESH:
                        print(f"[NAV] √ 成功到达: ({x2:.1f},{y2:.1f}) 误差: {dist2:.1f}mm")
                        return True # 返回成功
                    else:
                        print(f"[NAV] × 确认失败: 漂移 dist={dist2:.1f}mm, 重新精调")
                        state = "AXIS_ALIGN"
                        locked_near_target = True
                        self.dist_pid.reset()
                        continue

                u_yaw = self.yaw_pid.step(yaw_err_final, dt)
                ch1 = build_ch1_from_pid(u_yaw)
                turn_dir = decide_turn_direction_from_pwm(ch1)
                if turn_dir == "RIGHT": frame = robot_control.build_turn_right(ch1)
                elif turn_dir == "LEFT": frame = robot_control.build_turn_left(ch1)

            # 发送指令
            robot_control.send_frame(self.ser, frame, repeat=CMD_REPEAT_PER_FRAME, interval=CMD_INTERVAL)
            
            # 打印状态
            if time.time() - last_print > PRINT_DT:
                print(f"[LOG] State={state} | Dist={dist:.0f} | YawErr={yaw_err_final:.1f}")
                last_print = time.time()
            
            # 维持控制周期
            time.sleep(max(0.0, CONTROL_DT - (time.time() - now)))

        return False # 如果循环被 stop_flag 打断