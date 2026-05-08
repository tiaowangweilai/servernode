#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Point, PoseStamped
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import math
import time

from cv_bridge import CvBridge
import json
from . import Path_interpolation

# 🌟 删除了所有硬件库导入 (GPIO, IG35, motor_485)

# === 导航常量 ===
XY_TARGET = 30.0            
AXIS_ALIGN_THRESH = 5.0     
YAW_THRESH = 3.0            
ARRIVED_WAIT_SEC = 0.2
YAW_TURN_ENTER = 8.0
YAW_TURN_EXIT  = 2.0

def compute_signed_yaw_error(current_yaw, target_yaw): return (target_yaw - current_yaw + 540) % 360 - 180
def _normalize_deg(a): return (a + 180) % 360 - 180
def angle_diff_abs(a, b): return abs(_normalize_deg(b - a))

def choose_axis_from_yaw(tgt_yaw: float):
    yaw_norm = (_normalize_deg(tgt_yaw) + 360) % 360
    candidates = [0, 90, 180, 270]
    nearest = min(candidates, key=lambda a: min(abs(yaw_norm - a), 360 - abs(yaw_norm - a)))
    return ("x" if nearest in (0, 180) else "y"), nearest

class PID:
    def __init__(self, kp, ki, kd, i_clamp=400.0):
        self.kp = kp; self.ki = ki; self.kd = kd; self.i_clamp = abs(i_clamp)
        self.reset()
    def reset(self): self.i_term = 0.0; self.prev_e = None
    def step(self, e, dt):
        p = self.kp * e
        self.i_term += self.ki * e * dt
        self.i_term = float(np.clip(self.i_term, -self.i_clamp, self.i_clamp))
        d = 0.0 if self.prev_e is None else self.kd * (e - self.prev_e)
        self.prev_e = e
        return p + self.i_term + d

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller_node')
        
        self.control_freq = 20.0
        self.fine_align_start_time = 0.0 
        self.dist_pid = PID(5.0, 0.0, 25.0)
        self.yaw_pid = PID(15.0, 0.0, 35.0)
        
        self.bridge = CvBridge()
        self.latest_depth = None
        
        # === 1. 订阅端 (纯 ROS 话题) ===
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, qos_profile)
        self.click_sub = self.create_subscription(Point, '/mission/click', self.click_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.manual_sub = self.create_subscription(Twist, '/cmd_vel_manual', self.manual_override_check, 10)
        
        self.cmd_sub = self.create_subscription(String, '/mission/sys_command', self.command_callback, 10)
        self.auto_cmd_sub = self.create_subscription(String, '/cmd_vel_auto', self.auto_cmd_callback, 10)
        self.nav_goal_sub = self.create_subscription(Point, '/mission/nav_goal', self.nav_goal_callback, 10)
        self.motor_cmd_sub = self.create_subscription(Point, '/mission/motor_cmd', self.motor_cmd_callback, 10)
        self.params_sub = self.create_subscription(String, '/mission/params', self.params_callback, 10)

        # === 2. 发布端 (纯 ROS 话题) ===
        self.cmd_pub = self.create_publisher(String, '/cmd_vel_auto', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.target_idx_pub = self.create_publisher(Int32, '/mission/target_idx', 10)
        self.path_pub = self.create_publisher(Path, '/mission/planned_path', 10)
        self.event_pub = self.create_publisher(String, '/mission/events', 10)
        
        # 🌟 新增：对底层机构驱动节点发号施令的 Publisher
        self.m1_pub = self.create_publisher(Int32, '/mech/m1_target', 10)
        self.m2_pub = self.create_publisher(Int32, '/mech/m2_target', 10)
        self.ig35_pub = self.create_publisher(Int32, '/mech/ig35_target', 10)
        self.ig35_speed_pub = self.create_publisher(Int32, '/mech/ig35_speed', 10)
        self.pushrod_pub = self.create_publisher(Int32, '/mech/push_rod_time', 10)

        # 状态机变量
        self.current_pose = None
        self.targets = []
        self.state = "IDLE" 
        self.nav_submode = "ROTATE"
        self.approach_dir = "HEAD" 
        self.locked_near_target = False
        self.is_click_nav = False 
        
        self.current_target_idx = 0      
        self.current_col_point_idx = 0   
        self.capture_authorized = False
        self.col_start_initialized = False 
        self.last_time = self.get_clock().now()
        self.param_width, self.param_height, self.param_acc = 2000.0, 2000.0, 80.0
        
        self.ig35_start_pulse = 11
        self.ig35_end_pulse = 0
        self.scan_speed = 20

        self.timer = self.create_timer(1.0/self.control_freq, self.control_loop)
        
        self.get_logger().info(">>> 主控节点就绪 (✅ 大脑已解耦，不含任何阻塞型硬件驱动代码)")


    def command_callback(self, msg):
        import json as _json
        raw = msg.data
        
        # 1️⃣ 打印收到的最原始字符串
        self.get_logger().info(f"📥 [主控] 收到 command 话题数据: {raw}")
        
        try:
            parsed = _json.loads(raw)
        except Exception:
            # pure string cmd, not JSON
            parsed = {}

        # ==========================================
        # 🌟 智能提取指令引擎：自动穿透 payload, lidar 或 agv
        # ==========================================
        cmd = ""
        params = {} # 统一把参数提取到这个字典里
        
        payload = parsed.get("payload", {})
        if "lidar" in payload and "command" in payload["lidar"]:
            cmd = payload["lidar"].get("command")
            params = payload["lidar"]
        elif "agv" in payload and "command" in payload["agv"]:
            cmd = payload["agv"].get("command")
            params = payload["agv"]
        elif "command" in parsed:
            cmd = parsed.get("command")
            params = parsed
        else:
            cmd = raw
            params = parsed

        if cmd and cmd != raw:
            self.get_logger().info(f"🔍 [主控] 智能提取到指令: {cmd}")

        # ==========================================
        # 执行指令分支
        # ==========================================
        if cmd == "nav_path":
            w = float(params.get("target_x", self.param_width))
            h = float(params.get("target_y", self.param_height))
            
            spacing_raw = float(params.get("push_accuracy", 0.08))
            spacing = spacing_raw * 1000.0 if spacing_raw < 10.0 else spacing_raw 
            
            self.param_width = w
            self.param_height = h
            self.param_acc = spacing
            
            self.ig35_start_pulse = int(params.get("ig35_start", self.ig35_start_pulse))
            self.ig35_end_pulse = int(params.get("ig35_end", self.ig35_end_pulse))
            self.scan_speed = int(params.get("scan_speed", self.scan_speed))
            
            self.get_logger().info(f"🎯 [主控] 开始规划自动导航! 宽={w}mm, 高={h}mm, 间距={spacing}mm, 速度={self.scan_speed}rpm")
            
            self.is_click_nav = False
            self.generate_path(w, h, self.param_acc)
            
            self.get_logger().info(f"🗺️ [主控] 路径规划完成！共生成 {len(self.targets)} 个目标点。发布至 /mission/planned_path")
            
            if self.targets:
                self.current_target_idx = 0 
                self.current_col_point_idx = 0 
                self.capture_authorized = False 
                self.col_start_initialized = False 
                self.prepare_next_target()
                self._send_agv_event("nav_started")

        elif cmd == "single_scan":
            self.ig35_start_pulse = int(params.get("ig35_start", self.ig35_start_pulse))
            self.ig35_end_pulse = int(params.get("ig35_end", self.ig35_end_pulse))
            self.scan_speed = int(params.get("scan_speed", self.scan_speed))
            
            self.get_logger().info(f"🛠️ [主控] 触发 single_scan: start={self.ig35_start_pulse} end={self.ig35_end_pulse} speed={self.scan_speed}")
            
            # 强制切入手动测试模式，交接给 control_loop 执行
            self.state = "MANUAL_ACTION"

        elif cmd == "emergency_stop":
            self.state = "IDLE"
            self.get_logger().warn("🚨 emergency_stop received")
            
        elif cmd == "capture_successed":
            self.capture_authorized = True
            if self.state == "WAIT_CAPTURE":
                self.get_logger().info("🔓 capture authorized")
                self.state = "ACTION"

        elif cmd == "save_successed":
            self.get_logger().info("[主控] save confirmed by server")

        elif cmd == "work_complete":
            self.get_logger().info("[主控] work_complete confirmed by server")
                
        elif cmd == "update_status":
            status = params.get("status", "unknown")
            source = params.get("source_key", "unknown")
            self.get_logger().info(f"闸门状态更新: [{source}] status={status}")
    def nav_goal_callback(self, msg):
        self.param_width = msg.x
        self.param_height = msg.y
        self.is_click_nav = False
        if self.targets:
            self.current_target_idx = 0 
            self.current_col_point_idx = 0 
            self.capture_authorized = False 
            self.col_start_initialized = False 
            self.prepare_next_target()

    def auto_cmd_callback(self, msg):
        import json as _json
        raw = msg.data
        
        try:
            parsed = _json.loads(raw)
        except Exception:
            return  # 如果不是标准的 JSON，直接忽略
            
        # 🛡️ 过滤：如果是主控自己发布的底盘速度控制指令，或者是手柄发的指令，直接忽略
        if "vx" in parsed or "wz" in parsed or "x" in parsed:
            return

        cmd = parsed.get("command", "")
        
        # 🎯 拦截到导航指令！
        if cmd == "nav_path":
            self.get_logger().info(f"📥 [主控] 从 /cmd_vel_auto 拦截到导航指令: {raw}")
            
            # 因为 C++ 是直接透传 data，所以直接从 parsed 提取即可
            w = float(parsed.get("target_x", self.param_width))
            h = float(parsed.get("target_y", self.param_height))
            
            # 单位转换：如果精度数字小于10，判断为米(m)，转为底层的毫米(mm)
            spacing_raw = float(parsed.get("push_accuracy", 0.08))
            spacing = spacing_raw * 1000.0 if spacing_raw < 10.0 else spacing_raw 
            
            self.param_width = w
            self.param_height = h
            self.param_acc = spacing
            
            self.ig35_start_pulse = int(parsed.get("ig35_start", self.ig35_start_pulse))
            self.ig35_end_pulse = int(parsed.get("ig35_end", self.ig35_end_pulse))
            self.scan_speed = int(parsed.get("scan_speed", self.scan_speed))
            
            self.get_logger().info(f"🎯 [主控] 开始规划自动导航! 宽={w}mm, 高={h}mm, 间距={spacing}mm, 扫查速度={self.scan_speed}rpm")
            
            self.is_click_nav = False
            self.generate_path(w, h, self.param_acc)
            
            point_count = len(self.targets)
            self.get_logger().info(f"🗺️ [主控] 路径规划完成！共生成 {point_count} 个目标点。发布至 /mission/planned_path")
            
            if self.targets:
                self.current_target_idx = 0 
                self.current_col_point_idx = 0 
                self.capture_authorized = False 
                self.col_start_initialized = False 
                self.prepare_next_target()
                self._send_agv_event("nav_started")

    def motor_cmd_callback(self, msg):
        motor_id = int(msg.x)
        pulse = int(msg.y)
        if motor_id == 1: 
            self.m1_pub.publish(Int32(data=pulse))
        elif motor_id == 2: 
            self.m2_pub.publish(Int32(data=pulse))

    def _send_agv_event(self, cmd_str):
        msg = String(); msg.data = cmd_str
        self.event_pub.publish(msg)

    def depth_callback(self, msg):
        try: self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except: pass

    def click_callback(self, msg):
        # 原有视觉反算点击坐标逻辑保持不变
        if self.latest_depth is None: return
        u, v = int(msg.x), int(msg.y)
        if u < 0 or u >= self.latest_depth.shape[1] or v < 0 or v >= self.latest_depth.shape[0]: return
        z_c = self.latest_depth[v, u] * 0.001
        if z_c <= 0.001:
            window = self.latest_depth[max(0, v-2):min(self.latest_depth.shape[0], v+3), max(0, u-2):min(self.latest_depth.shape[1], u+3)]
            valid_depths = window[window > 0]
            if len(valid_depths) > 0: z_c = np.median(valid_depths) * 0.001
            else: return

        fx, fy = 434.663696, 433.539428; cx, cy = 421.902587, 243.643188
        x_c = (u - cx) * z_c / fx; y_c = (v - cy) * z_c / fy
        T_cam_to_lidar = np.array([[0., 0., 1., 0.], [-1., 0., 0., 0.], [0., -1., 0., 0.1], [0., 0., 0., 1.]])
        P_cam = np.array([x_c, y_c, z_c, 1.0])
        P_lidar = T_cam_to_lidar @ P_cam
        x_l, y_l = P_lidar[0], P_lidar[1]

        if self.current_pose is None: return
        rob_x, rob_y = self.current_pose['x'], self.current_pose['y']
        rob_yaw_rad = math.radians(self.current_pose['yaw'])
        target_odom_x = rob_x + x_l * math.cos(rob_yaw_rad) - y_l * math.sin(rob_yaw_rad)
        target_odom_y = rob_y + x_l * math.sin(rob_yaw_rad) + y_l * math.cos(rob_yaw_rad)
        target_yaw_deg = math.degrees(math.atan2(target_odom_y - rob_y, target_odom_x - rob_x))

        self.is_click_nav = True
        self.targets = [{"x": target_odom_x, "y": target_odom_y, "yaw": target_yaw_deg, "type": "NORMAL"}]
        self.publish_planned_path()
        self.current_target_idx = 0 
        self.prepare_next_target()

    def manual_override_check(self, msg):
        if self.state != "IDLE":
            self.get_logger().warn(f"🚨 [主控] 手动干预，状态 {self.state} -> IDLE")
            self.state = "IDLE" 

    def prepare_next_target(self):
        if self.current_target_idx >= len(self.targets):
            self.state = "IDLE"; return
        self.state = "NAV_APPROACH"
        self.nav_submode = "ROTATE"
        self.locked_near_target = False
        self.fine_align_start_time = 0.0
        self.col_start_initialized = False 
        self.dist_pid.reset(); self.yaw_pid.reset()
        
        tgt = self.targets[self.current_target_idx]
        curr = self.current_pose
        if not curr: return
        dx = tgt['x']*1000 - curr['x']*1000; dy = tgt['y']*1000 - curr['y']*1000
        heading = _normalize_deg(math.degrees(math.atan2(dy, dx)))
        cost_head = angle_diff_abs(curr['yaw'], heading) + angle_diff_abs(heading, tgt['yaw'])
        cost_tail = angle_diff_abs(curr['yaw'], _normalize_deg(heading + 180.0)) + angle_diff_abs(_normalize_deg(heading + 180.0), tgt['yaw'])
        self.approach_dir = "HEAD" if cost_head <= cost_tail else "TAIL"

    def generate_path(self, w, h, dy):
        self.targets = []
        try:
            grid_top, grid_bottom, grid_third = Path_interpolation.main_grid_interpolation(w, h, dy)
            num_cols = grid_top.shape[0]; num_rows = grid_top.shape[1]
            for i in range(num_cols):
                for j in range(num_rows):
                    pt = grid_top[i, j]; pt_type = "NORMAL"
                    if j == 0: pt_type = "COL_START"
                    if j == num_rows - 1: pt_type = "COL_END"
                    if i == 0 and j == 0: pt_type = "REGION_START"
                    if i == num_cols - 1 and j == num_rows - 1: pt_type = "REGION_END"
                    self.targets.append({"x": pt[0]/1000.0, "y": pt[1]/1000.0, "yaw": 90.0, "type": pt_type})
            for k, pt in enumerate(grid_bottom):
                pt_type = "NORMAL"
                if k == 0: pt_type = "REGION_START"
                if k == len(grid_bottom) - 1: pt_type = "REGION_END"
                self.targets.append({"x": pt[0]/1000.0, "y": pt[1]/1000.0, "yaw": 0.0, "type": pt_type})
            for k, pt in enumerate(grid_third):
                pt_type = "NORMAL"
                if k == 0: pt_type = "REGION_START"
                if k == len(grid_third) - 1: pt_type = "REGION_END"
                self.targets.append({"x": pt[0]/1000.0, "y": pt[1]/1000.0, "yaw": 180.0, "type": pt_type})
            self.publish_planned_path()
        except: pass

    def publish_planned_path(self):
        if not self.targets: return
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for tgt in self.targets:
            pose = PoseStamped(); pose.header = path_msg.header
            pose.pose.position.x = float(tgt['x']); pose.pose.position.y = float(tgt['y'])
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x; y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.degrees(math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2)))
        self.current_pose = {"x": x, "y": y, "yaw": yaw}
        
        state_msg = String(); state_msg.data = self.state
        self.state_pub.publish(state_msg)
        idx_msg = Int32(); idx_msg.data = self.current_target_idx
        self.target_idx_pub.publish(idx_msg)

    def control_loop(self):
        if self.state == "MANUAL_ACTION":
            self.cmd_pub.publish(String(data=json.dumps({"vx":0,"wz":0}))); self.do_manual_action(); return
        if self.current_pose is None or not self.targets: return
        now_ts = self.get_clock().now()
        dt = max(1e-3, (now_ts - self.last_time).nanoseconds / 1e9)
        self.last_time = now_ts
        if self.state == "WAIT_CAPTURE": self.cmd_pub.publish(String(data=json.dumps({"vx":0,"wz":0}))); return
        if "NAV_" in self.state: self.do_navigation_fsm(dt)
        elif self.state == "ACTION": self.cmd_pub.publish(String(data=json.dumps({"vx":0,"wz":0}))); self.do_action()

    def do_navigation_fsm(self, dt):
        tgt = self.targets[self.current_target_idx]
        curr = self.current_pose
        dx = tgt['x']*1000 - curr['x']*1000; dy = tgt['y']*1000 - curr['y']*1000
        dist = math.hypot(dx, dy); heading = math.degrees(math.atan2(dy, dx))
        yaw_err_final = compute_signed_yaw_error(curr['yaw'], tgt['yaw'])
        u_v, u_yaw = 0.0, 0.0

        if self.state == "NAV_APPROACH" and not self.locked_near_target:
            if dist <= XY_TARGET: self.state = "NAV_YAW_ALIGN"; return
            target_h = _normalize_deg(heading + 180.0) if self.approach_dir == "TAIL" else heading
            yaw_err_to_target = compute_signed_yaw_error(curr['yaw'], target_h)
            if self.nav_submode == "ROTATE":
                if abs(yaw_err_to_target) < YAW_TURN_EXIT: self.nav_submode = "MOVE"
                u_yaw = self.yaw_pid.step(yaw_err_to_target, dt)
            else:
                if abs(yaw_err_to_target) > YAW_TURN_ENTER: self.nav_submode = "ROTATE"
                u_v = self.dist_pid.step(dist, dt)
                speed_limit = max(40.0, dist * 1.5) 
                u_v = float(np.clip(u_v, -speed_limit, speed_limit))
                if self.approach_dir == "TAIL": u_v = -abs(u_v)

        elif self.state == "NAV_YAW_ALIGN":
            if abs(yaw_err_final) <= YAW_THRESH:
                self.state = "NAV_AXIS_ALIGN"
                self.axis_choice, self.axis_nearest = choose_axis_from_yaw(tgt['yaw'])
                if self.axis_choice == "x": self.axis_fwd_sign = 1 if self.axis_nearest == 0 else -1
                else: self.axis_fwd_sign = 1 if self.axis_nearest == 90 else -1
                self.dist_pid.reset(); return
            u_yaw = self.yaw_pid.step(yaw_err_final, dt)

        elif self.state == "NAV_AXIS_ALIGN":
            axis_err = (tgt['x']*1000 - curr['x']*1000) if self.axis_choice == "x" else (tgt['y']*1000 - curr['y']*1000)
            if abs(axis_err) <= AXIS_ALIGN_THRESH: self.state = "NAV_YAW_FINE"; return
            sign_err = 1 if axis_err > 0 else -1
            desired_motion = "FWD" if sign_err == self.axis_fwd_sign else "BWD"
            u_mag = abs(self.dist_pid.step(abs(axis_err), dt))
            u_v = u_mag if desired_motion == "FWD" else -u_mag

        elif self.state == "NAV_YAW_FINE":
            if abs(yaw_err_final) <= YAW_THRESH:
                if self.fine_align_start_time == 0.0: self.fine_align_start_time = time.time()
                if time.time() - self.fine_align_start_time >= ARRIVED_WAIT_SEC:
                    self.state = "ACTION"; self.fine_align_start_time = 0.0; return
                u_v, u_yaw = 0.0, 0.0
            else:
                self.fine_align_start_time = 0.0
                u_yaw = self.yaw_pid.step(yaw_err_final, dt)

        vx_cmd = float(np.clip(u_v / 500.0, -0.4, 0.4))
        wz_cmd = float(np.clip(u_yaw / 500.0, -0.6, 0.6))
        self.cmd_pub.publish(String(data=json.dumps({"vx":vx_cmd,"wz":wz_cmd})))

    # ==========================================
    # 🌟 物理序列改用 Topic 下发，大脑彻底解放！
    # ==========================================
    def _execute_auto_scan_sequence(self, is_outward):
        try:
            self.get_logger().info("▶️ 1. 下发：M1 电机下压...")
            self.m1_pub.publish(Int32(data=300))
            time.sleep(1.5)
            
            if is_outward:
                self.get_logger().info(f"▶️ 2. 下发：横杆扫出 (至 {self.ig35_start_pulse})...")
                self.ig35_speed_pub.publish(Int32(data=self.scan_speed))
                self.ig35_pub.publish(Int32(data=self.ig35_start_pulse))
                time.sleep(3.0) 
            else:
                self.get_logger().info(f"▶️ 2. 下发：横杆扫回 (至 {self.ig35_end_pulse})...")
                self.ig35_speed_pub.publish(Int32(data=self.scan_speed))
                self.ig35_pub.publish(Int32(data=self.ig35_end_pulse))
                time.sleep(2.5)

            self.get_logger().info("⚙️ 3. 下发：触发虚拟轴...")
            self.pushrod_pub.publish(Int32(data=500))
            time.sleep(0.6)
            
            self.get_logger().info("◀️ 4. 下发：M1 电机抬起...")
            self.m1_pub.publish(Int32(data=0))
            time.sleep(0.5)
        except Exception as e:
            self.get_logger().error(f"❌ 序列下发崩溃: {e}")
    def _execute_single_scan_sequence(self):
            try:
                self.get_logger().info("▶️ [单点测试] 下发 M1 下压...")
                self.m1_pub.publish(Int32(data=300))
                time.sleep(1.5)
                
                self.get_logger().info(f"▶️ [单点测试] 下发 横杆扫出...")
                self.ig35_speed_pub.publish(Int32(data=self.scan_speed+120))
                time.sleep(0.1) # 🌟 必须加延时！让底层有时间把速度设进去
                self.ig35_pub.publish(Int32(data=self.ig35_start_pulse))
                time.sleep(3.0)
                self.pushrod_pub.publish(Int32(data=500))
                time.sleep(0.6)
                
                self.get_logger().info(f"◀️ [单点测试] 下发 横杆扫回...")
                self.ig35_speed_pub.publish(Int32(data=self.scan_speed+120))
                time.sleep(0.1) # 🌟 必须加延时！
                self.ig35_pub.publish(Int32(data=self.ig35_end_pulse))
                time.sleep(2.5)
                self.pushrod_pub.publish(Int32(data=500))
                time.sleep(0.6)
                
                self.get_logger().info("◀️ [单点测试] 下发 M1 抬起...")
                self.m1_pub.publish(Int32(data=0))
                time.sleep(0.5)
            except Exception as e:
                self.get_logger().error(f"❌ 单次序列崩溃: {e}")

    def do_action(self):
        if self.is_click_nav:
            self.state = "IDLE"; return 

        if self.current_target_idx < len(self.targets):
            tgt_info = self.targets[self.current_target_idx]
            pt_type = tgt_info.get("type", "NORMAL")
            
            if pt_type in ["COL_START", "REGION_START"]:
                if not getattr(self, 'col_start_initialized', False):
                    self.get_logger().info("📌 ===== 开始列首初始化 =====")
                    self.m1_pub.publish(Int32(data=0))
                    time.sleep(0.5)
                    if self.current_target_idx == 0:
                        self._send_agv_event("capture")  
                    self.col_start_initialized = True
                    self.current_col_point_idx = 0
                
                if not self.capture_authorized:
                    if self.state != "WAIT_CAPTURE":
                        self.state = "WAIT_CAPTURE"
                        self.get_logger().info("⏳ 挂起等待解冻信号...")
                    return 
            
            self.get_logger().info(f"🎯 授权通过！执行作业: 总第 {self.current_target_idx + 1} 点")
            is_outward = (self.current_col_point_idx % 2 == 0)
            self._execute_auto_scan_sequence(is_outward)
            self.current_col_point_idx += 1 
                
            if pt_type in ["COL_END", "REGION_END"]:
                self.get_logger().info(f"🏁 本列结束，发 save！开往下一列")
                self._send_agv_event("save") 
                self.capture_authorized = False
                self.ig35_pub.publish(Int32(data=0)) # 横杆回零

            if self.current_target_idx == len(self.targets) - 1:
                self.get_logger().info(f"🎉 全部扫完！发 work_complete！")
                self._send_agv_event("work_complete") 
                self.state = "IDLE"
                self.current_target_idx += 1
                return
          
        self.current_target_idx += 1
        self.prepare_next_target()


    def params_callback(self, msg):
        try:
            raw = json.loads(msg.data)
            if isinstance(raw, list) and len(raw) > 0:
                item = raw[0]
                target = item.get("target", "")
                data = item.get("data", {})
                cmd_type = data.get("command", "")
                if cmd_type == "nav_start" and target == "agv":
                    w = float(data.get("width", self.param_width))
                    h = float(data.get("height", self.param_height))
                    spacing = float(data.get("spacing", self.param_acc))
                    self.param_width = w
                    self.param_height = h
                    self.param_acc = spacing
                    self.is_click_nav = False
                    self.get_logger().info(f"nav_start: w={w}, h={h}, spacing={spacing}")
                    self.generate_path(w, h, spacing)
                    if self.targets:
                        self.current_target_idx = 0
                        self.current_col_point_idx = 0
                        self.capture_authorized = False
                        self.col_start_initialized = False
                        self.prepare_next_target()
                        self._send_agv_event("nav_started")
        except Exception as e:
            self.get_logger().error(f"params_callback error: {e}")

    def do_manual_action(self):
        self._execute_single_scan_sequence()
        self.state = "IDLE"

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()