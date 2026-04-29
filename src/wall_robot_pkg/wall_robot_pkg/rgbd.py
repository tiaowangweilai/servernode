#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
orbbec_easy.py
一键即用的 Orbbec Femto Mega 封装：
- 调 display / save 函数即可自动初始化和采集
- 内置默认分辨率（可改），不需要提前写任何初始化代码
- 自动清理资源，避免 /dev/videoX busy
"""

import os
import time
import atexit
import cv2
import numpy as np
import json
from pyorbbecsdk import (
    Context, Pipeline, Config,
    OBSensorType, OBFormat
)
from pyorbbecsdk import (
    AlignFilter, PointCloudFilter, OBStreamType,
    save_point_cloud_to_ply
)
# ========================= 基础工具 =========================
# ========================= 对外：显示函数 =========================

def get_rgb_frame():
    """
    给上位机用：返回一帧彩色图像 (BGR, np.ndarray)，失败时返回 None
    """
    color, _, _ = _CAPTURE.get_frame_once()
    return color

def _ensure_dir(path: str):
    if path and not os.path.exists(path):
        os.makedirs(path, exist_ok=True)

def _depth_to_vis(d16: np.ndarray) -> np.ndarray:
    """uint16(mm) 深度 → 8bit 伪彩，仅用于显示"""
    if d16 is None or d16.size == 0:
        return np.zeros((1, 1, 3), np.uint8)
    vmax = max(1.0, float(np.quantile(d16.astype(np.float32), 0.98)))
    vis = np.clip(d16.astype(np.float32) / vmax * 255.0, 0, 255).astype(np.uint8)
    return cv2.applyColorMap(vis, cv2.COLORMAP_JET)

def _vf_size(vf) -> tuple[int, int]:
    """VideoFrame 尺寸（兼容不同绑定：get_height/height）"""
    gh, gw = getattr(vf, "get_height", None), getattr(vf, "get_width", None)
    if callable(gh) and callable(gw):
        return int(gh()), int(gw())
    return int(vf.height()), int(vf.width())

def _fbuf(f) -> memoryview | bytes:
    """帧缓冲（兼容 get_buffer/data/get_data）"""
    if hasattr(f, "get_buffer"):
        return f.get_buffer()
    if hasattr(f, "data"):
        return f.data()
    return f.get_data()

# ========================= 单例采集器 =========================

class _RGBDCapture:
    """
    内部单例：自动初始化 / 获取帧 / 关闭。
    你无需直接使用本类，对外只提供函数式 API。
    """
    def __init__(self,
                 color=(1920, 1080, 30),
                 depth=(640, 480, 30),
                 color_format=OBFormat.BGR,
                 depth_format=OBFormat.Y16):
        self.color = color
        self.depth = depth
        self.color_format = color_format
        self.depth_format = depth_format
        self.pipe = None
        self.scale_cache = None  # 深度 scale（毫米换算）

    # ----------- 生命周期管理 -----------
    def _start_if_needed(self):
        if self.pipe is not None:
            return
        # 构建 pipeline + config（严格使用 profile）
        pipe = Pipeline()
        cfg = Config()

        # 彩色 profile
        c_list = pipe.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        w, h, fps = self.color
        try:
            c_prof = c_list.get_video_stream_profile(w, h, self.color_format, fps)
        except Exception:
            c_prof = c_list.get_default_video_stream_profile()
        cfg.enable_stream(c_prof)

        # 深度 profile
        d_list = pipe.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        wd, hd, fpsd = self.depth
        try:
            d_prof = d_list.get_video_stream_profile(wd, hd, self.depth_format, fpsd)
        except Exception:
            d_prof = d_list.get_default_video_stream_profile()
        cfg.enable_stream(d_prof)

        pipe.start(cfg)
        self.pipe = pipe
        atexit.register(self.close)  # 进程结束时兜底清理

    def close(self):
        if self.pipe is not None:
            try:
                self.pipe.stop()
            except Exception:
                pass
            self.pipe = None

    # ----------- 取帧（单次）-----------
    def get_frame_once(self, timeout_ms=1000):
        """
        返回 (color_img[BGR], depth_raw[uint16], depth_scale[float])
        任一不可用时返回 None
        """
        self._start_if_needed()
        fs = self.pipe.wait_for_frames(timeout_ms)
        if fs is None:
            return None, None, None

        # 彩色
        color_img = None
        cf = fs.get_color_frame()
        if cf is not None:
            vf = cf.as_video_frame()
            h, w = _vf_size(vf)
            color_img = np.frombuffer(_fbuf(vf), np.uint8).reshape(h, w, 3)

        # 深度
        depth_raw, scale = None, None
        df = fs.get_depth_frame()
        if df is not None:
            vf = df.as_video_frame()
            hd, wd = _vf_size(vf)
            depth_raw = np.frombuffer(_fbuf(vf), np.uint16).reshape(hd, wd)
            # scale：不同绑定命名一致
            try:
                scale = df.get_depth_scale()
            except Exception:
                scale = None

        return color_img, depth_raw, scale

# 模块级“单例”
_CAPTURE = _RGBDCapture()

# ========================= 对外：显示函数 =========================

def show_rgb_live(window="Orbbec RGB", exit_keys=("q", 27)):
    """
    一键显示彩色视频（内部自动初始化）。
    退出：按 'q' 或 ESC（27）
    """
    try:
        while True:
            color, _, _ = _CAPTURE.get_frame_once()
            if color is None:
                continue
            cv2.imshow(window, color)
            k = cv2.waitKey(1) & 0xFF
            if k == 27 or k == ord('q'):
                break
    finally:
        cv2.destroyWindow(window)

def show_depth_live(window="Orbbec Depth", exit_keys=("q", 27)):
    """
    一键显示深度伪彩视频（内部自动初始化）。
    退出：按 'q' 或 ESC
    """
    try:
        while True:
            _, depth, _ = _CAPTURE.get_frame_once()
            if depth is None:
                continue
            vis = _depth_to_vis(depth)
            cv2.imshow(window, vis)
            k = cv2.waitKey(1) & 0xFF
            if k == 27 or k == ord('q'):
                break
    finally:
        cv2.destroyWindow(window)

def show_rgbd_live(window="Orbbec RGBD", exit_keys=("q", 27)):
    """
    一键并排显示（左：RGB，右：Depth 伪彩）
    """
    try:
        while True:
            color, depth, _ = _CAPTURE.get_frame_once()
            if color is None and depth is None:
                continue
            if color is not None and depth is not None:
                vis = _depth_to_vis(depth)
                if vis.shape[:2] != color.shape[:2]:
                    vis = cv2.resize(vis, (color.shape[1], color.shape[0]))
                show = np.hstack([color, vis])
            elif color is not None:
                show = color
            else:
                show = _depth_to_vis(depth)
            cv2.imshow(window, show)
            k = cv2.waitKey(1) & 0xFF
            if k != -1:  # 按下任意键
                break
    finally:
        cv2.destroyWindow(window)

# ========================= 对外：保存函数 =========================

def save_rgb_frame(out_dir: str, filename: str):
    """
    保存一帧彩色 PNG
    :param out_dir: 保存目录（必须存在/会自动创建）
    :param filename: 文件名（例如 "rgb.png"）
    :return: 完整保存路径
    """
    color, _, _ = _CAPTURE.get_frame_once()
    if color is None:
        raise RuntimeError("未获取到彩色帧")
    _ensure_dir(out_dir)
    path = os.path.join(out_dir, filename)
    cv2.imwrite(path, color)
    print(f"[保存RGB] {path}")
    return path


def save_depth_frame(out_dir: str, filename_raw: str,
                     filename_vis: str = None, filename_mm: str = None):
    """
    保存一帧深度图
    :param out_dir: 保存目录
    :param filename_raw: 原始深度图 (uint16 PNG)
    :param filename_vis: 伪彩 PNG（可选，传 None 则不保存）
    :param filename_mm: 毫米值矩阵 npy（可选，传 None 则不保存）
    :return: 保存路径 dict
    """
    _, depth, scale = _CAPTURE.get_frame_once()
    if depth is None:
        raise RuntimeError("未获取到深度帧")
    _ensure_dir(out_dir)

    paths = {}
    raw_p = os.path.join(out_dir, filename_raw)
    cv2.imwrite(raw_p, depth)
    paths["raw_png"] = raw_p

    if filename_vis:
        vis_p = os.path.join(out_dir, filename_vis)
        cv2.imwrite(vis_p, _depth_to_vis(depth))
        paths["vis_png"] = vis_p

    if filename_mm and (scale is not None):
        mm = depth.astype(np.float32) * float(scale)
        mm_p = os.path.join(out_dir, filename_mm)
        np.save(mm_p, mm)
        paths["mm_npy"] = mm_p

    print(f"[保存Depth] {paths}")
    return paths


def save_rgbd_frame(out_dir: str,
                    filename_color: str,
                    filename_depth_raw: str,
                    filename_depth_vis: str = None,
                    filename_depth_mm: str = None):
    """
    同时保存一帧 RGB 和 Depth
    :param out_dir: 保存目录
    :param filename_color: 彩色 PNG 文件名
    :param filename_depth_raw: 深度 PNG 文件名 (uint16)
    :param filename_depth_vis: 深度伪彩 PNG 文件名（可选）
    :param filename_depth_mm: 毫米值矩阵 npy 文件名（可选）
    :return: 保存路径 dict
    """
    color, depth, scale = _CAPTURE.get_frame_once()
    if color is None and depth is None:
        raise RuntimeError("未获取到 RGBD 帧")
    _ensure_dir(out_dir)

    paths = {}
    if color is not None:
        cp = os.path.join(out_dir, filename_color)
        cv2.imwrite(cp, color)
        paths["color_png"] = cp

    if depth is not None:
        rp = os.path.join(out_dir, filename_depth_raw)
        cv2.imwrite(rp, depth)
        paths["depth_raw_png"] = rp

        if filename_depth_vis:
            vp = os.path.join(out_dir, filename_depth_vis)
            cv2.imwrite(vp, _depth_to_vis(depth))
            paths["depth_vis_png"] = vp

        if filename_depth_mm and (scale is not None):
            mm = depth.astype(np.float32) * float(scale)
            mp = os.path.join(out_dir, filename_depth_mm)
            np.save(mp, mm)
            paths["depth_mm_npy"] = mp

    print(f"[保存RGBD] {paths}")
    return paths
def capture_pointcloud(out_dir: str, filename: str = "point_cloud.ply") -> str:
    if not os.path.exists(out_dir):
        os.makedirs(out_dir, exist_ok=True)

    pipeline = Pipeline()
    config = Config()

    # 配置深度流
    depth_profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
    depth_profile = depth_profile_list.get_default_video_stream_profile()
    print(f"[INFO] 默认深度流: {depth_profile.get_width()}x{depth_profile.get_height()} "
          f"{depth_profile.get_format()} {depth_profile.get_fps()}fps")
    config.enable_stream(depth_profile)

    has_color_sensor = False
    try:
        profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        if profile_list is not None:
            color_profile = profile_list.get_default_video_stream_profile()
            print(f"[INFO] 默认彩色流: {color_profile.get_width()}x{color_profile.get_height()} "
                  f"{color_profile.get_format()} {color_profile.get_fps()}fps")
            config.enable_stream(color_profile)
            has_color_sensor = True
    except Exception as e:
        print("[WARN] 彩色流配置失败:", e)

    pipeline.enable_frame_sync()
    pipeline.start(config)

    align_filter = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
    point_cloud_filter = PointCloudFilter()

    ply_path = os.path.join(out_dir, filename)

    try:
        while True:
            frames = pipeline.wait_for_frames(100)
            if frames is None:
                continue

            depth_frame = frames.get_depth_frame()
            if depth_frame is None:
                continue

            color_frame = frames.get_color_frame()
            if has_color_sensor and color_frame is None:
                continue

            frame = align_filter.process(frames)

            point_format = (
                OBFormat.RGB_POINT if has_color_sensor and color_frame is not None else OBFormat.POINT
            )
            point_cloud_filter.set_create_point_format(point_format)

            point_cloud_frame = point_cloud_filter.process(frame)
            if point_cloud_frame is None:
                continue

            save_point_cloud_to_ply(ply_path, point_cloud_frame)
            try:
                count = point_cloud_frame.get_point_count()
                print(f"[保存点云] {ply_path}, 点数={count}")
            except Exception:
                print(f"[保存点云] {ply_path}")
            break
    finally:
        pipeline.stop()
        print("[INFO] 停止 pipeline")

    return ply_path

# ========================= 相机参数（内外参） =========================

def _ensure_started():
    """确保 pipeline 已启动（不会重复启动）。"""
    _CAPTURE._start_if_needed()

def _intr_to_dict(it):
    """OBCameraIntrinsic -> dict（不包含畸变系数）"""
    return {
        "width": getattr(it, "width", None),
        "height": getattr(it, "height", None),
        "fx": getattr(it, "fx", None),
        "fy": getattr(it, "fy", None),
        "cx": getattr(it, "cx", None),
        "cy": getattr(it, "cy", None),
    }

def _dist_to_dict(dist):
    """
    OBCameraDistortion -> dict
    各版本字段名可能不同：有的提供 dist.coeffs，有的提供 k1,k2,p1,p2,k3,k4,k5,k6 或 model。
    都尽量兼容；没有就返回空数组。
    """
    # 先试试看是否有 dist.coeffs
    if hasattr(dist, "coeffs"):
        try:
            coeffs = list(dist.coeffs)
        except Exception:
            coeffs = []
    else:
        # 逐个收集常见名字
        names = ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6", "s1", "s2", "s3", "s4"]
        coeffs = []
        for n in names:
            if hasattr(dist, n):
                coeffs.append(getattr(dist, n))
        # 如果一个都没有，给空数组
    model = getattr(dist, "model", None) if hasattr(dist, "model") else None
    return {"model": model, "coeffs": coeffs}

def get_camera_params() -> dict:
    """
    获取彩色/深度的内参、畸变，以及深度->彩色的外参。
    """
    _ensure_started()
    pipe = _CAPTURE.pipe

    # 与当前启流一致：直接用默认 profile（如需严格一致，可改成你实际启用的分辨率/格式）
    c_list = pipe.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
    d_list = pipe.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
    color_prof = c_list.get_default_video_stream_profile()
    depth_prof = d_list.get_default_video_stream_profile()

    # 内参/畸变
    c_intr = color_prof.get_intrinsic()
    d_intr = depth_prof.get_intrinsic()
    c_dist = color_prof.get_distortion()
    d_dist = depth_prof.get_distortion()

    # 外参：深度 -> 彩色
    d2c = depth_prof.get_extrinsic_to(color_prof)

    params = {
        "color_intrinsic": _intr_to_dict(c_intr),
        "color_distortion": _dist_to_dict(c_dist),
        "depth_intrinsic": _intr_to_dict(d_intr),
        "depth_distortion": _dist_to_dict(d_dist),
        "depth_to_color_extrinsic": {
            "rotation": list(getattr(d2c, "rotation", [])),       # 3x3 按行展平
            "translation": list(getattr(d2c, "translation", [])), # 3 元素，单位 m
        },
    }
    return params

def save_pointcloud_frame(out_dir: str, filename_ply: str):
    """
    保存一帧点云数据（PLY ASCII 格式，带彩色）
    :param out_dir: 保存目录
    :param filename_ply: 点云文件名 (例如 "cloud.ply")
    :return: 完整保存路径
    """
    color, depth, scale = _CAPTURE.get_frame_once()
    if color is None or depth is None or scale is None:
        raise RuntimeError("未获取到完整的 RGBD 帧，无法保存点云")

    _ensure_dir(out_dir)

    # 获取相机内参
    pipe = _CAPTURE.pipe
    c_list = pipe.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
    color_prof = c_list.get_default_video_stream_profile()
    intr = color_prof.get_intrinsic()
    fx, fy = intr.fx, intr.fy
    cx, cy = intr.cx, intr.cy

    # 深度转毫米
    depth_mm = depth.astype(np.float32) * float(scale)

    # 生成点云
    points = []
    h, w = depth_mm.shape
    for v in range(h):
        for u in range(w):
            z = depth_mm[v, u]
            if z <= 0:  # 无效点
                continue
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            b, g, r = color[v, u]  # BGR 转换
            points.append(f"{x} {y} {z} {r} {g} {b}\n")

    ply_path = os.path.join(out_dir, filename_ply)
    with open(ply_path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        f.writelines(points)

    print(f"[保存PointCloud] {ply_path}, 点数={len(points)}")
    return ply_path

def save_camera_params(json_path: str) -> str:
    """
    一键保存相机参数为 JSON（若目录不存在会自动创建）。
    """
    
    _ensure_dir(os.path.dirname(json_path) or ".")
    params = get_camera_params()
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(params, f, ensure_ascii=False, indent=2)
    print(f"[保存相机参数] {json_path}")
    return json_path


# ========================= 可选：手动关闭 =========================

def close_device():
    """手动释放设备（通常不需要，程序退出会自动清理）"""
    _CAPTURE.close()
    # 也清理所有 OpenCV 窗口，防 /dev/videoX 残占
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass

# if __name__ == "__main__":
#     save_rgbd_frame("output", "color.png", "depth_raw.png", "depth_vis.png", "depth_mm.npy")
#     # save_pointcloud_frame("output", "cloud.ply")
# if __name__ == "__main__":
#     capture_pointcloud("output", "cloud.ply")

if __name__ == "__main__":
    # 直接运行本文件：实时显示 RGB + Depth 伪彩并排画面
    show_rgbd_live("Orbbec RGBD Live")
    # 退出时顺便关掉设备
    close_device()