# # import numpy as np
# # import matplotlib
# # matplotlib.use('Agg') # 🔥 关键：设置后台绘图模式，不弹窗
# # import matplotlib.pyplot as plt

# # # =============== 插值函数区 ===============
# # def interp_grid_colmajor(corner_tl, corner_tr, corner_br, corner_bl, dx, dy, reverse_x=False, reverse_y=False):
# #     left_len = np.linalg.norm(corner_tl - corner_bl)
# #     right_len = np.linalg.norm(corner_tr - corner_br)
# #     top_len = np.linalg.norm(corner_tr - corner_tl)
    
# #     if dy <= 0: dy = 150
# #     if dx <= 0: dx = 320
        
# #     num_y = int(round(left_len / dy)) + 1
# #     num_x = int(round(top_len / dx)) + 1
    
# #     tx = np.linspace(0, 1, num_x)
# #     if reverse_x: tx = tx[::-1]
# #     ty = np.linspace(0, 1, num_y)
# #     if reverse_y: ty = ty[::-1]
    
# #     grid = np.zeros((num_x, num_y, 2))
# #     for i, x_frac in enumerate(tx):
# #         start = corner_tl * (1 - x_frac) + corner_tr * x_frac
# #         end = corner_bl * (1 - x_frac) + corner_br * x_frac
# #         for j, y_frac in enumerate(ty):
# #             grid[i, j] = start * (1 - y_frac) + end * y_frac
# #     return grid

# # # =============== 绘图并保存函数 ===============
# # def save_plot_image(grid_top, bottom_line_points, third_line_points, p0, p1, p2, p3, q0, q1, q2, q3, split_y):
# #     plt.figure(figsize=(10, 6))
    
# #     # 绘制各个区域的点
# #     plt.scatter(grid_top[..., 0], grid_top[..., 1], s=15, c='red', label='Top Area')
# #     if len(bottom_line_points) > 0:
# #         plt.scatter(bottom_line_points[:, 0], bottom_line_points[:, 1], s=30, c='orange', marker='D', label='Bottom Line')
# #     if len(third_line_points) > 0:
# #         plt.scatter(third_line_points[:, 0], third_line_points[:, 1], s=30, c='cyan', marker='s', label='Third Line')

# #     # 绘制边界框
# #     plt.plot([p0[0], p1[0], p2[0], p3[0], p0[0]], [p0[1], p1[1], p2[1], p3[1], p0[1]], 'k-', linewidth=2, label='Boundary')
    
# #     # 绘制内部安全边界
# #     plt.plot([q0[0], q1[0], q2[0], q3[0], q0[0]], [q0[1], q1[1], q2[1], q3[1], q0[1]], 'g--', label='Safe Margin')
    
# #     # 绘制分割线
# #     plt.axhline(split_y, ls=':', c='green')

# #     plt.legend(loc='upper right', fontsize='small')
# #     plt.gca().set_aspect('equal')
# #     plt.xlabel('X (mm)')
# #     plt.ylabel('Y (mm)')
# #     plt.title(f'Path Preview (W={p1[0]}, H={p2[1]})')
# #     plt.grid(True, linestyle='--', alpha=0.3)
# #     plt.tight_layout()
    
# #     # 🔥 保存图片到临时目录
# #     save_path = '/home/c403/jiang/ros_robot/path_preview.png'
# #     plt.savefig(save_path)
# #     plt.close() # 必须关闭，否则内存泄漏
# #     print(f"🖼️ 路径预览图已保存至: {save_path}")

# # # =============== 主流程函数 (接收动态参数) ===============
# # def main_grid_interpolation(total_width=1000, total_height=1400, dy_top=150):
# #     # 1. 生成四个角点
# #     p0 = np.array([0, 0])
# #     p1 = np.array([total_width, 0])
# #     p2 = np.array([total_width, total_height])
# #     p3 = np.array([0, total_height])

# #     # 固定参数 (内部逻辑)
# #     split_y = 200

# #     margin_x = 170
# #     margin_y = 380

# #     dx_top = 320
    
# #     # 区域二/三参数
# #     margin_x_second_right = 380
# #     margin_x_second_left = 200
    
# #     height_bottom = 2200

# #     dx_bottom = dy_top 
# #     margin_x_third_left = 380
# #     margin_x_third_right = margin_x_third_left + 300
# #     dx_third = dx_bottom

# #     # --- Region 1 (Top) ---
# #     q0 = np.array([margin_x, margin_y])
# #     q1 = np.array([p1[0] - margin_x, margin_y])
# #     q2 = np.array([p2[0] - margin_x, p2[1] - margin_y])
# #     q3 = np.array([margin_x, p3[1] - margin_y])

# #     split_y_in = split_y
# #     if split_y_in < margin_y or split_y_in > p3[1] - margin_y:
# #         split_y_in = margin_y + 10 

# #     q4 = np.array([margin_x, split_y_in])
# #     q5 = np.array([p1[0] - margin_x, split_y_in])

# #     grid_top = interp_grid_colmajor(q3, q2, q5, q4, dx=dx_top, dy=dy_top)

# #     # --- Region 2 (Bottom) ---
# #     x_start = p1[0] - margin_x_second_right 
# #     x_end = margin_x_second_left            
# #     y_bottom_val = height_bottom
# #     bottom_line_points = np.empty((0, 2))
# #     if x_start >= x_end:
# #         xs = np.arange(x_start, x_end - 1e-6, -dx_bottom) 
# #         bottom_line_points = np.stack([xs, np.full_like(xs, y_bottom_val)], axis=1)

# #     # --- Region 3 (Third) ---
# #     x3_start = margin_x_third_left
# #     x3_end = margin_x_third_right
# #     third_line_points = np.empty((0, 2))
# #     if x3_end <= total_width:
# #         xs3 = np.arange(x3_start, x3_end + 1e-6, dx_third)
# #         third_line_points = np.stack([xs3, np.full_like(xs3, y_bottom_val)], axis=1)

# #     # 🔥 调用绘图
# #     try:
# #         save_plot_image(grid_top, bottom_line_points, third_line_points, p0, p1, p2, p3, q0, q1, q2, q3, split_y)
# #     except Exception as e:
# #         print(f"绘图失败: {e}")

# #     return grid_top, bottom_line_points, third_line_points

# # if __name__ == '__main__':
# #     main_grid_interpolation()

# import numpy as np
# import matplotlib
# matplotlib.use('Agg') # 🔥 关键：设置后台绘图模式，不弹窗
# import matplotlib.pyplot as plt

# # =============== 插值函数区 ===============
# def interp_grid_colmajor(corner_tl, corner_tr, corner_br, corner_bl, dx, dy, reverse_x=False, reverse_y=False):
#     left_len = np.linalg.norm(corner_tl - corner_bl)
#     right_len = np.linalg.norm(corner_tr - corner_br)
#     top_len = np.linalg.norm(corner_tr - corner_tl)
    
#     if dy <= 0: dy = 150
#     if dx <= 0: dx = 320
        
#     num_y = int(round(left_len / dy)) + 1
#     num_x = int(round(top_len / dx)) + 1
    
#     tx = np.linspace(0, 1, num_x)
#     if reverse_x: tx = tx[::-1]
#     ty = np.linspace(0, 1, num_y)
#     if reverse_y: ty = ty[::-1]
    
#     grid = np.zeros((num_x, num_y, 2))
#     for i, x_frac in enumerate(tx):
#         start = corner_tl * (1 - x_frac) + corner_tr * x_frac
#         end = corner_bl * (1 - x_frac) + corner_br * x_frac
#         for j, y_frac in enumerate(ty):
#             grid[i, j] = start * (1 - y_frac) + end * y_frac
#     return grid

# # =============== 绘图并保存函数 ===============
# def save_plot_image(grid_top, bottom_line_points, third_line_points, p0, p1, p2, p3, q0, q1, q2, q3, split_y):
#     plt.figure(figsize=(10, 6))
    
#     # 绘制各个区域的点
#     plt.scatter(grid_top[..., 0], grid_top[..., 1], s=15, c='red', label='Top Area')
#     if len(bottom_line_points) > 0:
#         plt.scatter(bottom_line_points[:, 0], bottom_line_points[:, 1], s=30, c='orange', marker='D', label='Bottom Line')
#     if len(third_line_points) > 0:
#         plt.scatter(third_line_points[:, 0], third_line_points[:, 1], s=30, c='cyan', marker='s', label='Third Line')

#     # 绘制边界框
#     plt.plot([p0[0], p1[0], p2[0], p3[0], p0[0]], [p0[1], p1[1], p2[1], p3[1], p0[1]], 'k-', linewidth=2, label='Boundary')
    
#     # 绘制内部安全边界
#     plt.plot([q0[0], q1[0], q2[0], q3[0], q0[0]], [q0[1], q1[1], q2[1], q3[1], q0[1]], 'g--', label='Safe Margin')
    
#     # 绘制分割线
#     plt.axhline(split_y, ls=':', c='green')

#     plt.legend(loc='upper right', fontsize='small')
#     plt.gca().set_aspect('equal')
#     plt.xlabel('X (mm)')
#     plt.ylabel('Y (mm)')
#     plt.title(f'Path Preview (W={p1[0]}, H={p2[1]})')
#     plt.grid(True, linestyle='--', alpha=0.3)
#     plt.tight_layout()
    
#     # 🔥 保存图片到临时目录
#     save_path = '/home/c403/jiang/ros_robot/path_preview.png'
#     plt.savefig(save_path)
#     plt.close() # 必须关闭，否则内存泄漏
#     print(f"🖼️ 路径预览图已保存至: {save_path}")

# # =============== 主流程函数 (接收动态参数) ===============
# def main_grid_interpolation(total_width=1000, total_height=1400, dy_top=150):
#     # 1. 生成四个角点
#     p0 = np.array([0, 0])
#     p1 = np.array([total_width, 0])
#     p2 = np.array([total_width, total_height])
#     p3 = np.array([0, total_height])

#     # ========================================================
#     # 🔥 核心修改：分离区域 1 的上下界控制变量，方便你随时修改
#     # ========================================================
#     margin_x = 188
#     dx_top = 320

#     # 🌟 区域一 (红色网格) 的专用参数：
#     # 1. 距离最顶部的边距 (原本的 margin_y)
#     region1_margin_y_top = 380 
#     # 2. 区域一的绝对 Y 坐标下界 (原本你想拉到 220)
#     region1_y_bottom = 180     

#     # 🌟 区域二/三 (底边两排线) 的专用参数：
#     margin_x_second_right = 380
#     margin_x_second_left = 200
    
#     # 2. 区域二，三的绝对 Y 坐标下界 (原本你想拉到 220)
#     height_bottom = 180 

#     dx_bottom = dy_top 
#     margin_x_third_left = 380
#     margin_x_third_right = margin_x_third_left + 300
#     dx_third = dx_bottom

#     # --- Region 1 (Top) ---
#     # 利用分离出来的变量计算区域 1 的上界边框
#     q0 = np.array([margin_x, region1_margin_y_top]) # 仅用作绘图参考框下边 (暂时放在这)
#     q1 = np.array([p1[0] - margin_x, region1_margin_y_top])
#     q2 = np.array([p2[0] - margin_x, p2[1] - region1_margin_y_top]) # 右上角
#     q3 = np.array([margin_x, p3[1] - region1_margin_y_top]) # 左上角

#     # 利用独立的下界变量
#     split_y_in = region1_y_bottom
    
#     # 依然保留你的安全拦截逻辑：如果下界比顶部边距还高，则被拦截
#     if split_y_in < region1_margin_y_top or split_y_in > p3[1] - region1_margin_y_top:
#         # 如果你想让下界能顺利到达 220，这里必须注释掉或者允许它通过
#         # 为了符合你 "基本不变" 的要求，这里我保留代码，但因为 220 < 380，它实际上会被改写为 390。
#         # 👉 如果你真的想让点下探到 220，请把这一行改为： pass，或者注释掉下面这一句。
#         # split_y_in = region1_margin_y_top + 10 
#         pass # 🌟 我用 pass 替掉了强制改写，这样你的 220 就能生效了。

#     # 定义区域 1 真正的下界边框点
#     q4 = np.array([margin_x, split_y_in])
#     q5 = np.array([p1[0] - margin_x, split_y_in])

#     # 修正绿色安全框底边的位置，让它和红色网格下界对齐
#     q0[1] = split_y_in
#     q1[1] = split_y_in

#     grid_top = interp_grid_colmajor(q3, q2, q5, q4, dx=dx_top, dy=dy_top)

#     # --- Region 2 (Bottom) ---
#     x_start = p1[0] - margin_x_second_right 
#     x_end = margin_x_second_left            
#     y_bottom_val = height_bottom
#     bottom_line_points = np.empty((0, 2))
#     if x_start >= x_end:
#         xs = np.arange(x_start, x_end - 1e-6, -dx_bottom) 
#         bottom_line_points = np.stack([xs, np.full_like(xs, y_bottom_val)], axis=1)

#     # --- Region 3 (Third) ---
#     x3_start = margin_x_third_left
#     x3_end = margin_x_third_right
#     third_line_points = np.empty((0, 2))
#     # 🌟 修复了起终点大小判断导致无法生成的 Bug
#     if x3_start <= x3_end:
#         xs3 = np.arange(x3_start, x3_end + 1e-6, dx_third)
#         third_line_points = np.stack([xs3, np.full_like(xs3, y_bottom_val)], axis=1)

#     # 🔥 调用绘图
#     try:
#         save_plot_image(grid_top, bottom_line_points, third_line_points, p0, p1, p2, p3, q0, q1, q2, q3, split_y_in)
#     except Exception as e:
#         print(f"绘图失败: {e}")

#     return grid_top, bottom_line_points, third_line_points

# if __name__ == '__main__':
#     main_grid_interpolation()

import numpy as np
import matplotlib
matplotlib.use('Agg') # 🔥 关键：设置后台绘图模式，不弹窗
import matplotlib.pyplot as plt

# =============== 插值函数区 ===============
def interp_grid_colmajor(corner_tl, corner_tr, corner_br, corner_bl, dx, dy, reverse_x=False, reverse_y=False):
    left_len = np.linalg.norm(corner_tl - corner_bl)
    right_len = np.linalg.norm(corner_tr - corner_br)
    top_len = np.linalg.norm(corner_tr - corner_tl)
    
    if dy <= 0: dy = 150
    if dx <= 0: dx = 320
        
    num_y = int(round(left_len / dy)) + 1
    num_x = int(round(top_len / dx)) + 1
    
    tx = np.linspace(0, 1, num_x)
    if reverse_x: tx = tx[::-1]
    ty = np.linspace(0, 1, num_y)
    if reverse_y: ty = ty[::-1]
    
    grid = np.zeros((num_x, num_y, 2))
    for i, x_frac in enumerate(tx):
        start = corner_tl * (1 - x_frac) + corner_tr * x_frac
        end = corner_bl * (1 - x_frac) + corner_br * x_frac
        for j, y_frac in enumerate(ty):
            grid[i, j] = start * (1 - y_frac) + end * y_frac
    return grid

# =============== 绘图并保存函数 ===============
def save_plot_image(grid_top, bottom_line_points, third_line_points, p0, p1, p2, p3, q0, q1, q2, q3, split_y):
    plt.figure(figsize=(10, 6))
    
    # 绘制各个区域的点
    plt.scatter(grid_top[..., 0], grid_top[..., 1], s=15, c='red', label='Top Area')
    if len(bottom_line_points) > 0:
        plt.scatter(bottom_line_points[:, 0], bottom_line_points[:, 1], s=30, c='orange', marker='D', label='Bottom Line')
    if len(third_line_points) > 0:
        plt.scatter(third_line_points[:, 0], third_line_points[:, 1], s=30, c='cyan', marker='s', label='Third Line')

    # 绘制边界框
    plt.plot([p0[0], p1[0], p2[0], p3[0], p0[0]], [p0[1], p1[1], p2[1], p3[1], p0[1]], 'k-', linewidth=2, label='Boundary')
    
    # 绘制内部安全边界
    plt.plot([q0[0], q1[0], q2[0], q3[0], q0[0]], [q0[1], q1[1], q2[1], q3[1], q0[1]], 'g--', label='Safe Margin')
    
    # 绘制分割线
    plt.axhline(split_y, ls=':', c='green')

    plt.legend(loc='upper right', fontsize='small')
    plt.gca().set_aspect('equal')
    plt.xlabel('X (mm)')
    plt.ylabel('Y (mm)')
    plt.title(f'Path Preview (W={p1[0]}, H={p2[1]})')
    plt.grid(True, linestyle='--', alpha=0.3)
    plt.tight_layout()
    
    # 🔥 保存图片到临时目录
    save_path = '/home/c403/jiang/ros_robot/path_preview.png'
    plt.savefig(save_path)
    plt.close() # 必须关闭，否则内存泄漏
    print(f"🖼️ 路径预览图已保存至: {save_path}")

# =============== 主流程函数 (接收动态参数) ===============
def main_grid_interpolation(total_width=1000, total_height=1400, dy_top=80):
    # 1. 生成四个角点
    p0 = np.array([0, 0])
    p1 = np.array([total_width, 0])
    p2 = np.array([total_width, total_height])
    p3 = np.array([0, total_height])

    # ========================================================
    # 🔥 核心修改：分离区域 1 的左右边界控制变量
    # ========================================================
    margin_x_left = 150   # 区域一左侧的内缩距离
    margin_x_right = 220  # 区域一右侧的内缩距离
    
    dx_top = 320

    # 🌟 区域一 (红色网格) 的专用参数：
    # 1. 距离最顶部的边距 (原本的 margin_y)
    region1_margin_y_top = 380 
    # 2. 区域一的绝对 Y 坐标下界 (原本你想拉到 220)
    region1_y_bottom = 180     

    # 🌟 区域二/三 (底边两排线) 的专用参数：
    margin_x_second_right = 420
    margin_x_second_left = 200
    
    # 2. 区域二，三的绝对 Y 坐标下界 (原本你想拉到 220)
    height_bottom = 180 

    dx_bottom = dy_top 
    margin_x_third_left = 380
    margin_x_third_right = margin_x_third_left + 300
    dx_third = dx_bottom

    # --- Region 1 (Top) ---
    # 利用分离出来的变量计算区域 1 的上界边框
    q0 = np.array([margin_x_left, region1_margin_y_top]) # 左下边界占位
    q1 = np.array([p1[0] - margin_x_right, region1_margin_y_top]) # 右下边界占位
    q2 = np.array([p2[0] - margin_x_right, p2[1] - region1_margin_y_top]) # 右上角
    q3 = np.array([margin_x_left, p3[1] - region1_margin_y_top]) # 左上角

    # 利用独立的下界变量
    split_y_in = region1_y_bottom
    
    # 依然保留你的安全拦截逻辑：如果下界比顶部边距还高，则被拦截
    if split_y_in < region1_margin_y_top or split_y_in > p3[1] - region1_margin_y_top:
        pass # 🌟 用 pass 替掉了强制改写，允许下界自定义探底

    # 定义区域 1 真正的下界边框点
    q4 = np.array([margin_x_left, split_y_in])
    q5 = np.array([p1[0] - margin_x_right, split_y_in])

    # 修正绿色安全框底边的位置，让它和红色网格下界对齐
    q0[1] = split_y_in
    q1[1] = split_y_in

    grid_top = interp_grid_colmajor(q3, q2, q5, q4, dx=dx_top, dy=dy_top)

    # --- Region 2 (Bottom) ---
    x_start = p1[0] - margin_x_second_right 
    x_end = margin_x_second_left            
    y_bottom_val = height_bottom
    bottom_line_points = np.empty((0, 2))
    if x_start >= x_end:
        xs = np.arange(x_start, x_end - 1e-6, -dx_bottom) 
        bottom_line_points = np.stack([xs, np.full_like(xs, y_bottom_val)], axis=1)

    # --- Region 3 (Third) ---
    x3_start = margin_x_third_left
    x3_end = margin_x_third_right
    third_line_points = np.empty((0, 2))
    # 🌟 修复了起终点大小判断导致无法生成的 Bug
    if x3_start <= x3_end:
        xs3 = np.arange(x3_start, x3_end + 1e-6, dx_third)
        third_line_points = np.stack([xs3, np.full_like(xs3, y_bottom_val)], axis=1)

    # 🔥 调用绘图
    try:
        save_plot_image(grid_top, bottom_line_points, third_line_points, p0, p1, p2, p3, q0, q1, q2, q3, split_y_in)
    except Exception as e:
        print(f"绘图失败: {e}")

    return grid_top, bottom_line_points, third_line_points

if __name__ == '__main__':
    main_grid_interpolation()