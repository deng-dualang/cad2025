import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- 中文顯示設定 ---
plt.rcParams['font.family'] = 'Microsoft JhengHei'  # 或 'Arial Unicode MS', 'SimHei'
plt.rcParams['font.size'] = 12
plt.rcParams['axes.unicode_minus'] = False  # 解決負號'-'顯示為方塊的問題

# --- 機構參數設定 (單位: mm) ---
base_length = 260  # 兩馬達間距
l2 = 165  # 右側第一桿 (增加長度以確保覆蓋整個正方形)
l3 = 165  # 右側第二桿
l4 = 165  # 左側第二桿
l5 = 165  # 左側第一桿
motor_radius = 30  # 馬達佔用半徑

# --- 定義繪圖區域與目標軌跡 (20cm x 20cm 正方形) ---
num_points_per_side = 50
total_points = num_points_per_side * 4

# 正方形邊界，避開馬達佔用區
min_x = motor_radius
min_y = motor_radius
max_x = base_length - motor_radius  # 230 mm
max_y = 200 + motor_radius  # 230 mm

pen_points = []
# 邊 1: 從 (min_x, min_y) 到 (max_x, min_y)
x_side1 = np.linspace(min_x, max_x, num_points_per_side)
y_side1 = np.full(num_points_per_side, min_y)
pen_points.extend(np.vstack((x_side1, y_side1)).T)
# 邊 2: 從 (max_x, min_y) 到 (max_x, max_y)
x_side2 = np.full(num_points_per_side, max_x)
y_side2 = np.linspace(min_y, max_y, num_points_per_side)
pen_points.extend(np.vstack((x_side2, y_side2)).T)
# 邊 3: 從 (max_x, max_y) 到 (min_x, max_y)
x_side3 = np.linspace(max_x, min_x, num_points_per_side)
y_side3 = np.full(num_points_per_side, max_y)
pen_points.extend(np.vstack((x_side3, y_side3)).T)
# 邊 4: 從 (min_x, max_y) 到 (min_x, min_y)
x_side4 = np.full(num_points_per_side, min_x)
y_side4 = np.linspace(max_y, min_y, num_points_per_side)
pen_points.extend(np.vstack((x_side4, y_side4)).T)
pen_points = np.array(pen_points)

# 馬達位置
O1 = np.array([0, 0])
O2 = np.array([base_length, 0])

# --- 運動學反解函數 (Inverse Kinematics) ---
def get_joint_coords(target_point):
    """
    根據繪圖筆目標位置 (target_point)，反解出左右兩側連桿的中間關節點座標。
    """
    # 計算左側連桿 (O1, l5, l4)
    vec_O1_P = target_point - O1
    d1 = np.linalg.norm(vec_O1_P)
    
    # 檢查是否可達
    if d1 > (l5 + l4 - 1e-6) or d1 < abs(l5 - l4) + 1e-6:
        return None
    
    cos_alpha1 = (l5**2 + d1**2 - l4**2) / (2 * l5 * d1)
    cos_alpha1 = np.clip(cos_alpha1, -1.0, 1.0)  # 避免數值誤差
    alpha1 = np.arccos(cos_alpha1)
    base_angle1 = np.arctan2(vec_O1_P[1], vec_O1_P[0])
    motor_angle1 = base_angle1 + alpha1  # 選擇正解 (另一解為 base_angle1 - alpha1)
    
    J_left = O1 + np.array([l5 * np.cos(motor_angle1), l5 * np.sin(motor_angle1)])
    
    # 計算右側連桿 (O2, l2, l3)
    vec_O2_P = target_point - O2
    d2 = np.linalg.norm(vec_O2_P)
    
    # 檢查是否可達
    if d2 > (l2 + l3 - 1e-6) or d2 < abs(l2 - l3) + 1e-6:
        return None
    
    cos_alpha2 = (l2**2 + d2**2 - l3**2) / (2 * l2 * d2)
    cos_alpha2 = np.clip(cos_alpha2, -1.0, 1.0)
    alpha2 = np.arccos(cos_alpha2)
    base_angle2 = np.arctan2(vec_O2_P[1], vec_O2_P[0])
    motor_angle2 = base_angle2 - alpha2  # 選擇負解以保持連桿配置
    
    J_right = O2 + np.array([l2 * np.cos(motor_angle2), l2 * np.sin(motor_angle2)])
    
    return J_left, J_right, target_point

# --- 建立機構動作序列 ---
trajectory_data = []
unreachable_points = []
for pt in pen_points:
    result = get_joint_coords(pt)
    if result is not None:
        trajectory_data.append(result)
    else:
        unreachable_points.append(pt)

if not trajectory_data:
    print("錯誤: 繪圖範圍完全超出機構能力，無法生成軌跡。")
    exit()
elif unreachable_points:
    print(f"警告: {len(unreachable_points)} 個點無法到達，軌跡可能不完整。")
    for pt in unreachable_points:
        print(f"無法到達的點: ({pt[0]:.2f}, {pt[1]:.2f})")

num_frames = len(trajectory_data)

# --- 動畫設定 ---
fig, ax = plt.subplots(figsize=(12, 10))
ax.set_aspect('equal')
ax.set_xlim(-50, base_length + 50)
ax.set_ylim(-50, max_y + 50)
ax.grid(True)
ax.set_title("5-Bar Linkage 機構動畫模擬 - 20cm x 20cm 正方形")
ax.set_xlabel("X 座標 (mm)")
ax.set_ylabel("Y 座標 (mm)")

# 固定元素
ax.plot(O1[0], O1[1], 'ro', markersize=10, label='左馬達')
ax.plot(O2[0], O2[1], 'bo', markersize=10, label='右馬達')
# 繪圖區域
ax.add_patch(plt.Rectangle((min_x, min_y), 200, 200, fill=False, edgecolor='green', linestyle=':', label='20cm x 20cm 繪圖區域'))
# 馬達佔用範圍
ax.add_patch(plt.Circle(O1, motor_radius, color='red', alpha=0.3, label='左馬達佔用範圍'))
ax.add_patch(plt.Circle(O2, motor_radius, color='blue', alpha=0.3, label='右馬達佔用範圍'))

# 動態圖元
link_l5, = ax.plot([], [], 'r-', lw=2, label='左側連桿')
link_l4, = ax.plot([], [], 'r-', lw=2)
link_l2, = ax.plot([], [], 'b-', lw=2, label='右側連桿')
link_l3, = ax.plot([], [], 'b-', lw=2)
pen_path, = ax.plot([], [], 'g-', lw=1, label='繪圖軌跡')
pen_point, = ax.plot([], [], 'ko', markersize=5, label='繪圖點')
ax.legend(loc='lower right')

# 動畫更新函數
def update(frame):
    J_left, J_right, pen = trajectory_data[frame]
    
    link_l5.set_data([O1[0], J_left[0]], [O1[1], J_left[1]])
    link_l4.set_data([J_left[0], pen[0]], [J_left[1], pen[1]])
    
    link_l2.set_data([O2[0], J_right[0]], [O2[1], J_right[1]])
    link_l3.set_data([J_right[0], pen[0]], [J_right[1], pen[1]])
    
    pen_point.set_data([pen[0]], [pen[1]])
    
    pen_trace_x = [d[2][0] for d in trajectory_data[:frame+1]]
    pen_trace_y = [d[2][1] for d in trajectory_data[:frame+1]]
    pen_path.set_data(pen_trace_x, pen_trace_y)
    
    return link_l5, link_l4, link_l2, link_l3, pen_path, pen_point

# 創建動畫
ani = FuncAnimation(fig, update, frames=num_frames, interval=40, blit=True)
plt.show()