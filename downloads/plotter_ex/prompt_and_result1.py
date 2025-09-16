'''
提問:

本研究旨在探討一個用於 20cm x 20cm 平面繪圖的五連桿機構，其連桿長度參數分別為 l1​,l2​,l3​,l4​,l5​。請根據運動學最佳化原則，提出一個連桿長度的解決方案，使機構的整體佔用空間最小化，並在整個工作範圍內最大化其機械剛性。請詳細說明此配置的設計依據。
'''
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- 中文顯示設定 ---
plt.rcParams['font.family'] = 'Microsoft JhengHei' # 或 'Arial Unicode MS', 'SimHei'
plt.rcParams['font.size'] = 12
plt.rcParams['axes.unicode_minus'] = False # 解決負號'-'顯示為方塊的問題
# --------------------

# 1. 新的機構參數設定 (單位: mm) - 對稱菱形設計
l1 = 200  # 兩馬達間距
l2 = 150  # 右側第一桿
l3 = 150  # 右側第二桿
l4 = 150  # 左側第二桿
l5 = 150  # 左側第一桿

# 2. 定義繪圖區域與目標軌跡
drawing_center = np.array([100, 100])
radius = 100  # mm
num_points = 200

# 以下程式碼保持不變...
# 馬達位置
O1 = np.array([0, 0])
O2 = np.array([l1, 0])

# 建立圓形軌跡
theta_path = np.linspace(0, 2 * np.pi, num_points)
pen_points = drawing_center + radius * np.vstack((np.cos(theta_path), np.sin(theta_path))).T

# 3. 運動學反解函數 (Inverse Kinematics)
def get_joint_coords(target_point):
    """
    根據繪圖筆目標位置 (target_point)，反解出左右兩側連桿的中間關節點座標。
    """
    
    # 計算左側連桿 (O1, l5, l4)
    vec_O1_P = target_point - O1
    d1 = np.linalg.norm(vec_O1_P)
    
    if d1 > (l5 + l4) or d1 < abs(l5 - l4):
        return None
        
    cos_alpha1 = (l5**2 + d1**2 - l4**2) / (2 * l5 * d1)
    alpha1 = np.arccos(np.clip(cos_alpha1, -1.0, 1.0))
    base_angle1 = np.arctan2(vec_O1_P[1], vec_O1_P[0])
    motor_angle1 = base_angle1 + alpha1
    
    J_left = O1 + np.array([l5 * np.cos(motor_angle1), l5 * np.sin(motor_angle1)])

    # 計算右側連桿 (O2, l2, l3)
    vec_O2_P = target_point - O2
    d2 = np.linalg.norm(vec_O2_P)
    
    if d2 > (l2 + l3) or d2 < abs(l2 - l3):
        return None
    
    cos_alpha2 = (l2**2 + d2**2 - l3**2) / (2 * l2 * d2)
    alpha2 = np.arccos(np.clip(cos_alpha2, -1.0, 1.0))
    base_angle2 = np.arctan2(vec_O2_P[1], vec_O2_P[0])
    
    motor_angle2 = base_angle2 - alpha2
    
    J_right = O2 + np.array([l2 * np.cos(motor_angle2), l2 * np.sin(motor_angle2)])
    
    return J_left, J_right, target_point

# 4. 建立機構動作序列
trajectory_data = []
for pt in pen_points:
    result = get_joint_coords(pt)
    if result is not None:
        trajectory_data.append(result)

if not trajectory_data:
    print("錯誤: 繪圖範圍超出機構能力，無法生成軌跡。")
    exit()

num_frames = len(trajectory_data)

# 5. 動畫設定
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_aspect('equal')
ax.set_xlim(-50, l1 + 50)
ax.set_ylim(-50, 250)
ax.grid(True)
ax.set_title("5-Bar Linkage 機構動畫模擬 - 完整圓形")
ax.set_xlabel("X 座標 (mm)")
ax.set_ylabel("Y 座標 (mm)")

# 固定元素
ax.plot(O1[0], O1[1], 'ro', markersize=10, label='左馬達')
ax.plot(O2[0], O2[1], 'bo', markersize=10, label='右馬達')
ax.add_patch(plt.Rectangle((0, 0), 200, 200, fill=False, edgecolor='gray', linestyle=':', label='繪圖區域'))

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

ani = FuncAnimation(fig, update, frames=num_frames, interval=40, blit=True)

plt.show()