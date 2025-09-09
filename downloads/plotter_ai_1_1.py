import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 最小化連桿長度配置 (cm)，旨在涵蓋 20x20cm 區域
L0 = 26       # 馬達間距
L1 = 18       # 主動臂 M1->P1
L2 = 18       # 主動臂 M2->P2
L3 = 16       # 被動臂 P1->筆尖
L4 = 16       # 被動臂 P2->筆尖

# 馬達固定點座標
M1 = np.array([0, 0])
M2 = np.array([L0, 0])

# 繪圖區域的四個角點
# 為了讓筆尖能達到，將區域起點設為 (3,3)
corner_points = [
    np.array([3, 3]),
    np.array([23, 3]),
    np.array([23, 23]),
    np.array([3, 23]),
    np.array([3, 3])  # 迴到起點
]

def inverse_kinematics(x, y):
    """
    給定筆尖座標 (x, y)，求解馬達角度 theta1, theta2
    """
    try:
        # 計算 P1 和 P2 點
        # 這是逆運動學的核心，求解 P1/P2 圓與筆尖圓的交點
        r_M1_pen = np.sqrt(x**2 + y**2)
        r_M2_pen = np.sqrt((x - L0)**2 + y**2)
        
        # 根據三角形餘弦定理求解 P1 點
        alpha1 = np.arccos((L1**2 + r_M1_pen**2 - L3**2) / (2 * L1 * r_M1_pen))
        theta_pen_M1 = np.arctan2(y, x)
        theta1 = theta_pen_M1 + alpha1  # 選擇一個解 (通常是上方的)

        # 根據三角形餘弦定理求解 P2 點
        beta2 = np.arccos((L2**2 + r_M2_pen**2 - L4**2) / (2 * L2 * r_M2_pen))
        theta_pen_M2 = np.arctan2(y, x - L0)
        theta2 = theta_pen_M2 - beta2 # 選擇一個解 (通常是上方的)

        return np.degrees(theta1), np.degrees(theta2)
    except Exception:
        return None, None

def forward_kinematics(theta1, theta2):
    """
    給定馬達角度，計算筆尖座標
    """
    P1 = M1 + L1 * np.array([np.cos(np.radians(theta1)), np.sin(np.radians(theta1))])
    P2 = M2 + L2 * np.array([np.cos(np.radians(theta2)), np.sin(np.radians(theta2))])
    
    d = np.linalg.norm(P2 - P1)
    if d > L3 + L4 or d < abs(L3 - L4) or d == 0:
        return None

    a = (L3**2 - L4**2 + d**2) / (2*d)
    h = np.sqrt(L3**2 - a**2)
    p = P1 + a * (P2 - P1) / d
    offset = h * np.array([-(P2[1] - P1[1]) / d, (P2[0] - P1[0]) / d])

    return p + offset if p[1] + offset[1] > p[1] - offset[1] else p - offset

# 生成要繪製的點
path_points = []
num_points = 50  # 每條邊長直線的取樣點數

for i in range(len(corner_points) - 1):
    start = corner_points[i]
    end = corner_points[i+1]
    
    # 在兩個角點間取樣，生成直線上的點
    x_coords = np.linspace(start[0], end[0], num_points)
    y_coords = np.linspace(start[1], end[1], num_points)
    
    for j in range(num_points):
        path_points.append(np.array([x_coords[j], y_coords[j]]))

# 根據每個路徑點，計算所需的馬達角度
motor_angles = []
for p in path_points:
    angle1, angle2 = inverse_kinematics(p[0], p[1])
    if angle1 is not None:
        motor_angles.append((angle1, angle2))

fig, ax = plt.subplots(figsize=(10, 10))

ax.set_xlim(-5, L0 + 5)
ax.set_ylim(-5, 30)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title('5-bar Linkage Plotter — 繪製 20x20cm 正方形')

# 20cm x 20cm 繪圖區域 (起點從(3,3)開始)
draw_area = plt.Rectangle((3, 3), 20, 20, linewidth=2, edgecolor='red', facecolor='none', linestyle='--', label='20cm x 20cm 繪圖區域')
ax.add_patch(draw_area)

# 馬達點標示
ax.plot([M1[0], M2[0]], [M1[1], M2[1]], 'ko')
ax.text(M1[0], M1[1]-2, 'M1', ha='center')
ax.text(M2[0], M2[1]-2, 'M2', ha='center')

# 繪製連桿及筆尖點
lines, = ax.plot([], [], 'o-', lw=3, color='blue')
pen_point, = ax.plot([], [], 'ro', markersize=8, label='筆尖')
pen_path, = ax.plot([], [], 'r-', lw=1.5, alpha=0.8, label='筆尖軌跡')
drawn_path_x = []
drawn_path_y = []

def update(frame):
    if frame >= len(motor_angles):
        return lines, pen_point, pen_path

    theta1, theta2 = motor_angles[frame]
    
    # 利用正向運動學計算連桿位置
    P1 = M1 + L1 * np.array([np.cos(np.radians(theta1)), np.sin(np.radians(theta1))])
    P2 = M2 + L2 * np.array([np.cos(np.radians(theta2)), np.sin(np.radians(theta2))])
    
    # 這裡直接使用目標點作為筆尖位置，因為角度是根據它逆推的
    pen = path_points[frame]

    # 更新連桿路徑：M1->P1->筆尖->P2->M2
    x_vals = [M1[0], P1[0], pen[0], P2[0], M2[0]]
    y_vals = [M1[1], P1[1], pen[1], P2[1], M2[1]]

    lines.set_data(x_vals, y_vals)
    pen_point.set_data([pen[0]], [pen[1]])
    
    # 紀錄筆尖實際繪圖路徑
    drawn_path_x.append(pen[0])
    drawn_path_y.append(pen[1])
    pen_path.set_data(drawn_path_x, drawn_path_y)

    return lines, pen_point, pen_path

ani = FuncAnimation(fig, update, frames=len(motor_angles), interval=50, blit=True)

plt.legend()
plt.show()