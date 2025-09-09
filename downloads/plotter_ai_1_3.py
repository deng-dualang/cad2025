import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 初始連桿長度配置 (cm)
# 這個配置已知可以成功繪圖，作為搜尋的起點
L0_base = 26
L1_base = 18
L2_base = 18
L3_base = 16
L4_base = 16

# 繪圖區域的四個角點 (20cm x 20cm)
# 為了找到最小桿長，將繪圖區域置於更中心的位置
corner_points = [
    np.array([4, 4]),
    np.array([24, 4]),
    np.array([24, 24]),
    np.array([4, 24]),
    np.array([4, 4])
]

# 生成要繪製的點
path_points = []
num_points = 50  # 每條邊長直線的取樣點數

for i in range(len(corner_points) - 1):
    start = corner_points[i]
    end = corner_points[i+1]
    x_coords = np.linspace(start[0], end[0], num_points)
    y_coords = np.linspace(start[1], end[1], num_points)
    for j in range(num_points):
        path_points.append(np.array([x_coords[j], y_coords[j]]))

def inverse_kinematics_check(L0, L1, L2, L3, L4):
    """
    檢查給定的連桿長度配置是否能完成繪圖任務。
    如果所有路徑點的逆運動學都有解，則返回 True。
    """
    for p in path_points:
        x, y = p[0], p[1]
        try:
            M1 = np.array([0, 0])
            M2 = np.array([L0, 0])

            r_M1_pen = np.linalg.norm(np.array([x,y]) - M1)
            r_M2_pen = np.linalg.norm(np.array([x,y]) - M2)

            # 檢查是否可達
            if (r_M1_pen > L1 + L3) or (r_M2_pen > L2 + L4) or (r_M1_pen < abs(L1 - L3)) or (r_M2_pen < abs(L2 - L4)):
                return False

            # 這裡的餘弦定理計算可能會因為浮點數誤差導致無解，因此使用 try-except
            alpha1 = np.arccos((L1**2 + r_M1_pen**2 - L3**2) / (2 * L1 * r_M1_pen))
            beta2 = np.arccos((L2**2 + r_M2_pen**2 - L4**2) / (2 * L2 * r_M2_pen))

        except (ValueError, ZeroDivisionError):
            return False

    return True

def find_minimum_link_lengths():
    """
    逐步縮小連桿長度，直到無法完成繪圖任務為止
    """
    current_L0, current_L1, current_L2, current_L3, current_L4 = L0_base, L1_base, L2_base, L3_base, L4_base

    step = 0.1  # 每次縮小的步長 (cm)

    print("開始逐步縮小連桿長度並測試...")

    while True:
        # 測試當前配置
        if inverse_kinematics_check(current_L0, current_L1, current_L2, current_L3, current_L4):
            print(f"✅ 成功: L0={current_L0:.2f}, L1={current_L1:.2f}, L2={current_L2:.2f}, L3={current_L3:.2f}, L4={current_L4:.2f}")

            # 儲存上一次成功的配置
            last_successful_config = (current_L0, current_L1, current_L2, current_L3, current_L4)

            # 繼續縮小連桿
            current_L0 -= step
            current_L1 -= step
            current_L2 -= step
            current_L3 -= step
            current_L4 -= step
        else:
            print(f"❌ 失敗: L0={current_L0:.2f}, L1={current_L1:.2f}, L2={current_L2:.2f}, L3={current_L3:.2f}, L4={current_L4:.2f}")
            print("---")
            print("已達到連桿極限，無法完成繪圖。找到接近最小的連桿配置:")
            final_L0, final_L1, final_L2, final_L3, final_L4 = last_successful_config
            print(f"L0={final_L0:.2f}, L1={final_L1:.2f}, L2={final_L2:.2f}, L3={final_L3:.2f}, L4={final_L4:.2f}")
            return final_L0, final_L1, final_L2, final_L3, final_L4

# 尋找最小桿長並執行模擬
final_L0, final_L1, final_L2, final_L3, final_L4 = find_minimum_link_lengths()

# 根據找到的最小長度重新計算所有路徑點的角度
motor_angles = []
for p in path_points:
    try:
        M1 = np.array([0, 0])
        M2 = np.array([final_L0, 0])
        x, y = p[0], p[1]

        r_M1_pen = np.linalg.norm(np.array([x, y]) - M1)
        r_M2_pen = np.linalg.norm(np.array([x, y]) - M2)

        alpha1 = np.arccos((final_L1**2 + r_M1_pen**2 - final_L3**2) / (2 * final_L1 * r_M1_pen))
        theta_pen_M1 = np.arctan2(y - M1[1], x - M1[0])
        theta1 = theta_pen_M1 + alpha1

        beta2 = np.arccos((final_L2**2 + r_M2_pen**2 - final_L4**2) / (2 * final_L2 * r_M2_pen))
        theta_pen_M2 = np.arctan2(y - M2[1], x - M2[0])
        theta2 = theta_pen_M2 - beta2

        motor_angles.append((np.degrees(theta1), np.degrees(theta2)))
    except (ValueError, ZeroDivisionError):
        # 應不發生，因為我們已事先檢查
        pass

fig, ax = plt.subplots(figsize=(10, 10))

# 設定繪圖範圍
ax.set_xlim(-5, final_L0 + 5)
ax.set_ylim(-5, 30)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title(f'5-bar Linkage Plotter — min link length design L0={final_L0:.2f}cm')

# 繪圖區域
draw_area = plt.Rectangle((4, 4), 20, 20, linewidth=2, edgecolor='red', facecolor='none', linestyle='--', label='20cm x 20cm plot area')
ax.add_patch(draw_area)

# 馬達點標示
M1 = np.array([0, 0])
M2 = np.array([final_L0, 0])
ax.plot([M1[0], M2[0]], [M1[1], M2[1]], 'ko')
ax.text(M1[0], M1[1]-2, 'M1', ha='center')
ax.text(M2[0], M2[1]-2, 'M2', ha='center')

lines, = ax.plot([], [], 'o-', lw=3, color='blue')
pen_point, = ax.plot([], [], 'ro', markersize=8, label='pen')
pen_path, = ax.plot([], [], 'r-', lw=1.5, alpha=0.8, label='pen contour')
drawn_path_x = []
drawn_path_y = []

def update(frame):
    if frame >= len(motor_angles):
        return lines, pen_point, pen_path

    theta1, theta2 = motor_angles[frame]
    pen = path_points[frame]

    P1 = M1 + final_L1 * np.array([np.cos(np.radians(theta1)), np.sin(np.radians(theta1))])
    P2 = M2 + final_L2 * np.array([np.cos(np.radians(theta2)), np.sin(np.radians(theta2))])

    x_vals = [M1[0], P1[0], pen[0], P2[0], M2[0]]
    y_vals = [M1[1], P1[1], pen[1], P2[1], M2[1]]

    lines.set_data(x_vals, y_vals)
    pen_point.set_data([pen[0]], [pen[1]])
    
    drawn_path_x.append(pen[0])
    drawn_path_y.append(pen[1])
    pen_path.set_data(drawn_path_x, drawn_path_y)

    return lines, pen_point, pen_path

ani = FuncAnimation(fig, update, frames=len(motor_angles), interval=50, blit=True)

plt.legend()
plt.show()

# --- 新增的 GIF 產生功能 ---
# 將動畫儲存為 GIF 檔案
# dpi (每英吋點數) 決定了 GIF 的解析度
# figsize 10，要寬度 600 像素，則 dpi = 60
ani.save('5_bar_linkage_600px.gif', writer='pillow', dpi=60)
print("\nGIF檔案已儲存為 5_bar_linkage_600px.gif")