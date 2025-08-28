from controller import Supervisor
import math
import matplotlib.pyplot as plt
from inverse_kinematics import inverse

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# 初始化馬達
theta1 = robot.getDevice("theta1")
theta2 = robot.getDevice("theta2")
theta1.setPosition(float('inf'))
theta2.setPosition(float('inf'))
theta1.setVelocity(1.0)
theta2.setVelocity(1.0)

# 桿件長度（單位：m）
L1 = 67.27 / 400
L2 = 110 / 400
L3 = 110 / 400
L4 = 67.27 / 400

# 圓軌跡參數
center_x = 25 / 400
center_y = -125.47 / 400
radius = 50 / 400
N = 12

# 建立通過點
points = []
theta1_list, theta2_list = [], []
for i in range(N + 1):
    angle = 2 * math.pi * i / N
    x = center_x + radius * math.cos(angle)
    y = center_y + radius * math.sin(angle)
    points.append((x, y))

# === 加入粗紅色連線表示畫筆軌跡 ===
proto = """
DEF DRAWN_PATH Shape {
  appearance Appearance {
    material Material {
      diffuseColor 1 0 0
      emissiveColor 1 0 0
    }
  }
  geometry IndexedLineSet {
    coord Coordinate {
      point [
"""
for x, y in points:
    proto += f"{x} {y} 0.18,\n"
proto += "]\n"
proto += "}\n"
proto += "coordIndex [\n"
for i in range(len(points) - 1):
    proto += f"{i}, {i+1}, -1,\n"
proto += "]\n"
proto += "} }\n"

# 插入路徑進 Webots 世界
robot.getRoot().getField("children").importMFNodeFromString(-1, proto)

# 控制流程
print("通過點與對應角度：")
index = 0
pause_counter = 0
target_reached = True

while robot.step(timestep) != -1:
    if index >= len(points):
        break

    if target_reached and pause_counter == 0:
        cx, cy = points[index]
        result = inverse(cx, cy, L1, L2, L3, L4)
        if isinstance(result[0], tuple):
            t1, t2 = result[0]
            print(f"{index+1:02d}. ({cx:.5f}, {cy:.5f}) → θ1: {t1:.2f}°, θ2: {t2:.2f}°")
            theta1.setPosition(math.radians(t1))
            theta2.setPosition(math.radians(t2))
            theta1_list.append(t1)
            theta2_list.append(t2)
        else:
            print(f"{index+1:02d}. ({cx:.5f}, {cy:.5f}) → ❌ 無法求解")
            theta1_list.append(None)
            theta2_list.append(None)
        index += 1
        target_reached = False
        pause_counter = 20  # 模擬每點停留時間
    elif pause_counter > 0:
        pause_counter -= 1
    else:
        target_reached = True

# === 模擬結束後，繪圖分析 ===
cx_vals = [pt[0] for pt in points]
cy_vals = [pt[1] for pt in points]

plt.figure(figsize=(12, 5))

# C 點軌跡圖
plt.subplot(1, 2, 1)
plt.plot(cx_vals, cy_vals, 'ro-', label='C 軌跡')
plt.axis('equal')
plt.title("C 點繞行輪廓")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)

# θ1 / θ2 變化圖
plt.subplot(1, 2, 2)
plt.plot(theta1_list, 'b.-', label='theta1 (deg)')
plt.plot(theta2_list, 'g.-', label='theta2 (deg)')
plt.title("theta1 / theta2 變化趨勢")
plt.xlabel("點序")
plt.ylabel("角度 (°)")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()