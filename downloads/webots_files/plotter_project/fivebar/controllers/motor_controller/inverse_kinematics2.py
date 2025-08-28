from controller import Supervisor
import math
import cv2
import numpy as np

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# 馬達初始化
theta1 = robot.getDevice("theta1")
theta2 = robot.getDevice("theta2")
theta1.setPosition(float('inf'))
theta2.setPosition(float('inf'))
theta1.setVelocity(1.0)
theta2.setVelocity(1.0)

# 機構參數（單位：m）
L1 = 67.27 / 400
L2 = 110 / 400
L3 = 110 / 400
L4 = 67.27 / 400
A = (0.0, 0.0)
E = (-50.0 / 400, 0.0)

# 圖像輪廓分析
img = cv2.imread("mickey_outline.png", cv2.IMREAD_GRAYSCALE)
if img is None:
    raise FileNotFoundError("❌ 無法讀取圖檔 'mickey_outline.png'")

_, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
if not contours:
    raise ValueError("❌ 沒有發現輪廓")

contour = max(contours, key=cv2.contourArea)
N = 64
indices = np.linspace(0, len(contour) - 1, N, dtype=int)
points = np.array([contour[i][0] for i in indices], dtype=np.float32)

# 座標轉換：翻轉 Y 軸 → 平移中心 → 縮放到繪圖區 → A 換為 E 為原點 → 單位 mm→m
points[:, 1] *= -1
points -= np.mean(points, axis=0)
points *= 0.06
points += np.array([-25.0, -100.0])
points += np.array([50.0, 0.0])  # A → E 原點位移
C_path = (points / 400.0).tolist()  # 單位轉為公尺

# 逆向運動學
def circle_circle_intersection(P0, r0, P1, r1):
    dx, dy = P1[0] - P0[0], P1[1] - P0[1]
    d = math.hypot(dx, dy)
    if d > r0 + r1 or d < abs(r0 - r1) or d == 0:
        return []
    a = (r0**2 - r1**2 + d**2) / (2 * d)
    h = math.sqrt(max(0.0, r0**2 - a**2))
    x2 = P0[0] + a * dx / d
    y2 = P0[1] + a * dy / d
    rx, ry = -dy * h / d, dx * h / d
    return [(x2 + rx, y2 + ry), (x2 - rx, y2 - ry)]

def inverse(cx, cy):
    C = (cx, cy)
    BA = circle_circle_intersection(A, L1, C, L2)
    DE = circle_circle_intersection(E, L4, C, L3)
    if BA and DE:
        B = BA[0]
        D = DE[1]
        t1 = math.degrees(math.atan2(-B[1], B[0]))
        t2 = math.degrees(math.atan2(D[1], D[0] - E[0]))
        return (t1, t2)
    return (None, None)

# 繪製紅線路徑
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
for x, y in C_path:
    proto += f"{x:.6f} {y:.6f} 0.18,\n"
proto += "]\n}\ncoordIndex [\n"
for i in range(len(C_path) - 1):
    proto += f"{i}, {i+1}, -1,\n"
proto += "]\n} }\n"

robot.getRoot().getField("children").importMFNodeFromString(-1, proto)

# 控制流程
index = 0
pause_counter = 0
target_reached = True

while robot.step(timestep) != -1:
    if index >= len(C_path):
        break

    if target_reached and pause_counter == 0:
        cx, cy = C_path[index]
        t1, t2 = inverse(cx, cy)
        if t1 is not None:
            theta1.setPosition(math.radians(t1))
            theta2.setPosition(math.radians(t2))
            print(f"{index+1:02d}. ({cx:.4f}, {cy:.4f}) → θ1={t1:.2f}°, θ2={t2:.2f}°")
        else:
            print(f"{index+1:02d}. ({cx:.4f}, {cy:.4f}) → ❌ 無解")
        index += 1
        target_reached = False
        pause_counter = 20
    elif pause_counter > 0:
        pause_counter -= 1
    else:
        target_reached = True