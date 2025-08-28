# pip install numpy opencv-python
from controller import Supervisor
import cv2
import numpy as np
import math

# === 機構參數 ===
L1 = L4 = 16.82
L2 = L3 = 27.5
A = np.array([6.25, 15.0])
E = np.array([-6.25, 15.0])

def inverse_kinematics(C):
    x, y = C
    BA = circle_circle_intersection(A, L1, C, L2)
    DE = circle_circle_intersection(E, L4, C, L3)
    solutions = []
    for B in BA:
        for D in DE:
            dx1, dy1 = B - A
            theta1 = np.arctan2(-dy1, dx1)
            dx2, dy2 = D - E
            theta2 = np.arctan2(dy2, dx2)
            solutions.append((theta1, theta2))
    return solutions

def circle_circle_intersection(P0, r0, P1, r1):
    d = np.linalg.norm(P1 - P0)
    if d > r0 + r1 or d < abs(r0 - r1) or d == 0:
        return []
    a = (r0**2 - r1**2 + d**2) / (2 * d)
    h = np.sqrt(r0**2 - a**2)
    vec = (P1 - P0) / d
    P2 = P0 + a * vec
    offset = h * np.array([-vec[1], vec[0]])
    return [P2 + offset, P2 - offset]

# === 初始化 Supervisor ===
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
theta1 = robot.getDevice("theta1")
theta2 = robot.getDevice("theta2")
theta1.setPosition(float('inf'))
theta2.setPosition(float('inf'))
theta1.setVelocity(1.0)
theta2.setVelocity(1.0)

# === 輪廓圖形處理 ===
img = cv2.imread("mickey_outline.png", cv2.IMREAD_GRAYSCALE)
_, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
contour = max(contours, key=cv2.contourArea)
N = 64
indices = np.linspace(0, len(contour) - 1, N, dtype=int)
points = np.array([contour[i][0] for i in indices], dtype=np.float32)
# Flip vertically and scale
points[:, 1] *= -1
points -= np.mean(points, axis=0)
points /= 9000
points += np.array([0.0, -0.15])
# 大寫的 Points 為 [[-0.0715, -0.240], ......] 各點只到小數點後四位
Points = [[round(x, 4), round(y, 4)] for x, y in points.tolist()] + [[round(x, 4), round(y, 4)] for x, y in [points[0]]]

# === 加入紅線目標路徑 ===
proto = """DEF TARGET_PATH Shape {
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
for x, y in Points:
    proto += f"{x:.6f} {y:.6f} 0.18,\n"
proto += "]\n}\ncoordIndex [\n"
for i in range(len(Points) - 1):
    proto += f"{i}, {i+1}, -1,\n"
proto += "]\n} }\n"
robot.getRoot().getField("children").importMFNodeFromString(-1, proto)

# === 控制邏輯 ===
print("通過點與對應角度：")
index = 0
pause_counter = 0
target_reached = True

while robot.step(timestep) != -1:
    if index >= len(Points):
        break
    if target_reached and pause_counter == 0:
        C = np.array(Points[index])
        solutions = inverse_kinematics(C)
        if solutions:
            # 取第一組解
            t1, t2 = solutions[0]
            print(C, t1, t2)
            theta1.setPosition(t1)
            theta2.setPosition(t2)
        else:
            print(f"無法解出逆解：{C}")
            theta1_list.append(None)
            theta2_list.append(None)
        index += 1
        target_reached = False
        pause_counter = 20  # 模擬每點停留時間
    elif pause_counter > 0:
        pause_counter -= 1
    else:
        target_reached = True