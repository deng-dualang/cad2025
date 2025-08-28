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

# 啟用感測器以讀取馬達實際位置
sensor1 = theta1.getPositionSensor()
sensor2 = theta2.getPositionSensor()
sensor1.enable(timestep)
sensor2.enable(timestep)

def angle_reached(sensor, target, tol=0.01):
    current = sensor.getValue()
    return abs(current - target) < tol

# === 輪廓圖形處理 ===
img = cv2.imread("mickey_outline.png", cv2.IMREAD_GRAYSCALE)
_, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
contour = max(contours, key=cv2.contourArea)
N = 64
indices = np.linspace(0, len(contour) - 1, N, dtype=int)
points = np.array([contour[i][0] for i in indices], dtype=np.float32)
points -= np.mean(points, axis=0)
points /= 9000
points += np.array([0.0, -0.15])
C_targets = points.tolist() + [points[0]]

# === 加入紅線目標路徑 ===
proto_red = """DEF TARGET_PATH Shape {
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
for x, y in C_targets:
    proto_red += f"{x:.6f} {y:.6f} 0.18,\n"
proto_red += "]\n}\ncoordIndex [\n"
for i in range(len(C_targets) - 1):
    proto_red += f"{i}, {i+1}, -1,\n"
proto_red += "]\n} }\n"
robot.getRoot().getField("children").importMFNodeFromString(-1, proto_red)

# === 控制邏輯 ===
actual_C = []
index = 0
current_goal = None

while robot.step(timestep) != -1:
    if index >= len(C_targets):
        break

    if current_goal is None:
        C = np.array(C_targets[index])
        solutions = inverse_kinematics(C)
        if solutions:
            # 取第一組解
            θ1, θ2 = solutions[0]
            theta1.setPosition(θ1)
            theta2.setPosition(θ2)
            current_goal = (θ1, θ2)
        else:
            print(f"⚠️ 無法解出逆解：{C}")
            index += 1

    elif current_goal:
        r1 = angle_reached(sensor1, current_goal[0])
        r2 = angle_reached(sensor2, current_goal[1])
        if r1 and r2:
            Bx = A[0] + L1 * math.cos(current_goal[0])
            By = A[1] - L1 * math.sin(current_goal[0])
            Dx = E[0] + L4 * math.cos(current_goal[1])
            Dy = E[1] + L4 * math.sin(current_goal[1])
            Cx = (Bx + Dx) / 2
            Cy = (By + Dy) / 2
            actual_C.append((Cx, Cy))
            index += 1
            current_goal = None

# === 綠線實際走過的路徑 ===
proto_green = """DEF ACTUAL_PATH Shape {
  appearance Appearance {
    material Material {
      diffuseColor 0 1 0
      emissiveColor 0 1 0
    }
  }
  geometry IndexedLineSet {
    coord Coordinate {
      point [
"""
for x, y in actual_C:
    proto_green += f"{x:.6f} {y:.6f} 0.181,\n"
proto_green += "]\n}\ncoordIndex [\n"
for i in range(len(actual_C) - 1):
    proto_green += f"{i}, {i+1}, -1,\n"
proto_green += "]\n} }\n"
robot.getRoot().getField("children").importMFNodeFromString(-1, proto_green)