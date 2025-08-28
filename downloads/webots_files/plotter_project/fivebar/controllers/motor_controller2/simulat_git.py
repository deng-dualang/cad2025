import cv2
import numpy as np
import math
import csv

# === 幾何設定 ===
scale = 1 / 400.0
L1 = L4 = 67.27 * scale
L2 = L3 = 110 * scale
A = np.array([50.0, 0.0]) * scale
E = np.array([0.0, 0.0]) * scale

def circle_circle_intersection(P0, r0, P1, r1):
    d = np.linalg.norm(P1 - P0)
    if d > r0 + r1 or d < abs(r0 - r1) or d == 0:
        return []
    a = (r0**2 - r1**2 + d**2) / (2 * d)
    h = np.sqrt(abs(r0**2 - a**2))
    v = (P1 - P0) / d
    P2 = P0 + a * v
    offset = h * np.array([-v[1], v[0]])
    return [P2 + offset, P2 - offset]

def inverse_kinematics(C):
    BA = circle_circle_intersection(A, L1, C, L2)
    DE = circle_circle_intersection(E, L4, C, L3)
    if BA and DE:
        B = BA[0]
        D = DE[1]
        θ1 = math.atan2(-B[1], B[0])  # AB：順時針為正
        θ2 = math.atan2(D[1], D[0] - E[0])  # ED：逆時針為正
        return θ1, θ2
    return None

# === 擷取輪廓點 ===
img = cv2.imread("mickey_outline.png", cv2.IMREAD_GRAYSCALE)
_, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
contour = max(contours, key=cv2.contourArea)

N = 64
indices = np.linspace(0, len(contour) - 1, N, dtype=int)
points = np.array([contour[i][0] for i in indices], dtype=np.float32)

# ❌ 不再翻轉 Y 軸：保持 OpenCV 原圖方向
points -= np.mean(points, axis=0)
points *= 0.06
points += np.array([25.0, -100.0])
points *= scale
C_path = points.tolist() + [points[0]]  # 封閉輪廓

# === 儲存為 CSV ===
with open("c_target.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y", "theta1_deg", "theta2_deg"])
    for C in C_path:
        res = inverse_kinematics(np.array(C))
        if res:
            θ1, θ2 = res
            writer.writerow([round(C[0],4), round(C[1],4), round(math.degrees(θ1),4), round(math.degrees(θ2),4)])

print("✅ 完成：c_target.csv 已生成")