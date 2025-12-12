"""draw_apple_supervisor controller."""

from controller import Supervisor
import cv2
import numpy as np
import math

# ----------------- 固定參數 (米) -----------------
L1, L2, L3, L4, L5 = 0.20, 0.27, 0.27, 0.20, 0.20
Ex = L5  # E 點的 x 座標 (0.2)

# 目標圓心與半徑
target_center = np.array([0.1, 0.3])
target_radius = 0.1
N_POINTS = 50

# ----------------- 反解函式 -----------------
def ik(cx: float, cy: float) -> tuple:
    L_AC_sq = cx**2 + cy**2
    L_AC = math.sqrt(L_AC_sq)
    if L_AC > (L1 + L2) or L_AC < abs(L1 - L2) or L_AC == 0:
        return (None, None)
    alpha = math.atan2(cy, cx)
    cos_beta1 = (L1**2 + L_AC_sq - L2**2) / (2 * L1 * L_AC)
    cos_beta1 = np.clip(cos_beta1, -1.0, 1.0)
    beta1 = math.acos(cos_beta1)
    sigma1 = 1
    t1 = alpha + sigma1 * beta1

    dx, dy = cx - Ex, cy
    L_EC_sq = dx**2 + dy**2
    L_EC = math.sqrt(L_EC_sq)
    if L_EC > (L4 + L3) or L_EC < abs(L4 - L3) or L_EC == 0:
        return (None, None)
    gamma = math.atan2(dy, dx)
    cos_beta2 = (L4**2 + L_EC_sq - L3**2) / (2 * L4 * L_EC)
    cos_beta2 = np.clip(cos_beta2, -1.0, 1.0)
    beta2 = math.acos(cos_beta2)
    sigma2 = -1
    t2 = gamma + sigma2 * beta2

    t1_rad = t1 % (2 * math.pi)
    t2_rad = t2 % (2 * math.pi)
    return math.degrees(t1_rad), math.degrees(t2_rad)

# ----------------- 蘋果輪廓處理 -----------------
def load_apple_points(image_path, N_points=100):
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Error: 無法讀取圖片 {image_path}")
        exit(1)
    _, binary = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        print("Error: 找不到輪廓")
        exit(1)
    main_contour = max(contours, key=cv2.contourArea)
    if len(main_contour) > N_points:
        indices = np.linspace(0, len(main_contour) - 1, N_points, dtype=int)
        sampled = main_contour[indices]
    else:
        sampled = main_contour
    pts = sampled[:, 0, :]
    h, w = binary.shape
    pts[:, 1] = h - pts[:, 1]
    center = np.mean(pts, axis=0)
    pts_centered = pts - center
    scale = np.max(np.linalg.norm(pts_centered, axis=1))
    pts_normalized = pts_centered / scale
    pts_scaled = pts_normalized * target_radius + target_center
    return pts_scaled

# ----------------- IndexedLineSet 字串生成 -----------------
def generate_indexed_line_set(points):
    point_str = ",\n      ".join([f"{x:.5f} {y:.5f} 0.001" for x, y in points])
    indices = ", ".join([str(i) for i in range(len(points))]) + ", 0"  # 形成封閉線條
    node_str = f"""
DEF APPLE_SHAPE Shape {{
  appearance Appearance {{
    material Material {{
      diffuseColor 1 0 0
      emissiveColor 1 0 0
    }}
  }}
  geometry IndexedLineSet {{
    coord Coordinate {{
      point [
        {point_str}
      ]
    }}
    coordIndex [
      {indices}
    ]
  }}
}}
"""
    return node_str

# ----------------- 主流程 -----------------
def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    # 載入輪廓點
    apple_points = load_apple_points("apple-clip-art.jpg", N_points=N_POINTS)

    # 建立 IndexedLineSet 串接綠線
    root = robot.getRoot()
    children_field = root.getField("children")
    line_node = generate_indexed_line_set(apple_points)
    children_field.importMFNodeFromString(-1, line_node)

    # 取得馬達
    t1 = robot.getDevice("t1")
    t2 = robot.getDevice("t2")
    t1.setVelocity(5.0)
    t2.setVelocity(5.0)
    deg = math.pi / 180
    t1_offset = -90
    t2_offset = -17.9

    # 畫圖：依序移動到每個輪廓點
    for (x, y) in apple_points:
        t1_deg, t2_deg = ik(x, y)
        if t1_deg is None or t2_deg is None:
            print(f"點 ({x:.3f}, {y:.3f}) 超出可達範圍，略過")
            continue
        t1_rad = (t1_deg + t1_offset) * deg
        t2_rad = (t2_deg + t2_offset) * deg
        t1.setPosition(t1_rad)
        t2.setPosition(t2_rad)

        # 簡單延遲讓 plotter 有時間移動
        for _ in range(5):
            if robot.step(timestep) == -1:
                return
        print(f"描點: ({x:.3f}, {y:.3f})")

if __name__ == "__main__":
    main()
