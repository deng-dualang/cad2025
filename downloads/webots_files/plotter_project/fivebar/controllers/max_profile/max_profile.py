from controller import Supervisor
import math
import numpy as np
from scipy.spatial import ConvexHull

# --- 機構參數（單位：米）---
L1 = L4 = 0.1682
L2 = L3 = 0.275
A = np.array([0.0625, 0.15])
E = np.array([-0.0625, 0.15])
offset = math.radians(41.9872)

# --- 幾何計算 ---
def circle_circle_intersection(P0, r0, P1, r1):
    d = np.linalg.norm(P1 - P0)
    if d > r0 + r1 + 1e-9 or d < abs(r0 - r1) - 1e-9:
        return []
    if d < 1e-9 and abs(r0 - r1) < 1e-9:
        return []
    a = (r0**2 - r1**2 + d**2) / (2 * d)
    h2 = r0**2 - a**2
    if h2 < -1e-9:
        return []
    h = math.sqrt(max(h2, 0.0))
    vec = (P1 - P0) / d
    P2 = P0 + a * vec
    offset_vec = h * np.array([-vec[1], vec[0]])
    if h < 1e-12:
        return [P2]
    return [P2 + offset_vec, P2 - offset_vec]

def forward_kinematics(t1, t2):
    B = A + L1 * np.array([math.cos(t1 + offset), -math.sin(t1 + offset)])
    D = E - L4 * np.array([math.cos(t2 + offset), math.sin(t2 + offset)])
    pts = circle_circle_intersection(B, L2, D, L3)
    return min(pts, key=lambda p: p[1]) if pts else None

def inverse_kinematics(C):
    C = np.array(C)
    Bs = circle_circle_intersection(A, L1, C, L2)
    Ds = circle_circle_intersection(E, L4, C, L3)
    res = []
    for B in Bs:
        for D in Ds:
            t1 = math.atan2(-(B[1]-A[1]), B[0]-A[0]) - offset
            t2 = math.atan2(-(D[1]-E[1]), -(D[0]-E[0])) - offset
            t1d = (math.degrees(t1) + 180) % 360 - 180
            t2d = (math.degrees(t2) + 180) % 360 - 180
            res.append((t1d, t2d))
    return res

# --- Webots 初始化 ---
robot = Supervisor()
ts = int(robot.getBasicTimeStep())
m1 = robot.getDevice("theta1")
m2 = robot.getDevice("theta2")
m1.setPosition(float('inf'))
m2.setPosition(float('inf'))
m1.setVelocity(5.0)
m2.setVelocity(5.0)

# --- 掃描所有可達 C 點 ---
reachable_C = []
angle_step = 2
for t1 in range(-180, 181, angle_step):
    for t2 in range(-180, 181, angle_step):
        C = forward_kinematics(math.radians(t1), math.radians(t2))
        if C is not None:
            reachable_C.append(C)

# --- 畫紅色輪廓線（所有可達點） ---
def draw_polyline(points, color=(1,0,0), name="REACHABLE_SHAPE"):
    proto = f"DEF {name} Shape {{ appearance Appearance {{ material Material {{ diffuseColor {color[0]} {color[1]} {color[2]} emissiveColor {color[0]} {color[1]} {color[2]} }} }} geometry IndexedLineSet {{ coord Coordinate {{ point [\n"
    for p in points:
        proto += f"{p[0]:.6f} {p[1]:.6f} 0.18,\n"
    proto += "] } coordIndex [\n"
    for i in range(len(points)-1):
        proto += f"{i}, {i+1}, -1,\n"
    proto += f"{len(points)-1}, 0, -1,\n"
    proto += "] } }"
    robot.getRoot().getField("children").importMFNodeFromString(-1, proto)

draw_polyline(reachable_C, color=(1,0,0), name="REACHABLE_SHAPE")

# --- 找出凸包最大輪廓邊界 ---
pts_np = np.array(reachable_C)
hull = ConvexHull(pts_np)
hull_pts = pts_np[hull.vertices]
center = np.mean(hull_pts, axis=0)
angles = np.arctan2(hull_pts[:,1]-center[1], hull_pts[:,0]-center[0])
order = np.argsort(angles)
boundary_pts = hull_pts[order]

# --- 綠色輪廓線（最大封閉區域外框） ---
draw_polyline(boundary_pts, color=(0,1,0), name="OUTER_HULL_SHAPE")

# --- 邊界點轉為角度解（IK） ---
boundary_angles = []
for C in boundary_pts:
    sols = inverse_kinematics(C)
    if sols:
        best = min(sols, key=lambda s: abs(s[0]) + abs(s[1]))
        boundary_angles.append(best)

# --- 插補邊界角度點形成平滑連續運動路徑 ---
path = []
for i in range(len(boundary_angles)):
    a1, a2 = boundary_angles[i]
    b1, b2 = boundary_angles[(i+1)%len(boundary_angles)]
    for j in range(20):
        t = j/20
        path.append((a1 + (b1-a1)*t, a2 + (b2-a2)*t))

# --- 動畫控制器讓 Plotter 沿著外框走 ---
idx = 0
m1.setPosition(math.radians(-path[0][0]))
m2.setPosition(math.radians(path[0][1]))
print("✅ 準備繪製外圍輪廓（綠線）...")

while robot.step(ts) != -1:
    if idx >= len(path):
        print("✔️ 輪廓描繪完成")
        break
    t1, t2 = path[idx]
    m1.setPosition(math.radians(-t1))
    m2.setPosition(math.radians(t2))
    idx += 1
