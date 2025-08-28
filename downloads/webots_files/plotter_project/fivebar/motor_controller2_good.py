from controller import Supervisor
import math
import numpy as np

# --- 機構參數（單位：米）---
L1 = L4 = 0.1682
L2 = L3 = 0.275
A = np.array([0.0625, 0.15])
E = np.array([-0.0625, 0.15])
offset = math.radians(41.9872) # 這個 offset 的來源和確切定義非常重要

# --- 幾何函數 ---
def circle_circle_intersection(P0, r0, P1, r1):
    d = np.linalg.norm(P1 - P0)
    # If circles are too far apart, too close, or coincident
    if d > r0 + r1 or d < abs(r0 - r1) or d == 0:
        return []
    
    a = (r0**2 - r1**2 + d**2) / (2 * d)
    
    # h^2 can be very small negative due to floating point inaccuracies, clamp to 0
    h = math.sqrt(max(0.0, r0**2 - a**2))
    
    vec = (P1 - P0) / d # Unit vector from P0 to P1
    P2 = P0 + a * vec  # Point on the line P0P1
    offset_vec = h * np.array([-vec[1], vec[0]]) # Perpendicular vector
    
    if h == 0: # Tangent case
        return [P2]
    return [P2 + offset_vec, P2 - offset_vec]

# --- 正運動學 (確認為正確版本) ---
def forward_kinematics(theta1_rad, theta2_rad):
    # theta1_rad: 從 A 點出發，相對於「A到B的零點方向+offset」的角度
    # B 相對於 A 的座標分量: L1 * cos(theta1_rad + offset) 在 X 軸， -L1 * sin(theta1_rad + offset) 在 Y 軸 (Y 軸減小表示順時針)
    B = A + L1 * np.array([math.cos(theta1_rad + offset), -math.sin(theta1_rad + offset)])
    
    # theta2_rad: 從 E 點出發，相對於「E到D的零點方向+offset」的角度
    # D 相對於 E 的座標分量: -L4 * cos(theta2_rad + offset) 在 X 軸， -L4 * sin(theta2_rad + offset) 在 Y 軸
    D = E - L4 * np.array([math.cos(theta2_rad + offset), math.sin(theta2_rad + offset)])
    
    C_solutions = circle_circle_intersection(B, L2, D, L3)
    
    # 選擇 Y 座標較小的解 (肘下姿態)
    return min(C_solutions, key=lambda p: p[1]) if C_solutions else None

# --- 修正後逆運動學 ---
def inverse_kinematics(C):
    C = np.array(C)

    # 找出 B 點：A 為圓心，L1 為半徑；C 為圓心，L2 為半徑
    B_candidates = circle_circle_intersection(A, L1, C, L2)
    if not B_candidates:
        return None, None
    # 選擇 Y 座標較小的 B 點 (通常是肘下姿態的 B 點)
    B = min(B_candidates, key=lambda p: p[1])

    # 找出 D 點：E 為圓心，L4 為半徑；C 為圓心，L3 為半徑
    D_candidates = circle_circle_intersection(E, L4, C, L3)
    if not D_candidates:
        return None, None
    # 選擇 Y 座標較小的 D 點 (通常是肘下姿態的 D 點)
    D = min(D_candidates, key=lambda p: p[1])

    # 由 A → B 向量計算 theta1
    vec_AB = B - A
    
    # 根据 forward_kinematics: B_x - A_x = L1 * cos(theta1_rad + offset)
    #                       B_y - A_y = -L1 * sin(theta1_rad + offset)
    # 这意味着 atan2( -(B_y - A_y), (B_x - A_x) ) = theta1_rad + offset
    theta1_rad_plus_offset = math.atan2(-vec_AB[1], vec_AB[0])
    theta1 = theta1_rad_plus_offset - offset

    # 由 E → D 向量計算 theta2
    vec_ED = D - E
    
    # 根据 forward_kinematics: D_x - E_x = -L4 * cos(theta2_rad + offset)
    #                       D_y - E_y = -L4 * sin(theta2_rad + offset)
    # 这意味着 atan2( -(D_y - E_y), -(D_x - E_x) ) = theta2_rad + offset
    # 或者 atan2( vec_ED[1], vec_ED[0] ) = (theta2_rad + offset) + pi (如果vec_ED在负半轴)
    # 简化为: atan2( -vec_ED[1], -vec_ED[0] ) = theta2_rad + offset
    theta2_rad_plus_offset = math.atan2(-vec_ED[1], -vec_ED[0])
    theta2 = theta2_rad_plus_offset - offset

    return math.degrees(theta1), math.degrees(theta2)

# --- 初始化裝置 ---
robot = Supervisor()
timestep = 32
theta1_motor = robot.getDevice("theta1")
theta2_motor = robot.getDevice("theta2")
sensor1 = robot.getDevice("sensor1")
sensor2 = robot.getDevice("sensor2")
sensor1.enable(timestep)
sensor2.enable(timestep)
theta1_motor.setPosition(float('inf'))
theta2_motor.setPosition(float('inf'))
theta1_motor.setVelocity(10.0)
theta2_motor.setVelocity(10.0)

# --- 圓形路徑參數 ---
circle_center = np.array([0.0, -0.13]) # 保持這個測試圓心
circle_radius = 0.1 # 保持這個測試半徑
N = 12 # 保持 4 個點方便測試
circular_points = []
for i in range(N):
    angle = 2 * math.pi * i / N
    x = circle_center[0] + circle_radius * math.cos(angle)
    y = circle_center[1] + circle_radius * math.sin(angle)
    circular_points.append(np.array([x, y]))

# --- 逆運動學驗證 ---
print("=== 逆運動學驗證 ===")
ik_angles = []
for i, C in enumerate(circular_points):
    t1_deg, t2_deg = inverse_kinematics(C)
    
    if t1_deg is None or t2_deg is None:
        print(f"[點 {i+1}] 目標 C=({C[0]:.4f}, {C[1]:.4f}) -> IK解失敗 (不可達或多解問題)")
        ik_angles.append((None, None)) # 標記為失敗
        continue

    t1_rad = math.radians(t1_deg)
    t2_rad = math.radians(t2_deg)
    
    C_fk = forward_kinematics(t1_rad, t2_rad)
    
    error = float('inf')
    if C_fk is not None:
        dx, dy = C_fk - C
        error = math.hypot(dx, dy)
        print(f"[點 {i+1}] 目標 C=({C[0]:.4f}, {C[1]:.4f})")
        print(f"      IK: θ1={t1_deg:.2f}°, θ2={t2_deg:.2f}°")
        print(f"      FK: C=({C_fk[0]:.4f}, {C_fk[1]:.4f}), 誤差={error:.6f} m")
    else:
        print(f"[點 {i+1}] 目標 C=({C[0]:.4f}, {C[1]:.4f}) -> FK驗證失敗 (IK解可能無效)")
    
    ik_angles.append((t1_deg, t2_deg))

# --- 控制流程 ---
print("\n=== 模擬開始 ===")
index = 0
pause = 0
WAIT = 30
target_set = True

while robot.step(timestep) != -1:
    # 過濾掉 IK 解失敗的點
    while index < len(ik_angles) and (ik_angles[index][0] is None or ik_angles[index][1] is None):
        index += 1
        if index >= len(ik_angles):
            break # 所有點都檢查完了

    if index >= len(ik_angles):
        print("✔️ 所有可達點已完成")
        break

    # 獲取實際馬達角度並通過 FK 計算當前 C 點
    theta1_sensor_rad = -sensor1.getValue()  # θ1 方向修正
    theta2_sensor_rad = sensor2.getValue()
    C_now = forward_kinematics(theta1_sensor_rad, theta2_sensor_rad)
    if C_now is not None:
        # print(f"[模擬] 當前 C = ({C_now[0]:.4f}, {C_now[1]:.4f})") # 這行如果頻繁打印會刷屏

        # 可選: 打印與當前目標點的距離
        if not target_set and pause > 0: # 只有在運動過程中才計算距離
            target_C_coord = circular_points[index-1] # index-1 是當前目標點
            dist_to_target = math.hypot(C_now[0] - target_C_coord[0], C_now[1] - target_C_coord[1])
            # print(f"  距離目標點 {index-1}: {dist_to_target:.6f} m") # 可以在需要時打開

    if target_set and pause == 0:
        t1_deg, t2_deg = ik_angles[index]
        theta1_motor.setPosition(math.radians(-t1_deg)) # 馬達方向修正
        theta2_motor.setPosition(math.radians(t2_deg))   # 馬達方向修正
        print(f"\n[命令] 點 {index+1}: 目標 C=({circular_points[index][0]:.4f}, {circular_points[index][1]:.4f}) IK: θ1={t1_deg:.2f}°, θ2={t2_deg:.2f}°")
        pause = WAIT
        target_set = False
        index += 1
    elif pause > 0:
        pause -= 1
    else:
        target_set = True