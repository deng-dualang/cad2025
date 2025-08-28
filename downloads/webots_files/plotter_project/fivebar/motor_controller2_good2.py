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
    # 如果有多於一個解，確保總是選擇相同的解來保持運動連續性
    if C_solutions:
        if len(C_solutions) == 1:
            return C_solutions[0]
        else:
            # 選擇 Y 座標較小的那一個 (肘下姿態)
            # 這通常能保證連續的運動，避免機械臂翻轉
            if C_solutions[0][1] < C_solutions[1][1]:
                return C_solutions[0]
            else:
                return C_solutions[1]
    return None

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
theta1_motor.setPosition(float('inf')) # 設定為位置控制模式
theta2_motor.setPosition(float('inf'))
theta1_motor.setVelocity(5.0) # 預設速度，可在模擬中透過插補平滑化運動
theta2_motor.setVelocity(5.0)

# --- 圓形路徑參數 ---
circle_center = np.array([0.0, -0.13]) 
circle_radius = 0.1
N_main_points = 32 # 圓周上主要點的數量，這些點會被 IK 計算
circular_points = []
for i in range(N_main_points):
    angle = 2 * math.pi * i / N_main_points
    x = circle_center[0] + circle_radius * math.cos(angle)
    y = circle_center[1] + circle_radius * math.sin(angle)
    circular_points.append(np.array([x, y]))

# 將第一個點加到末尾，以形成閉合循環軌跡，方便插補
circular_points.append(circular_points[0]) 

# --- 逆運動學計算主要點的角度 ---
print("=== 逆運動學驗證與角度計算 ===")
ik_angles_for_main_points = [] # 儲存主要點的 IK 角度
for i, C in enumerate(circular_points):
    t1_deg, t2_deg = inverse_kinematics(C)
    
    if t1_deg is None or t2_deg is None:
        print(f"[點 {i+1}] 目標 C=({C[0]:.4f}, {C[1]:.4f}) -> IK解失敗 (不可達或多解問題), 此點將被跳過。")
        ik_angles_for_main_points.append((None, None)) # 標記為失敗
        continue

    t1_rad = math.radians(t1_deg)
    t2_rad = math.radians(t2_deg)
    
    C_fk = forward_kinematics(t1_rad, t2_rad)
    
    error = float('inf')
    if C_fk is not None:
        dx, dy = C_fk - C
        error = math.hypot(dx, dy)
        # 僅在驗證階段打印詳細信息，運行時保持控制台整潔
        # print(f"[點 {i+1}] 目標 C=({C[0]:.4f}, {C[1]:.4f})")
        # print(f"      IK: θ1={t1_deg:.2f}°, θ2={t2_deg:.2f}°")
        # print(f"      FK: C=({C_fk[0]:.4f}, {C_fk[1]:.4f}), 誤差={error:.6f} m")
    else:
        print(f"[點 {i+1}] 目標 C=({C[0]:.4f}, {C[1]:.4f}) -> FK驗證失敗 (IK解可能無效), 此點將被跳過。")
    
    ik_angles_for_main_points.append((t1_deg, t2_deg))

# 過濾掉無法計算 IK 的點，只保留可達點
valid_ik_angles = []
valid_circular_points = [] # 同步保存可達點的 Cartesian 座標
for i in range(len(ik_angles_for_main_points)):
    if ik_angles_for_main_points[i][0] is not None and ik_angles_for_main_points[i][1] is not None:
        valid_ik_angles.append(ik_angles_for_main_points[i])
        valid_circular_points.append(circular_points[i])

# 檢查是否還有足夠的可達點
if len(valid_ik_angles) < 2:
    print("錯誤: 沒有足夠的可達點來生成平滑軌跡。請檢查機構參數或圓形軌跡設定。")
    robot.simulationQuit(1)

# --- 關節空間插補軌跡生成 ---
interpolated_angles = []
SUB_POINTS_PER_SEGMENT = 20 # 每兩個主要點之間插入的子點數量，越大越平滑

for i in range(len(valid_ik_angles) - 1):
    start_t1_deg, start_t2_deg = valid_ik_angles[i]
    end_t1_deg, end_t2_deg = valid_ik_angles[i+1]

    # 處理角度跨越 +/-180 度邊界的問題 (短路徑插補)
    # 例如，從 170 度到 -170 度，不應該通過 0 度，而是通過 180 度。
    if abs(end_t1_deg - start_t1_deg) > 180:
        if end_t1_deg > start_t1_deg:
            start_t1_deg += 360
        else:
            end_t1_deg += 360
    
    if abs(end_t2_deg - start_t2_deg) > 180:
        if end_t2_deg > start_t2_deg:
            start_t2_deg += 360
        else:
            end_t2_deg += 360

    for j in range(SUB_POINTS_PER_SEGMENT):
        t = j / SUB_POINTS_PER_SEGMENT # 插補係數從 0 到 (SUB_POINTS_PER_SEGMENT-1)/SUB_POINTS_PER_SEGMENT
        
        # 線性插補關節角度
        interp_t1_deg = start_t1_deg + (end_t1_deg - start_t1_deg) * t
        interp_t2_deg = start_t2_deg + (end_t2_deg - start_t2_deg) * t
        
        # 規範化角度到 -180 到 180 之間
        interp_t1_deg = (interp_t1_deg + 180) % 360 - 180
        interp_t2_deg = (interp_t2_deg + 180) % 360 - 180

        interpolated_angles.append((interp_t1_deg, interp_t2_deg))

# 添加最後一個主要點的角度，確保完整覆蓋整個路徑
interpolated_angles.append(valid_ik_angles[-1])

# --- 在 Webots 世界中繪製預計路徑的綠線 ---
# 使用插補後的所有角度計算對應的 C 點，並繪製成綠線
interpolated_c_points_for_drawing = []
for t1_deg, t2_deg in interpolated_angles:
    C_reconstructed = forward_kinematics(math.radians(t1_deg), math.radians(t2_deg))
    if C_reconstructed is not None:
        interpolated_c_points_for_drawing.append(C_reconstructed)

if len(interpolated_c_points_for_drawing) >= 2:
    proto = """DEF CIRCLE_PATH Shape {
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
    for point_coord in interpolated_c_points_for_drawing:
        # Z 軸高度可調整，確保在機器人上方可見
        proto += f"{point_coord[0]:.6f} {point_coord[1]:.6f} 0.18,\n" 
    proto += "]\n}\ncoordIndex [\n"

    # 連接所有插補點形成平滑線條
    for i in range(len(interpolated_c_points_for_drawing) - 1):
        proto += f"{i}, {i + 1}, -1,\n"
    # 連接最後一個點回到第一個點，形成閉合圓
    proto += f"{len(interpolated_c_points_for_drawing)-1}, 0, -1,\n"
    proto += "]\n} }\n"

    robot.getRoot().getField("children").importMFNodeFromString(-1, proto)
else:
    print("無法生成足夠的插補點來繪製圓形路徑。")

# --- 控制流程 ---
print("\n=== 模擬開始 (關節空間平滑插補) ===")
current_interp_index = 0 # 當前目標插補點的索引
WAIT_AT_SUBPOINT = 1 # 每個插補子點停留的 timesteps 數量。越小運動越快。

# 設定初始位置到插補路徑的第一個點
if interpolated_angles:
    initial_t1_deg, initial_t2_deg = interpolated_angles[0]
    theta1_motor.setPosition(math.radians(-initial_t1_deg))
    theta2_motor.setPosition(math.radians(initial_t2_deg))
    print(f"[命令] 設定起始點: θ1={initial_t1_deg:.2f}°, θ2={initial_t2_deg:.2f}°")

while robot.step(timestep) != -1:
    # 檢查是否完成所有插補點
    if current_interp_index >= len(interpolated_angles):
        print("✔️ 所有插補點已完成")
        break

    # 獲取當前目標插補點的角度
    target_t1_deg, target_t2_deg = interpolated_angles[current_interp_index]

    # 將角度轉換為 Webots 馬達所需的弧度 (已根據方向調整)
    t1_rad_for_motor = math.radians(-target_t1_deg)
    t2_rad_for_motor = math.radians(target_t2_deg)

    # 設定馬達的目標位置
    theta1_motor.setPosition(t1_rad_for_motor)
    theta2_motor.setPosition(t2_rad_for_motor)

    # 每隔 WAIT_AT_SUBPOINT 個 timestep，移動到下一個插補點
    # 這裡的邏輯可以簡化為一個計數器
    if (robot.getTime() * 1000 // timestep) % WAIT_AT_SUBPOINT == 0:
        current_interp_index += 1
        # print(f"  [命令] 移至插補點 {current_interp_index}/{len(interpolated_angles)}") # 這行會非常頻繁地打印