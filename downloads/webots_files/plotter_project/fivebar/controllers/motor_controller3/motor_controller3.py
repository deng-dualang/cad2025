from controller import Supervisor
import math
import numpy as np

# --- 機構參數 ---
# 確認 L1 和 L4 長度為 16.82
L1 = L4 = 16.82 
L2 = L3 = 27.5    # 保持不變
A = np.array([6.25, 15.0]) # 關節 A (theta1 旋轉軸) 的座標
E = np.array([-6.25, 15.0]) # 關節 E (theta2 旋轉軸) 的座標

# --- 圓圓相交函數 ---
def circle_circle_intersection(P0, r0, P1, r1):
    d = np.linalg.norm(P1 - P0)
    
    if d > r0 + r1 or d < abs(r0 - r1) or d == 0:
        return []
    
    a = (r0**2 - r1**2 + d**2) / (2 * d)
    
    h_squared = r0**2 - a**2
    if h_squared < 0:
        h = 0.0
    else:
        h = np.sqrt(h_squared)

    vec = (P1 - P0) / d
    P2 = P0 + a * vec
    offset = h * np.array([-vec[1], vec[0]])
    
    if h == 0.0:
        return [P2]
    return [P2 + offset, P2 - offset]

# --- 正運動學函數 (Forward Kinematics) ---
# 根據給定的 theta1 和 theta2 (弧度) 計算末端點 C 的座標
# 這裡的角度定義應與你「期望的物理行為」一致：
# theta1: 當其值為正時，導致連桿順時針轉動 (Y 減少)
# theta2: 當其值為正時，導致連桿逆時針轉動 (Y 增加)
def forward_kinematics(theta1_rad, theta2_rad):
    # 推導 B 點座標 (從 A 點出發，距離 L1)
    # theta1 為正時，順時針轉動，Y 減少
    Bx = A[0] + L1 * math.cos(theta1_rad)
    By = A[1] - L1 * math.sin(theta1_rad) # <--- 保持負號 (t1 順時針為正)
    B_coord = np.array([Bx, By])

    # 推導 D 點座標 (從 E 點出發，距離 L4)
    # theta2 為正時，逆時針轉動，Y 增加 (根據你最終確認的定義！)
    Dx = E[0] + L4 * math.cos(theta2_rad)
    Dy = E[1] + L4 * math.sin(theta2_rad) # <--- 修正：改回正號 (t2 逆時針為正)
    D_coord = np.array([Dx, Dy])

    # C 點是 B 點為圓心、L2 半徑的圓，與 D 點為圓心、L3 半徑的圓的交點
    C_solutions = circle_circle_intersection(B_coord, L2, D_coord, L3)
    
    if C_solutions:
        if len(C_solutions) == 1:
            return C_solutions[0]
        else:
            # 如果有兩個解 (肘上/肘下)，我們通常選擇 Y 座標較小的那個 (肘下姿態)。
            # 請確保這符合你的實際機構動作。
            if C_solutions[0][1] < C_solutions[1][1]:
                return C_solutions[0]
            else:
                return C_solutions[1]
    return None # 無法找到 C 點 (可能機構在該角度配置下無法閉合)

# --- 初始化 Supervisor ---
robot = Supervisor()
timestep = 32 # 固定 timestep (ms)，請確保與你的 Webots 世界設定一致

# 獲取馬達設備
theta1_motor = robot.getDevice("theta1") 
theta2_motor = robot.getDevice("theta2") 

# 獲取感測器設備
sensor1 = robot.getDevice("sensor1")
sensor2 = robot.getDevice("sensor2")

# 啟用感測器
if sensor1:
    sensor1.enable(timestep)
else:
    print("錯誤：無法找到 'sensor1'。請檢查 Webots 模型中的設備名稱。")
    robot.simulationQuit(1)

if sensor2:
    sensor2.enable(timestep)
else:
    print("錯誤：無法找到 'sensor2'。請檢查 Webots 模型中的設備名稱。")
    robot.simulationQuit(1)

# 設定馬達為位置控制模式並設定速度
theta1_motor.setPosition(float('inf'))
theta2_motor.setPosition(float('inf'))
theta1_motor.setVelocity(5.0) # 馬達速度 (弧度/秒)，可調整
theta2_motor.setVelocity(5.0) # 馬達速度 (弧度/秒)，可調整

# --- 定義目標角度點 (單位：度) ---
# 這些是你期望的「物理上的」目標角度：
# t1 的正值為順時針旋轉
# t2 的正值為逆時針旋轉
target_angles_deg = [
    (15, 45),
    (30, 60),
    (60, 30),
    (45, 15)
]

# --- 預計算並繪製 C 點路徑 (可選，用於視覺化) ---
target_c_points_for_drawing = []
print("--- 預計算目標角度與 C 點座標 ---")
for i, (t1_deg_original, t2_deg_original) in enumerate(target_angles_deg):
    # 為 forward_kinematics 提供「物理上期望的」角度。
    # 因為 FK 內部已根據物理定義處理正負號。
    t1_rad_for_fk = math.radians(t1_deg_original) 
    t2_rad_for_fk = math.radians(t2_deg_original) 
    
    C_coord = forward_kinematics(t1_rad_for_fk, t2_rad_for_fk)
    if C_coord is not None:
        target_c_points_for_drawing.append(C_coord)
        print(f"預計點 {i+1}: 期望角度=(t1={t1_deg_original:.2f}°, t2={t2_deg_original:.2f}°) -> C_點=({C_coord[0]:.4f}, {C_coord[1]:.4f})")
    else:
        print(f"警告：預計點 {i+1} (期望角度 t1={t1_deg_original:.2f}°, t2={t2_deg_original:.2f}°) 無法通過正運動學找到 C 點。")
print("---------------------------------------")

# 在 Webots 世界中繪製預計路徑的紅線
if len(target_c_points_for_drawing) >= 2:
    proto = """DEF TEST_PATH Shape {
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
    for point_coord in target_c_points_for_drawing:
        # Z 軸高度可調整，確保在機器人上方可見
        proto += f"{point_coord[0]:.6f} {point_coord[1]:.6f} 0.18,\n" 
    proto += "]\n}\ncoordIndex [\n"

    for i in range(len(target_c_points_for_drawing)):
        proto += f"{i}, {(i + 1) % len(target_c_points_for_drawing)}, -1,\n"
    proto += "]\n} }\n"

    robot.getRoot().getField("children").importMFNodeFromString(-1, proto)
else:
    print("無法生成足夠的點來繪製路徑。")

# --- 控制邏輯 ---
print("\n--- 機械臂運動控制與即時資料列印 ---")
current_point_index = 0 
pause_counter = 0       
target_set = True       
POINTS_PAUSE_DURATION = 30 # 每個點停留的 timesteps 數量

# --- 主控制迴圈 ---
while robot.step(timestep) != -1:
    # --- 退出條件 ---
    if current_point_index >= len(target_angles_deg):
        print("所有指定角度點已測試完畢。程式結束。")
        break

    # --- 狀態機核心邏輯 ---
    if target_set and pause_counter == 0:
        # 1. 取得目標角度 (原始物理定義)
        t1_deg_original, t2_deg_original = target_angles_deg[current_point_index]

        # **修正：將 t1 的原始角度取負，以抵消 Webots 馬達的「逆時針為正」的行為**
        # **t2 馬達：直接發送正值，因為它就是要逆時針轉，且馬達「正」是逆時針。**
        t1_rad_for_motor = math.radians(-t1_deg_original) 
        t2_rad_for_motor = math.radians(t2_deg_original) # <--- 修正：t2 不再取負，直接發送

        # 2. 計算並即時列印 C 點座標 (使用原始物理角度，因為 FK 已處理正負)
        C_coord_at_target_angle = forward_kinematics(math.radians(t1_deg_original), math.radians(t2_deg_original))
        
        # 3. 設定馬達的目標位置 (發送經過調整的弧度值)
        theta1_motor.setPosition(t1_rad_for_motor)
        theta2_motor.setPosition(t2_rad_for_motor)

        # 4. 打印設定信息，包含 C 點座標
        print(f"\n--- Timestep: {robot.getTime():.2f}s, 點 {current_point_index+1}/{len(target_angles_deg)} ---")
        print(f"  期望物理角度: t1={t1_deg_original:.2f}°, t2={t2_deg_original:.2f}°")
        print(f"  實際發送給Webots馬達: t1_motor={math.degrees(t1_rad_for_motor):.2f}°, t2_motor={math.degrees(t2_rad_for_motor):.2f}°")
        if C_coord_at_target_angle is not None:
            print(f"  預計 C 點座標 (由FK計算): ({C_coord_at_target_angle[0]:.4f}, {C_coord_at_target_angle[1]:.4f})")
        else:
            print("  警告：無法計算此目標角度對應的 C 點座標。")
        
        # 5. 進入等待階段
        target_set = False 
        pause_counter = POINTS_PAUSE_DURATION 
        current_point_index += 1 

    elif pause_counter > 0:
        # 處於延遲狀態，只遞減計數器
        pause_counter -= 1
    
    else: 
        # 延遲結束，準備設定下一個目標
        target_set = True