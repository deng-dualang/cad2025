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
    """
    計算兩個圓的交點。
    P0, P1: 圓心 (numpy array)
    r0, r1: 半徑
    返回一個包含交點的列表 (最多兩個 numpy array)，如果沒有交點則返回空列表。
    """
    d = np.linalg.norm(P1 - P0) # 兩圓心距離
    
    # 檢查是否沒有交點（圓太遠或一個在另一個內部）
    # 增加一個小的容忍度來處理浮點數精度問題
    if d > r0 + r1 + 1e-9 or d < abs(r0 - r1) - 1e-9:
        return []
    # 檢查是否圓心重合且半徑相同（無限多解，視為無特定交點）
    if d < 1e-9 and abs(r0 - r1) < 1e-9:
        return []
    
    # 計算公式中的 'a' 值
    a = (r0**2 - r1**2 + d**2) / (2 * d)
    
    # 計算 'h_squared'
    h_squared = r0**2 - a**2
    
    # 處理浮點數精度問題：如果 h_squared 略微負數，則視為 0（切線情況）
    if h_squared < -1e-9: # 如果是明顯的負數，表示計算有問題，無交點
        return []
    elif h_squared < 0: # 輕微負數，視為 0 (兩個圓相切，只有一個交點)
        h = 0.0
    else:
        h = math.sqrt(h_squared) # 兩個圓相交，有兩個交點
    
    # 計算交點在 P0P1 線上的投影點 P2
    vec = (P1 - P0) / d # 從 P0 到 P1 的單位向量
    P2 = P0 + a * vec  # 交點在 P0P1 線上的投影點
    
    # 計算垂直於 P0P1 線的偏移向量
    offset_vec = h * np.array([-vec[1], vec[0]]) # 逆時針旋轉 90 度得到垂直向量
    
    if h == 0: # 切線情況，只有一個交點
        return [P2]
    return [P2 + offset_vec, P2 - offset_vec] # 兩個交點

# --- 正運動學 ---
def forward_kinematics(theta1_rad, theta2_rad):
    """
    根據給定的關節角度 (theta1, theta2) 計算末端執行器 C 點的座標。
    """
    # 計算 B 點的座標 (A 為圓心，L1 為半徑)
    # 這裡的 offset 應用於 theta1 加上 offset 後，sin 值的負號表示 Y 軸順時針減小
    B = A + L1 * np.array([math.cos(theta1_rad + offset), -math.sin(theta1_rad + offset)])
    
    # 計算 D 點的座標 (E 為圓心，L4 為半徑)
    # 這裡的 offset 應用於 theta2 加上 offset 後，sin 值的負號表示 Y 軸減小，cos 值的負號表示 X 軸反向
    D = E - L4 * np.array([math.cos(theta2_rad + offset), math.sin(theta2_rad + offset)])
    
    # C 點是 B 為圓心 L2 半徑 和 D 為圓心 L3 半徑 的交點
    C_solutions = circle_circle_intersection(B, L2, D, L3)
    
    # 對於固定的 (theta1, theta2)，C 點應該是唯一的。
    # 如果 circle_circle_intersection 返回多個點，這是由於幾何模型的對稱性。
    # 為了保持一致性，我們選擇 Y 座標較小（通常是「肘下」姿態）的那一個解。
    return min(C_solutions, key=lambda p: p[1]) if C_solutions else None

# --- 逆運動學 (返回所有可能的解) ---
def inverse_kinematics(C):
    """
    根據給定的末端執行器 C 點座標，計算所有可能的關節角度 (theta1, theta2) 組合。
    """
    C = np.array(C)
    all_solutions = []

    # 找出所有可能的 B 點：A 為圓心，L1 為半徑；C 為圓心，L2 為半徑
    B_candidates = circle_circle_intersection(A, L1, C, L2)
    if not B_candidates:
        return []

    # 找出所有可能的 D 點：E 為圓心，L4 為半徑；C 為圓心，L3 為半徑
    D_candidates = circle_circle_intersection(E, L4, C, L3)
    if not D_candidates:
        return []

    # 遍歷所有 B 點和 D 點的組合，計算對應的 (theta1, theta2)
    for B_cand in B_candidates:
        for D_cand in D_candidates:
            # 計算 theta1
            vec_AB = B_cand - A
            theta1_rad_plus_offset = math.atan2(-vec_AB[1], vec_AB[0])
            theta1 = theta1_rad_plus_offset - offset

            # 計算 theta2
            vec_ED = D_cand - E
            theta2_rad_plus_offset = math.atan2(-vec_ED[1], -vec_ED[0])
            theta2 = theta2_rad_plus_offset - offset

            # 將角度規範化到 -180 到 180 度之間
            theta1_deg = math.degrees(theta1)
            theta2_deg = math.degrees(theta2)
            theta1_deg = (theta1_deg + 180) % 360 - 180
            theta2_deg = (theta2_deg + 180) % 360 - 180
            
            all_solutions.append((theta1_deg, theta2_deg))
            
    return all_solutions

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
N_main_points = 12 # 圓周上主要點的數量，這些點會被 IK 計算
circular_points = []
for i in range(N_main_points):
    angle = 2 * math.pi * i / N_main_points
    x = circle_center[0] + circle_radius * math.cos(angle)
    y = circle_center[1] + circle_radius * math.sin(angle)
    circular_points.append(np.array([x, y]))

# 將第一個點加到末尾，以形成閉合循環軌跡，方便插補
# 這樣在路徑結束時會回到起點，形成一個完整的圓
circular_points.append(circular_points[0]) 

# --- 逆運動學計算主要點的所有可能解並進行驗證 ---
print("=== 逆運動學計算所有解並驗證 ===")
# 儲存每個主要點的所有有效 IK 解候選，每個元素是一個列表 (可能包含多個解)
ik_solutions_for_main_points_candidates = [] 
for i, C in enumerate(circular_points):
    solutions = inverse_kinematics(C) # 獲取所有可能的解
    
    if not solutions:
        print(f"[點 {i+1}] 目標 C=({C[0]:.4f}, {C[1]:.4f}) -> IK無解，此點將被跳過。")
        ik_solutions_for_main_points_candidates.append([]) # 儲存空列表表示無解
        continue

    valid_solutions_for_this_point = []
    for sol_idx, (t1_deg, t2_deg) in enumerate(solutions):
        t1_rad = math.radians(t1_deg)
        t2_rad = math.radians(t2_deg)
        
        C_fk = forward_kinematics(t1_rad, t2_rad) # 用 FK 驗證 IK 解
        
        error = float('inf')
        if C_fk is not None:
            dx, dy = C_fk - C
            error = math.hypot(dx, dy)
            # 如果 FK 誤差在容忍度內，則視為有效解
            if error < 1e-4: # 設置一個小的容忍度
                valid_solutions_for_this_point.append((t1_deg, t2_deg))
    
    if not valid_solutions_for_this_point:
        print(f"  [警告] 點 {i+1} 雖然找到解，但所有解的FK驗證誤差過大或不存在，此點將被跳過。")
        ik_solutions_for_main_points_candidates.append([])
    else:
        # 對當前點的所有有效解進行排序，優先選擇「角度絕對值之和」較小的解。
        # 這作為次要選擇標準，當多個解與前一點的增量角度都非常接近時提供偏好。
        valid_solutions_for_this_point.sort(key=lambda sol: abs(sol[0]) + abs(sol[1]))
        ik_solutions_for_main_points_candidates.append(valid_solutions_for_this_point)

# --- 為每個主要點選擇最佳解，以確保最小增量角度路徑 ---
final_main_point_angles = [] # 儲存最終構成平滑路徑的每個主要點的選定角度
prev_t1_for_path = None # 用於計算與前一個點的角度增量
prev_t2_for_path = None

for i in range(len(ik_solutions_for_main_points_candidates)):
    current_point_solutions = ik_solutions_for_main_points_candidates[i]
    
    # 如果當前點沒有任何有效解，則無法繼續軌跡
    if not current_point_solutions:
        print(f"[警告] 點 {i+1} 無有效解，軌跡可能從此中斷。")
        prev_t1_for_path = None # 重置前一個點，等待下一個有效起始
        prev_t2_for_path = None
        continue

    best_t1_deg = None
    best_t2_deg = None
    min_dist = float('inf')

    if prev_t1_for_path is None: # 如果是路徑的第一個有效點 (或前一個點中斷了)
        # 對於路徑的第一個點，選擇其有效解列表中的第一個（因為已經排序過，這是角度較小的那個）
        best_t1_deg, best_t2_deg = current_point_solutions[0]
    else:
        # 從所有候選解中，選擇與前一個點角度距離最小的解 (最小增量角度策略)
        for t1_cand, t2_cand in current_point_solutions:
            # 計算最短角度距離，考慮角度的周期性
            dist_t1 = abs(t1_cand - prev_t1_for_path)
            if dist_t1 > 180: dist_t1 = 360 - dist_t1
            
            dist_t2 = abs(t2_cand - prev_t2_for_path)
            if dist_t2 > 180: dist_t2 = 360 - dist_t2
            
            total_dist = dist_t1 + dist_t2 # 可以使用歐幾里得距離或加權和
            
            if total_dist < min_dist:
                min_dist = total_dist
                best_t1_deg, best_t2_deg = t1_cand, t2_cand
    
    # 如果成功找到了最佳解，則添加到最終路徑中
    if best_t1_deg is not None:
        final_main_point_angles.append((best_t1_deg, best_t2_deg))
        prev_t1_for_path = best_t1_deg
        prev_t2_for_path = best_t2_deg
    else:
        # 理論上如果 current_point_solutions 不為空，這裡總能找到 best_t1_deg。
        # 除非 min_dist 始終是 float('inf')，這不太可能。
        print(f"[錯誤] 點 {i+1} 無法找到與前一點連接的最佳解，軌跡中斷。")
        prev_t1_for_path = None
        prev_t2_for_path = None
        
# 檢查最終選定的主要點是否足夠構成軌跡
if len(final_main_point_angles) < 2:
    print("錯誤: 沒有足夠的有效主要點來生成平滑軌跡。請檢查機構參數或圓形軌跡設定。")
    robot.simulationQuit(1)

# --- 關節空間插補軌跡生成 ---
interpolated_angles = []
SUB_POINTS_PER_SEGMENT = 20 # 每兩個主要點之間插入的子點數量，越大越平滑

# 將路徑的第一個主要點添加到插補列表的起始
if final_main_point_angles:
    interpolated_angles.append(final_main_point_angles[0])

# 在每個選定的主要點之間進行線性插補
for i in range(len(final_main_point_angles) - 1):
    start_t1_deg, start_t2_deg = final_main_point_angles[i]
    end_t1_deg, end_t2_deg = final_main_point_angles[i+1]

    # 處理角度跨越 +/-180 度邊界的問題，確保在插補時走最短路徑
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

    # 生成子點：從 1 開始，避免重複添加起始點（因為已在循環前添加）
    for j in range(1, SUB_POINTS_PER_SEGMENT + 1): 
        t = j / SUB_POINTS_PER_SEGMENT # 插補係數從 1/M 到 1 (到達終點)
        
        # 線性插補關節角度
        interp_t1_deg = start_t1_deg + (end_t1_deg - start_t1_deg) * t
        interp_t2_deg = start_t2_deg + (end_t2_deg - start_t2_deg) * t
        
        # 規範化角度到 -180 到 180 之間，以防插補後超出範圍
        interp_t1_deg = (interp_t1_deg + 180) % 360 - 180
        interp_t2_deg = (interp_t2_deg + 180) % 360 - 180

        interpolated_angles.append((interp_t1_deg, interp_t2_deg))


# --- 在 Webots 世界中繪製預計路徑的綠線 ---
# 使用插補後的所有角度計算對應的 C 點，並繪製成綠線，提供視覺確認
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
        # Z 軸高度可調整，確保線條在機器人上方可見
        proto += f"{point_coord[0]:.6f} {point_coord[1]:.6f} 0.18,\n" 
    proto += "]\n}\ncoordIndex [\n"

    # 連接所有插補點形成平滑線條
    for i in range(len(interpolated_c_points_for_drawing) - 1):
        proto += f"{i}, {i + 1}, -1,\n"
    
    # 判斷是否需要閉合路徑 (如果原始主要點形成了閉合循環)
    if len(final_main_point_angles) > 1 and \
       final_main_point_angles[0] == final_main_point_angles[-1]:
        # 連接最後一個點回到第一個點，形成閉合圓
        proto += f"{len(interpolated_c_points_for_drawing)-1}, 0, -1,\n"
    
    proto += "]\n} }\n"

    robot.getRoot().getField("children").importMFNodeFromString(-1, proto)
else:
    print("無法生成足夠的插補點來繪製圓形路徑。")

# --- 控制流程 ---
print("\n=== 模擬開始 (動態選擇最小增量角度解 + 平滑插補) ===")
current_interp_index = 0 # 當前目標插補點的索引
WAIT_AT_SUBPOINT = 1 # 每個插補子點停留的 timesteps 數量。值越大，運動速度越慢。

# 設定馬達的初始位置到插補路徑的第一個點
if interpolated_angles:
    initial_t1_deg, initial_t2_deg = interpolated_angles[0]
    theta1_motor.setPosition(math.radians(-initial_t1_deg)) # 注意 Webots 馬達方向可能與計算方向相反
    theta2_motor.setPosition(math.radians(initial_t2_deg))  # 注意 Webots 馬達方向可能與計算方向相反
    print(f"[命令] 設定起始點: θ1={initial_t1_deg:.2f}°, θ2={initial_t2_deg:.2f}°")
else:
    print("無可執行軌跡。請檢查上述警告和錯誤訊息。")
    robot.simulationQuit(1) # 如果沒有任何插補點，直接退出模擬


while robot.step(timestep) != -1:
    # 檢查是否完成所有插補點
    if current_interp_index >= len(interpolated_angles):
        print("✔️ 所有插補點已完成")
        break

    # 獲取當前目標插補點的角度
    target_t1_deg, target_t2_deg = interpolated_angles[current_interp_index]

    # 將角度轉換為 Webots 馬達所需的弧度 (已根據 Webots 馬達的旋轉方向進行修正)
    t1_rad_for_motor = math.radians(-target_t1_deg) # 根據您的 sensor1 和 motor1 設定的關係
    t2_rad_for_motor = math.radians(target_t2_deg)  # 根據您的 sensor2 和 motor2 設定的關係

    # 設定馬達的目標位置
    theta1_motor.setPosition(t1_rad_for_motor)
    theta2_motor.setPosition(t2_rad_for_motor)

    # 每隔 WAIT_AT_SUBPOINT 個 timestep，移動到下一個插補點
    # 這是一種簡單的基於時間步的速率控制，可以平滑運動
    if (robot.getTime() * 1000 // timestep) % WAIT_AT_SUBPOINT == 0:
        current_interp_index += 1
        # 您可以在這裡加入打印語句來觀察每個子點的進度，但會產生大量輸出
        # print(f"  [命令] 移至插補點 {current_interp_index}/{len(interpolated_angles)}")