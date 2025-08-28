from controller import Supervisor
import math
import numpy as np

# --- 機構參數 ---
L1 = L4 = 0.1682 
L2 = L3 = 0.275
A = np.array([0.0625, 0.15]) 
E = np.array([-0.0625, 0.15])

# --- 圓軌跡參數 (已修改，使其在機構可達範圍內) ---
# 現在這些座標都應在 Webots 的「米」單位下，與上方機構參數一致。
circle_center_x = 0.0   # X 軸中心點
circle_center_y = -0.13  # Y 軸中心點，調整到機械臂可達範圍
circle_radius = 0.1    # 圓的半徑
N_points_on_circle = 4 # 圓上的點數，越多越平滑

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

# --- 正運動學 --- (正確版本)
def forward_kinematics(theta1_rad, theta2_rad):
    offset = math.radians(41.9872)
    Bx = A[0] + L1 * math.cos(theta1_rad+offset)
    By = A[1] - L1 * math.sin(theta1_rad+offset)
    B_coord = np.array([Bx, By])
    Dx = E[0] - L4 * math.cos(theta2_rad+offset)
    Dy = E[1] - L4 * math.sin(theta2_rad+offset)
    D_coord = np.array([Dx, Dy])
    C_solutions = circle_circle_intersection(B_coord, L2, D_coord, L3)
    if C_solutions:
        if len(C_solutions) == 1:
            return C_solutions[0]
        else:
            return C_solutions[0] if C_solutions[0][1] < C_solutions[1][1] else C_solutions[1]
    return None

def inverse_kinematics(C):
    offset = math.radians(41.9872)

    # 計算向量
    vec_AC = C - A
    vec_EC = C - E

    # θ1 是從 A 到 B，順時針為正 → 所以取負的 atan2 結果
    theta1 = -math.atan2(-vec_AC[1], vec_AC[0]) - offset

    # θ2 是從 E 到 D，逆時針為正 → 直接使用 atan2 結果
    theta2 = math.atan2(-vec_EC[1], -vec_EC[0]) - offset

    return math.degrees(theta1), math.degrees(theta2)
    
# --- 初始化 Supervisor ---
robot = Supervisor()
timestep = 32 

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
theta1_motor.setVelocity(10.0) 
theta2_motor.setVelocity(10.0) 

# --- 生成圓軌跡點 ---
circular_points = []
for i in range(N_points_on_circle + 1):
    angle = 2 * math.pi * i / N_points_on_circle
    x = circle_center_x + circle_radius * math.cos(angle)
    y = circle_center_y + circle_radius * math.sin(angle)
    circular_points.append((x, y))

# --- 預計算圓軌跡的 IK 角度和 C 點 ---
# 這些是我們希望機械臂通過的點，將會繪製成綠色線條
circular_c_points_for_drawing = []
print("--- 預計算圓軌跡的 IK 角度和 C 點 ---")
circular_target_angles_deg = []

for i, (target_Cx, target_Cy) in enumerate(circular_points):
    t1_ik_deg, t2_ik_deg = inverse_kinematics((target_Cx, target_Cy))
    
    if t1_ik_deg is not None and t2_ik_deg is not None:
        circular_target_angles_deg.append((t1_ik_deg, t2_ik_deg))
        
        # 使用計算出的 IK 角度進行正運動學，以驗證 C 點是否正確
        C_reconstructed = forward_kinematics(math.radians(t1_ik_deg), math.radians(t2_ik_deg))
        
        if C_reconstructed is not None:
            circular_c_points_for_drawing.append(C_reconstructed)
            print(f"圓點 {i+1}: 目標C=({target_Cx:.4f}, {target_Cy:.4f}) -> IK角度=(t1={t1_ik_deg:.2f}°, t2={t2_ik_deg:.2f}°) -> FK驗證C=({C_reconstructed[0]:.4f}, {C_reconstructed[1]:.4f})")
        else:
            print(f"警告：圓點 {i+1} ({target_Cx:.2f}, {target_Cy:.2f}) - IK 角度 ({t1_ik_deg:.2f}°, {t2_ik_deg:.2f}°) 無法通過 FK 驗證。")
    else:
        print(f"警告：圓點 {i+1} ({target_Cx:.2f}, {target_Cy:.2f}) 不可達，跳過。")
print("---------------------------------------")

# 在 Webots 世界中繪製預計路徑的綠線
if len(circular_c_points_for_drawing) >= 2:
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
    #for point_coord in circular_c_points_for_drawing:
    for point_coord in circular_points:
        # Z 軸高度可調整，確保在機器人上方可見
        proto += f"{point_coord[0]:.6f} {point_coord[1]:.6f} 0.18,\n" 
    proto += "]\n}\ncoordIndex [\n"

    # 連接所有點形成閉合圓 (最後一點連接到第一點)
    for i in range(len(circular_c_points_for_drawing)):
        proto += f"{i}, {(i + 1) % len(circular_c_points_for_drawing)}, -1,\n"
    proto += "]\n} }\n"

    robot.getRoot().getField("children").importMFNodeFromString(-1, proto)
else:
    print("無法生成足夠的點來繪製圓形路徑。")


# --- 控制邏輯 ---
print("\n--- 機械臂圓形路徑運動控制 ---")
current_point_index = 0 
pause_counter = 0       
target_set = True       
POINTS_PAUSE_DURATION = 30 # 每個點停留的 timesteps 數量

# --- 主控制迴圈 ---
while robot.step(timestep) != -1:
    # --- 退出條件 ---
    if current_point_index >= len(circular_target_angles_deg):
        print("所有圓形軌跡點已測試完畢。程式結束。")
        break

    # --- 狀態機核心邏輯 ---
    if target_set and pause_counter == 0:
        # 1. 取得目標角度 (IK 計算出的角度，符合你的物理定義)
        t1_ik_deg, t2_ik_deg = circular_target_angles_deg[current_point_index]

        # 2. 將 IK 角度轉換為 Webots 馬達所需的弧度 (根據我們最終確認的方向調整)
        # t1 馬達：因為 Webots 馬達「正」是逆時針，所以給負值讓它順時針
        t1_rad_for_motor = math.radians(-t1_ik_deg) 
        # t2 馬達：因為 Webots 馬達「正」是逆時針，而 t2_ik_deg 已經是逆時針，所以直接給正值
        t2_rad_for_motor = math.radians(t2_ik_deg) 

        # 3. 設定馬達的目標位置
        theta1_motor.setPosition(t1_rad_for_motor)
        theta2_motor.setPosition(t2_rad_for_motor)

        # 4. 打印設定信息
        print(f"\n--- Timestep: {robot.getTime():.2f}s, 圓點 {current_point_index+1}/{len(circular_target_angles_deg)} ---")
        print(f"  目標 C 點: ({circular_points[current_point_index][0]:.4f}, {circular_points[current_point_index][1]:.4f})")
        print(f"  計算IK角度: t1={t1_ik_deg:.2f}°, t2={t2_ik_deg:.2f}°")
        print(f"  實際發送給Webots馬達: t1_motor={math.degrees(t1_rad_for_motor):.2f}°, t2_motor={math.degrees(t2_rad_for_motor):.2f}°")
        
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