from controller import Supervisor
import math
import numpy as np

'''
這個程式的 forward_kinematics() 的方程式與 solvespace 及 Webots fivebar
連桿的各項設定吻合, 從 position sensor 讀取的角度值則以逆時針方向為正
因此位於 theta1 轉角讀取的 position sensor 值必須取負值才能與 forward_kinematics 
算出來的 C 點位置吻合
接下來則必須處理 inverse_kinematics() 函式, 以便與場景中的參數配合運算
'''
# --- 機構參數 ---
L1 = L4 = 16.82/100  
L2 = L3 = 27.5/100    
A = np.array([6.25/100, 15.0/100]) 
E = np.array([-6.25/100, 15.0/100])

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

    # 根據點 C 反推 B 點與 D 點到 A 和 E 的向量
    vec_AC = C - A
    vec_EC = C - E

    # 計算與 L1 連桿相連的角度 theta1
    # 注意：原始定義中，θ1 是從 A 出發、向 B 點轉，方向為逆時針為正，
    # 但在 forward_kinematics 中 theta1 是與 offset 相加後送入 cos/sin。
    # 因此這裡計算出來的角度也應減去 offset
    theta1 = math.atan2(-vec_AC[1], vec_AC[0]) - offset
    theta2 = math.atan2(-vec_EC[1], -vec_EC[0]) - offset  # 注意 D 點在 E 點左側

    return theta1, theta2
    
# --- 初始化 Supervisor ---
robot = Supervisor()
timestep = 32

# --- 裝置連接 ---
theta1_motor = robot.getDevice("theta1") 
theta2_motor = robot.getDevice("theta2")
sensor1 = robot.getDevice("sensor1")
sensor2 = robot.getDevice("sensor2")

sensor1.enable(timestep)
sensor2.enable(timestep)

theta1_motor.setPosition(float('inf'))
theta2_motor.setPosition(float('inf'))
theta1_motor.setVelocity(5.0)
theta2_motor.setVelocity(5.0)

# --- 預定角度 ---
target_angles_deg = [
    (15, 45),
    (30, 60),
    (60, 30),
    (45, 15)
]

# --- 預計 C 點繪圖資料 ---
target_c_points_for_drawing = []
print("--- 預計目標角度與 C 點座標 ---")
for i, (t1_deg, t2_deg) in enumerate(target_angles_deg):
    t1_rad = math.radians(t1_deg)
    t2_rad = math.radians(t2_deg)
    C = forward_kinematics(t1_rad, t2_rad)
    if C is not None:
        target_c_points_for_drawing.append(C)
        print(f"預計點 {i+1}: t1={t1_deg}°, t2={t2_deg}° -> C=({C[0]:.4f}, {C[1]:.4f})")
    else:
        print(f"警告：無法計算點 {i+1} 的 C 點")
print("---------------------------------------")

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
    for point in target_c_points_for_drawing:
        proto += f"{point[0]:.6f} {point[1]:.6f} 0.18,\n"
    proto += "]\n}\ncoordIndex [\n"
    for i in range(len(target_c_points_for_drawing)):
        proto += f"{i}, {(i + 1) % len(target_c_points_for_drawing)}, -1,\n"
    proto += "]\n} }\n"
    robot.getRoot().getField("children").importMFNodeFromString(-1, proto)

# --- 運動控制 ---
print("\n--- 機械臂運動控制與實時資料列印 ---")
current_index = 0
pause_counter = 0
target_set = True
POINTS_PAUSE_DURATION = 30

while robot.step(timestep) != -1:
    if current_index >= len(target_angles_deg):
        print("所有目標角度已完成。程式結束。")
        break

    # 取得感測器角度(實際從起始位置的相對轉角)
    theta1_actual_rad = -sensor1.getValue()
    theta2_actual_rad = sensor2.getValue()
    C_actual = forward_kinematics(theta1_actual_rad, theta2_actual_rad)

    if C_actual is not None:
        print(f"[實際] C 點位置: ({C_actual[0]:.4f}, {C_actual[1]:.4f})")
        print("theta1, theta2 感測角度:", math.degrees(theta1_actual_rad), math.degrees(theta2_actual_rad))
    else:
        print("[實際] 無法計算當前角度的 C 點")

    if target_set and pause_counter == 0:
        t1_deg, t2_deg = target_angles_deg[current_index]
        t1_rad_motor = math.radians(-t1_deg)
        t2_rad_motor = math.radians(t2_deg)

        C_target = forward_kinematics(math.radians(t1_deg), math.radians(t2_deg))

        theta1_motor.setPosition(t1_rad_motor)
        theta2_motor.setPosition(t2_rad_motor)

        print(f"\n--- Timestep {robot.getTime():.2f}s | 點 {current_index+1}/{len(target_angles_deg)} ---")
        print(f"  目標角度: t1={t1_deg}°, t2={t2_deg}°")
        print(f"  發送至馬達: t1={-t1_deg}°, t2={t2_deg}°")
        if C_target is not None:
            print(f"  [預期] C 點位置: ({C_target[0]:.4f}, {C_target[1]:.4f})")
        else:
            print("  [預期] 無法求得 C 點")

        target_set = False
        pause_counter = POINTS_PAUSE_DURATION
        current_index += 1

    elif pause_counter > 0:
        pause_counter -= 1
    else:
        target_set = True