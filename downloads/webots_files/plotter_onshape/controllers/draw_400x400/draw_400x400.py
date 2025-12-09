"""
Webots Controller: 繪製 0.3m x 0.25m 矩形 (使用 L=321mm 修正參數和角度偏移)
"""

from controller import Robot, Motor
import cmath
import numpy as np

# ----------------------------------------------------------------------
# 0. 座標系參數設定與角度偏移
# ----------------------------------------------------------------------

# 使用成功運行所需的經驗修正值
L = 321.0  # 修正後的連桿長度 (mm)
B = 400.0  # 馬達間距 (mm)

# 角度偏移補償 (確保與 Webots 模型的零點匹配)
PHI_START_L = np.arctan2(0.316246, -0.048872) # 1.724 弧度
PHI_START_R = np.arctan2(0.316246, 0.048872) # 1.417 弧度

# ----------------------------------------------------------------------
# 1. 逆運動學函式 (Solution 4: L=321 mm, B=400 mm)
# ----------------------------------------------------------------------

L_TERM = L * 2 
L2_TERM = L * L * 2 

def t1_sol4(x, y):
    """計算 t1 (left_motor) 的目標絕對角度 (弧度) - 使用 L=321mm。"""
    return 2 * cmath.atan((L * y + cmath.sqrt(-x**4 - 2 * x**2 * y**2 + L2_TERM * x**2 - y**4 + L2_TERM * y**2)) / (x**2 + L_TERM * x + y**2))

def t2_sol4(x, y):
    """計算 t2 (right_motor) 的目標絕對角度 (弧度) - 仍使用 L=320mm 常數項 (穩定性考量)。"""
    return 2 * cmath.atan((320.0 * y + cmath.sqrt(-x**4 + 800.0 * x**3 - 2 * x**2 * y**2 - 137600.0 * x**2 + 800.0 * x * y**2 - 8960000.0 * x - y**4 + 224000.0 * y**2 - 256000000.0)) / (x**2 - 800.0 * x + y**2 + 256000.0))

# --- 2. 路徑生成器 (矩形) ---

def generate_rectangle_path_m(x_size=0.3, y_size=0.25, start_x=0.05, start_y=0.05, steps_per_side=80):
    """
    生成繪製一個矩形所需的點。
    :param x_size: 矩形寬度 (m)
    :param y_size: 矩形高度 (m)
    :param start_x: 起點 X 座標 (m)
    :param start_y: 起點 Y 座標 (m)
    """
    path = []
    
    # 矩形四個頂點座標
    p1 = (start_x, start_y)                      # (0.05, 0.05) 左下
    p2 = (start_x + x_size, start_y)             # (0.35, 0.05) 右下
    p3 = (start_x + x_size, start_y + y_size)    # (0.35, 0.30) 右上
    p4 = (start_x, start_y + y_size)             # (0.05, 0.30) 左上

    points = [p1, p2, p3, p4, p1] # 閉合路徑

    print(f"Drawing rectangle with size {x_size}m x {y_size}m. Max Y={p3[1]:.2f}m.")

    # 繪製四條邊 (P1->P2, P2->P3, P3->P4, P4->P1)
    for i in range(4):
        start_point = points[i]
        end_point = points[i+1]
        
        x_segment = np.linspace(start_point[0], end_point[0], steps_per_side)
        y_segment = np.linspace(start_point[1], end_point[1], steps_per_side)
        
        # 將線段點加入路徑 (跳過線段的起點，避免重複)
        for j in range(1, steps_per_side):
            path.append((x_segment[j], y_segment[j]))

    return path

# --- 3. Webots 控制器初始化 ---

robot = Robot()
timestep = int(robot.getBasicTimeStep())
left_motor = robot.getDevice('left_motor')
right_motor = robot.getDevice('right_motor') 

left_motor.setPosition(0.0)
left_motor.setVelocity(5.0) 
right_motor.setPosition(0.0)
right_motor.setVelocity(5.0)

path_points_ik_m = generate_rectangle_path_m()
path_index = 0
total_points = len(path_points_ik_m)

# ----------------------------------------------------------------------
# 4. 主迴圈
# ----------------------------------------------------------------------

print(f"Controller started. Total points: {total_points}")

while robot.step(timestep) != -1:
    
    if path_index >= total_points:
        print("INFO: Rectangle drawing completed successfully.")
        break
        
    x_target_ik_m, y_target_ik_m = path_points_ik_m[path_index]
    x_target_mm = x_target_ik_m * 1000
    y_target_mm = y_target_ik_m * 1000
    
    try:
        # 1. 計算 IK 絕對角度
        t1_ik_complex = t1_sol4(x_target_mm, y_target_mm)
        t2_ik_complex = t2_sol4(x_target_mm, y_target_mm) 
        
        if t1_ik_complex.imag != 0 or t2_ik_complex.imag != 0:
            print(f"Error: Point ({x_target_ik_m:.3f}, {y_target_ik_m:.3f}) m is unreachable (Complex solution). Stopping.")
            break
            
        # 2. 應用角度偏移
        t1_motor = t1_ik_complex.real - PHI_START_L
        t2_motor = t2_ik_complex.real - PHI_START_R
        
        # 3. 設置馬達位置
        left_motor.setPosition(t1_motor)
        right_motor.setPosition(t2_motor)
        
        path_index += 1

    except Exception as e:
        print(f"Critical Error processing point ({x_target_ik_m:.3f}, {y_target_ik_m:.3f}) m: {e}. Stopping.")
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        break