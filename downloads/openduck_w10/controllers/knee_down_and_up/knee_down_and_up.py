from controller import Robot, Keyboard
import math

# ========================
# 初始化機器人與基本設定
# ========================

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 初始化鍵盤
keyboard = Keyboard()
keyboard.enable(timestep)

# ========================
# 取得馬達與位置感測器
# ========================

# 左腿馬達
t1 = robot.getDevice('t1')
t2 = robot.getDevice('t2')
t3 = robot.getDevice('t3')
t4 = robot.getDevice('t4')
t5 = robot.getDevice('t5')

# 右腿馬達
rt1 = robot.getDevice('rt1')
rt2 = robot.getDevice('rt2')
rt3 = robot.getDevice('rt3')
rt4 = robot.getDevice('rt4')
rt5 = robot.getDevice('rt5')

# 位置感測器
t1_sensor = robot.getDevice('t1_sensor')
t2_sensor = robot.getDevice('t2_sensor')
t3_sensor = robot.getDevice('t3_sensor')
t4_sensor = robot.getDevice('t4_sensor')
t5_sensor = robot.getDevice('t5_sensor')

rt1_sensor = robot.getDevice('rt1_sensor')
rt2_sensor = robot.getDevice('rt2_sensor')
rt3_sensor = robot.getDevice('rt3_sensor')
rt4_sensor = robot.getDevice('rt4_sensor')
rt5_sensor = robot.getDevice('rt5_sensor')

# 啟用感測器
for sensor in [t1_sensor, t2_sensor, t3_sensor, t4_sensor, t5_sensor,
               rt1_sensor, rt2_sensor, rt3_sensor, rt4_sensor, rt5_sensor]:
    sensor.enable(timestep)

# 馬達列表（方便操作）
left_motors = [t1, t2, t3, t4, t5]
right_motors = [rt1, rt2, rt3, rt4, rt5]
left_sensors = [t1_sensor, t2_sensor, t3_sensor, t4_sensor, t5_sensor]
right_sensors = [rt1_sensor, rt2_sensor, rt3_sensor, rt4_sensor, rt5_sensor]

# ========================
# 定義兩種姿態（度）
# ========================

# 站直姿態
STAND_DEGREES = [0, 0, 0, 0, 0]  # t1~t5, rt1~rt5 都一樣

# 蹲下姿態（你提供的）
SQUAT_DEGREES = [0, 0, 55, -110, 55]

# 目前目標姿態
current_target_deg = STAND_DEGREES.copy()

# 轉換為弧度目標
def degrees_to_targets(deg_list):
    return [math.radians(d) for d in deg_list]

# 初始目標（站直）
target_positions = degrees_to_targets(STAND_DEGREES * 2)  # 左右各五個
initial_positions = [0.0] * 10

# 運動時間
MOVE_TIME = 1.0  # 秒
MARGIN = 1.1     # 速度裕度
TOLERANCE = 0.01 # 弧度

# ========================
# 設定馬達到目標（帶速度限制）
# ========================

def set_targets(left_deg, right_deg, move_time=MOVE_TIME):
    global target_positions, initial_positions

    # 目標角度（度 → 弧度）
    left_targets = [math.radians(d) for d in left_deg]
    right_targets = [math.radians(d) for d in right_deg]
    target_positions = left_targets + right_targets

    # 讀取當前位置
    current = [s.getValue() for s in left_sensors + right_sensors]
    initial_positions = current

    # 計算所需速度
    velocities = []
    for i in range(10):
        delta = abs(target_positions[i] - current[i])
        vel = delta / move_time if delta > 1e-6 else 0.0
        velocities.append(vel * MARGIN)

    # 設定馬達
    for i, motor in enumerate(left_motors + right_motors):
        motor.setPosition(target_positions[i])
        motor.setVelocity(velocities[i])

    print(f"開始移動到: 左腿 {left_deg}, 右腿 {right_deg}")

# 初始設定為站直
robot.step(timestep)  # 讓感測器初始化
set_targets(STAND_DEGREES, STAND_DEGREES)

# ========================
# 主控制迴圈
# ========================

print("按 's' 站直, 按 'k' 蹲下")

last_key = -1
moving = True

while robot.step(timestep) != -1:
    key = keyboard.getKey()

    # 按鍵觸發（避免重複觸發）
    if key != -1 and key != last_key:
        if key == ord('S') or key == ord('s'):
            print("=== 站直 ===")
            set_targets(STAND_DEGREES, STAND_DEGREES)
            moving = True
        elif key == ord('K') or key == ord('k'):
            print("=== 蹲下 ===")
            set_targets(SQUAT_DEGREES, SQUAT_DEGREES)
            moving = True

    last_key = key

    # 檢查是否所有馬達到達目標
    if moving:
        current = [s.getValue() for s in left_sensors + right_sensors]
        arrived = all(abs(current[i] - target_positions[i]) < TOLERANCE for i in range(10))

        if arrived:
            # 停止所有馬達
            for motor in left_motors + right_motors:
                motor.setVelocity(0.0)
            moving = False
            print("姿態切換完成！")