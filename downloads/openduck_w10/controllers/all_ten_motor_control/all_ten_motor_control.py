from controller import Robot  # 從 Webots 的 controller 模組匯入 Robot 類別
import math                    # 匯入 math 模組，用於角度轉弧度等數學運算

# ========================
# 初始化機器人與基本設定
# ========================

# 建立 Robot 實例，這是 Webots 中所有控制器的基礎物件
robot = Robot()

# 取得目前世界的基本時間步長（單位：毫秒），決定每一步模擬的時間間隔
# 例如：若 timestep = 32，代表每 32ms 更新一次控制器
timestep = int(robot.getBasicTimeStep())

# ========================
# 取得馬達與位置感測器
# ========================

# 透過名稱取得名為 't1', 't2', 't3', 't4', 't5' 的馬達裝置（對應機器人模型中的關節）
t1 = robot.getDevice('t1')
t2 = robot.getDevice('t2')
t3 = robot.getDevice('t3')
t4 = robot.getDevice('t4')
t5 = robot.getDevice('t5')

# 透過名稱取得 'rt1', 'rt2', 'rt3', 'rt4', 'rt5' 右邊馬達
rt1 = robot.getDevice('rt1')
rt2 = robot.getDevice('rt2')
rt3 = robot.getDevice('rt3')
rt4 = robot.getDevice('rt4')
rt5 = robot.getDevice('rt5')

# 取得對應的旋轉位置感測器（PositionSensor），用來讀取目前關節角度
t1_sensor = robot.getDevice('t1_sensor')
t2_sensor = robot.getDevice('t2_sensor')
t3_sensor = robot.getDevice('t3_sensor')
t4_sensor = robot.getDevice('t4_sensor')
t5_sensor = robot.getDevice('t5_sensor')

# 取得右邊馬達的感測器
rt1_sensor = robot.getDevice('rt1_sensor')
rt2_sensor = robot.getDevice('rt2_sensor')
rt3_sensor = robot.getDevice('rt3_sensor')
rt4_sensor = robot.getDevice('rt4_sensor')
rt5_sensor = robot.getDevice('rt5_sensor')

# 啟用位置感測器，並設定更新頻率與 timestep 同步
# 這樣每一步 robot.step(timestep) 後，感測器才會更新數值
t1_sensor.enable(timestep)
t2_sensor.enable(timestep)
t3_sensor.enable(timestep)
t4_sensor.enable(timestep)
t5_sensor.enable(timestep)

# 啟用右邊馬達感測器
rt1_sensor.enable(timestep)
rt2_sensor.enable(timestep)
rt3_sensor.enable(timestep)
rt4_sensor.enable(timestep)
rt5_sensor.enable(timestep)

# ========================
# 設定目標角度（度）
# ========================

# 定義左邊五個馬達的目標角度（單位：度）
t1_degree = 0
t2_degree = 0
t3_degree = -15
t4_degree = -5
t5_degree = 20

# 定義右邊五個馬達的目標角度（單位：度）
rt1_degree = 0
rt2_degree = 0
rt3_degree = 15
rt4_degree = 5
rt5_degree = -20

# 將「度」轉換為「弧度」（Webots 內部使用弧度）
t1_target = math.radians(t1_degree)
t2_target = math.radians(t2_degree)
t3_target = math.radians(t3_degree)
t4_target = math.radians(t4_degree)
t5_target = math.radians(t5_degree)

# 轉換右邊馬達的角度到弧度
rt1_target = math.radians(rt1_degree)
rt2_target = math.radians(rt2_degree)
rt3_target = math.radians(rt3_degree)
rt4_target = math.radians(rt4_degree)
rt5_target = math.radians(rt5_degree)

# ========================
# 設定運動時間
# ========================

# 希望五個馬達在 1.0 秒內到達目標位置
target_time = 0.01  # 單位：秒

# ========================
# 讀取初始位置（重要！）
# ========================

# 先執行一次 robot.step(timestep)，讓感測器有機會初始化
# 否則第一次 getValue() 可能會回傳 NaN（無效值）
robot.step(timestep)

# 讀取左邊五個馬達的「當前位置」（弧度）
t1_initial = t1_sensor.getValue()
t2_initial = t2_sensor.getValue()
t3_initial = t3_sensor.getValue()
t4_initial = t4_sensor.getValue()
t5_initial = t5_sensor.getValue()

# 讀取右邊五個馬達的「當前位置」（弧度）
rt1_initial = rt1_sensor.getValue()
rt2_initial = rt2_sensor.getValue()
rt3_initial = rt3_sensor.getValue()
rt4_initial = rt4_sensor.getValue()
rt5_initial = rt5_sensor.getValue()

# ========================
# 計算所需平均速度（弧度/秒）
# ========================

# 計算每個馬達需要移動的「角度差」（取絕對值）
# 再除以目標時間，得到「平均速度」（rad/s）
# 若已經在目標位置，速度設為 0
t1_vel = abs(t1_target - t1_initial) / target_time if t1_initial != t1_target else 0.0
t2_vel = abs(t2_target - t2_initial) / target_time if t2_initial != t2_target else 0.0
t3_vel = abs(t3_target - t3_initial) / target_time if t3_initial != t3_target else 0.0
t4_vel = abs(t4_target - t4_initial) / target_time if t4_initial != t4_target else 0.0
t5_vel = abs(t5_target - t5_initial) / target_time if t5_initial != t5_target else 0.0

# 右邊馬達的平均速度
rt1_vel = abs(rt1_target - rt1_initial) / target_time if rt1_initial != rt1_target else 0.0
rt2_vel = abs(rt2_target - rt2_initial) / target_time if rt2_initial != rt2_target else 0.0
rt3_vel = abs(rt3_target - rt3_initial) / target_time if rt3_initial != rt3_target else 0.0
rt4_vel = abs(rt4_target - rt4_initial) / target_time if rt4_initial != rt4_target else 0.0
rt5_vel = abs(rt5_target - rt5_initial) / target_time if rt5_initial != rt5_target else 0.0

# ========================
# 加入安全裕度（避免來不及到達）
# ========================

# 實際設定最大速度時，乘上 1.1 倍裕度
# 確保即使有誤差或阻力，也能在 1 秒內到達
margin = 1.1
t1_max_vel = t1_vel * margin
t2_max_vel = t2_vel * margin
t3_max_vel = t3_vel * margin
t4_max_vel = t4_vel * margin
t5_max_vel = t5_vel * margin

# 右邊馬達的最大速度
rt1_max_vel = rt1_vel * margin
rt2_max_vel = rt2_vel * margin
rt3_max_vel = rt3_vel * margin
rt4_max_vel = rt4_vel * margin
rt5_max_vel = rt5_vel * margin

# ========================
# 設定馬達目標位置與最大速度
# ========================

# 設定左邊五個馬達的「目標位置」與「最大速度」
t1.setPosition(t1_target)
t2.setPosition(t2_target)
t3.setPosition(t3_target)
t4.setPosition(t4_target)
t5.setPosition(t5_target)

# 設定右邊五個馬達的「目標位置」與「最大速度」
rt1.setPosition(rt1_target)
rt2.setPosition(rt2_target)
rt3.setPosition(rt3_target)
rt4.setPosition(rt4_target)
rt5.setPosition(rt5_target)

# 設定左邊五個馬達的「最大允許速度」
t1.setVelocity(t1_max_vel)
t2.setVelocity(t2_max_vel)
t3.setVelocity(t3_max_vel)
t4.setVelocity(t4_max_vel)
t5.setVelocity(t5_max_vel)

# 設定右邊五個馬達的「最大允許速度」
rt1.setVelocity(rt1_max_vel)
rt2.setVelocity(rt2_max_vel)
rt3.setVelocity(rt3_max_vel)
rt4.setVelocity(rt4_max_vel)
rt5.setVelocity(rt5_max_vel)

# ========================
# 輸出除錯資訊（可選）
# ========================

print(f"移動目標：t1={t1_degree}°, t2={t2_degree}°, t3={t3_degree}°, t4={t4_degree}°, t5={t5_degree}°")
print(f"移動目標：rt1={rt1_degree}°, rt2={rt2_degree}°, rt3={rt3_degree}°, rt4={rt4_degree}°, rt5={rt5_degree}°")
print(f"最大速度限制：t1={t1_max_vel:.3f}, t2={t2_max_vel:.3f}, t3={t3_max_vel:.3f}, t4={t4_max_vel:.3f}, t5={t5_max_vel:.3f}")
print(f"最大速度限制：rt1={rt1_max_vel:.3f}, rt2={rt2_max_vel:.3f}, rt3={rt3_max_vel:.3f}, rt4={rt4_max_vel:.3f}, rt5={rt5_max_vel:.3f}")

# ========================
# 設定到達容許誤差
# ========================

# 定義「視為到達」的誤差範圍（單位：弧度）
# 0.01 rad ≈ 0.57°，足夠精準又避免震盪
tolerance = 0.01

# ========================
# 主控制迴圈
# ========================

# 持續執行模擬，直到手動停止或達到條件
while robot.step(timestep) != -1:
    # 讀取當前五個左邊馬達的實際位置
    current_t1 = t1_sensor.getValue()
    current_t2 = t2_sensor.getValue()
    current_t3 = t3_sensor.getValue()
    current_t4 = t4_sensor.getValue()
    current_t5 = t5_sensor.getValue()

    # 讀取當前五個右邊馬達的實際位置
    current_rt1 = rt1_sensor.getValue()
    current_rt2 = rt2_sensor.getValue()
    current_rt3 = rt3_sensor.getValue()
    current_rt4 = rt4_sensor.getValue()
    current_rt5 = rt5_sensor.getValue()

    # 檢查是否「所有」馬達都接近目標位置（誤差 < 0.01 rad）
    if (abs(current_t1 - t1_target) < tolerance and
        abs(current_t2 - t2_target) < tolerance and
        abs(current_t3 - t3_target) < tolerance and
        abs(current_t4 - t4_target) < tolerance and
        abs(current_t5 - t5_target) < tolerance and
        abs(current_rt1 - rt1_target) < tolerance and
        abs(current_rt2 - rt2_target) < tolerance and
        abs(current_rt3 - rt3_target) < tolerance and
        abs(current_rt4 - rt4_target) < tolerance and
        abs(current_rt5 - rt5_target) < tolerance):
        
        # 所有馬達都到達目標 → 停止運動
        t1.setVelocity(0.0)
        t2.setVelocity(0.0)
        t3.setVelocity(0.0)
        t4.setVelocity(0.0)
        t5.setVelocity(0.0)

        rt1.setVelocity(0.0)
        rt2.setVelocity(0.0)
        rt3.setVelocity(0.0)
        rt4.setVelocity(0.0)
        rt5.setVelocity(0.0)

        print("所有目標位置已到達！")
        break  # 跳出迴圈，結束控制器

    # （可選）進階功能：接近目標時動態減速
    # 但 Webots 的 PID 控制器在限制速度後已能平滑停止，此處不需額外處理

