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

# 透過名稱取得名為 't1', 't2', 't3' 的馬達裝置（對應機器人模型中的關節）
t1 = robot.getDevice('t1')
t2 = robot.getDevice('t2')
t3 = robot.getDevice('t3')

# 取得對應的旋轉位置感測器（PositionSensor），用來讀取目前關節角度
t1_sensor = robot.getDevice('t1_sensor')
t2_sensor = robot.getDevice('t2_sensor')
t3_sensor = robot.getDevice('t3_sensor')

# 啟用位置感測器，並設定更新頻率與 timestep 同步
# 這樣每一步 robot.step(timestep) 後，感測器才會更新數值
t1_sensor.enable(timestep)
t2_sensor.enable(timestep)
t3_sensor.enable(timestep)

# ========================
# 設定目標角度（度）
# ========================

# 定義三個馬達的目標角度（單位：度）
t1_degree = 20   # t1 要轉到 20 度
t2_degree = 20   # t2 要轉到 20 度
t3_degree = 60   # t3 要轉到 60 度（您改為 60°）

# 將「度」轉換為「弧度」（Webots 內部使用弧度）
# 轉換公式：radians = degrees × π / 180
t1_target = math.radians(t1_degree)
t2_target = math.radians(t2_degree)
t3_target = math.radians(t3_degree)

# ========================
# 設定運動時間
# ========================

# 希望三個馬達在 1.0 秒內到達目標位置
target_time = 1.0  # 單位：秒

# ========================
# 讀取初始位置（重要！）
# ========================

# 先執行一次 robot.step(timestep)，讓感測器有機會初始化
# 否則第一次 getValue() 可能會回傳 NaN（無效值）
robot.step(timestep)

# 讀取三個馬達的「當前位置」（弧度）
t1_initial = t1_sensor.getValue()
t2_initial = t2_sensor.getValue()
t3_initial = t3_sensor.getValue()

# ========================
# 計算所需平均速度（弧度/秒）
# ========================

# 計算每個馬達需要移動的「角度差」（取絕對值）
# 再除以目標時間，得到「平均速度」（rad/s）
# 若已經在目標位置，速度設為 0
t1_vel = abs(t1_target - t1_initial) / target_time if t1_initial != t1_target else 0.0
t2_vel = abs(t2_target - t2_initial) / target_time if t2_initial != t2_target else 0.0
t3_vel = abs(t3_target - t3_initial) / target_time if t3_initial != t3_target else 0.0

# ========================
# 加入安全裕度（避免來不及到達）
# ========================

# 實際設定最大速度時，乘上 1.1 倍裕度
# 確保即使有誤差或阻力，也能在 1 秒內到達
margin = 1.1
t1_max_vel = t1_vel * margin
t2_max_vel = t2_vel * margin
t3_max_vel = t3_vel * margin

# ========================
# 設定馬達目標位置與最大速度
# ========================

# 設定馬達的「目標位置」（啟用內建 PID 位置控制器）
t1.setPosition(t1_target)
t2.setPosition(t2_target)
t3.setPosition(t3_target)

# 設定馬達的「最大允許速度」（限制運動速度，避免太快）
# Webots 會自動以不超過此速度的方式，平滑移動到目標
t1.setVelocity(t1_max_vel)
t2.setVelocity(t2_max_vel)
t3.setVelocity(t3_max_vel)

# ========================
# 輸出除錯資訊（可選）
# ========================

print(f"移動目標：t1={t1_degree}°, t2={t2_degree}°, t3={t3_degree}°，預計 {target_time} 秒完成")
print(f"最大速度限制：t1={t1_max_vel:.3f}, t2={t2_max_vel:.3f}, t3={t3_max_vel:.3f} rad/s")

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
    # 讀取當前三個馬達的實際位置
    current_t1 = t1_sensor.getValue()
    current_t2 = t2_sensor.getValue()
    current_t3 = t3_sensor.getValue()

    # 檢查是否「全部」馬達都接近目標位置（誤差 < 0.01 rad）
    if (abs(current_t1 - t1_target) < tolerance and
        abs(current_t2 - t2_target) < tolerance and
        abs(current_t3 - t3_target) < tolerance):
        
        # 三個馬達都到達目標 → 停止運動
        t1.setVelocity(0.0)
        t2.setVelocity(0.0)
        t3.setVelocity(0.0)
        
        print("所有目標位置已到達！")
        break  # 跳出迴圈，結束控制器

    # （可選）進階功能：接近目標時動態減速
    # 但 Webots 的 PID 控制器在限制速度後已能平滑停止，此處不需額外處理