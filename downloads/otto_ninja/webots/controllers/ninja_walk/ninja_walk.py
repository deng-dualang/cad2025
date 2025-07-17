from controller import Robot, Keyboard
import math

class OttoNinjaController:
    TIME_STEP = 32  # 模擬時間步長，Webots 'WorldInfo.basicTimeStep' 建議也設為 32。
    DRIVE_SPEED = 4.0  # 輪子推進/旋轉速度。

    # 讓輪子「立起來」的角度（度）。這個值至關重要，請精確調整！
    # 通常 -90 度會讓腳底垂直於地面。如果機器人不穩，請微調此值。
    STAND_ANGLE_DEG = -90 
    
    # Y 軸馬達旋轉速度，影響站立和恢復動作的速度與平穩性。
    # 增加此值可使動作更快，但若太高會導致不穩和警告。
    STAND_MOTOR_VELOCITY = 0.8 # 從 0.3 增加到 0.8。如果仍然不穩，請降低此值。

    def __init__(self):
        self.robot = Robot()
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.TIME_STEP)

        self.right_motory = self.get_dev('right_motory')
        self.left_motory = self.get_dev('left_motory')
        self.right_motorz = self.get_dev('right_motorz')
        self.left_motorz = self.get_dev('left_motorz')

        # 設定 Y 軸馬達的速度，用於站立或恢復平躺。
        self.right_motory.setVelocity(self.STAND_MOTOR_VELOCITY)
        self.left_motory.setVelocity(self.STAND_MOTOR_VELOCITY)
        
        # 初始狀態：輪子平躺。
        self.set_leg_y_angle(0, 0)
        
        # 設定 Z 軸馬達為無限位置模式 (速度控制)，初始速度為 0。
        for wheel in (self.right_motorz, self.left_motorz):
            wheel.setPosition(float('inf'))
            wheel.setVelocity(0)

        self.is_standing = False 
        print("🤖 Otto-Ninja 控制器啟動。")
        print("按下 'S' 鍵讓機器人站立。")
        print("站立後，使用方向鍵控制：上/下前進後退，左/右轉彎。")
        print("按下 'Space' 鍵停止，按下 'R' 鍵恢復平躺。")

    def get_dev(self, name):
        """獲取 Webots 裝置。如果找不到裝置，則拋出 RuntimeError。"""
        d = self.robot.getDevice(name)
        if d is None:
            raise RuntimeError(f'無法找到裝置: {name}')
        return d

    def set_leg_y_angle(self, right_deg, left_deg):
        """設定左右腿部 Y 軸的角度。角度以度為單位，會轉換為弧度。"""
        self.right_motory.setPosition(math.radians(right_deg))
        self.left_motory.setPosition(math.radians(left_deg))

    def set_z_speeds(self, left_speed, right_speed):
        """設定左右腳 Z 軸馬達的速度。"""
        self.left_motorz.setVelocity(left_speed)
        self.right_motorz.setVelocity(right_speed)

    def run(self):
        """機器人的主控制迴圈。"""
        while self.robot.step(self.TIME_STEP) != -1:
            key = self.keyboard.getKey()

            if not self.is_standing:
                # 機器人尚未站立
                if key == ord('S'):
                    print("🚀 Otto-Ninja 站立中...")
                    # 根據您確認的 Y 軸馬達方向，這樣設置可以讓輪子正確立起來
                    self.set_leg_y_angle(self.STAND_ANGLE_DEG, -self.STAND_ANGLE_DEG)
                    self.is_standing = True
                    
                    # 減少站立後的等待時間，如果機器人站立穩定，可以將其設為0
                    # 如果仍有警告，則需要增加此值或降低 STAND_MOTOR_VELOCITY
                    for _ in range(50): # 從 150 步減少到 50 步 (約 1.6 秒)
                        if self.robot.step(self.TIME_STEP) == -1: return 
                    print("🤖 Otto-Ninja 已站立，可以使用方向鍵控制。")
                else:
                    self.set_z_speeds(0, 0)
            else:
                # 機器人已站立，根據方向鍵控制 Z 軸馬達
                current_left_speed = 0
                current_right_speed = 0

                if key == Keyboard.UP:
                    current_left_speed = self.DRIVE_SPEED
                    current_right_speed = self.DRIVE_SPEED
                elif key == Keyboard.DOWN:
                    current_left_speed = -self.DRIVE_SPEED
                    current_right_speed = -self.DRIVE_SPEED
                elif key == Keyboard.LEFT:
                    current_left_speed = -self.DRIVE_SPEED
                    current_right_speed = self.DRIVE_SPEED
                elif key == Keyboard.RIGHT:
                    current_left_speed = self.DRIVE_SPEED
                    current_right_speed = -self.DRIVE_SPEED
                elif key == ord(' '): # 按下空白鍵停止
                    print("🛑 停止。")
                    current_left_speed = 0
                    current_right_speed = 0
                elif key == ord('R'): # 按下 'R' 鍵恢復初始平躺狀態
                    print("🛋️ 恢復平躺狀態。")
                    self.set_leg_y_angle(0, 0) # 讓 Y 軸馬達回到平躺角度
                    self.set_z_speeds(0, 0)    # Z 軸馬達停止
                    self.is_standing = False
                    
                    # 減少恢復平躺後等待時間，同上
                    for _ in range(50): # 從 100 步減少到 50 步
                        if self.robot.step(self.TIME_STEP) == -1: return
                
                self.set_z_speeds(current_left_speed, current_right_speed)

# 執行控制器
if __name__ == "__main__":
    controller = OttoNinjaController()
    controller.run()