from controller import Robot
import math

class OttoNinjaController:
    TIME_STEP = 32  # Webots simulation time step in ms

    def __init__(self):
        self.robot = Robot()

        # 安全取得裝置（有錯就直接報）
        self.right_motory = self.get_required_device('right_motory')
        self.left_motory = self.get_required_device('left_motory')
        self.right_motorz = self.get_required_device('right_motorz')
        self.left_motorz = self.get_required_device('left_motorz')

        # motory 角度控制，設定初始值
        self.right_motory.setVelocity(0.5)  # 降低速度避免模擬崩潰
        self.left_motory.setVelocity(0.5)
        self.right_motory.setPosition(0)
        self.left_motory.setPosition(0)

        # motorz 使用速度控制模式
        self.right_motorz.setPosition(float('inf'))
        self.left_motorz.setPosition(float('inf'))
        self.right_motorz.setVelocity(0)
        self.left_motorz.setVelocity(0)

    def get_required_device(self, name):
        device = self.robot.getDevice(name)
        if device is None:
            raise RuntimeError(f'Device "{name}" was not found on robot.')
        return device

    def set_leg_orientation(self, right_angle_deg=-80, left_angle_deg=80):
        right_rad = math.radians(right_angle_deg)
        left_rad = math.radians(left_angle_deg)
        self.right_motory.setPosition(right_rad)
        self.left_motory.setPosition(left_rad)
        print(f"[Info] Setting leg orientation: right = {right_angle_deg}°, left = {left_angle_deg}°")

    def step_forward(self, duration=1.0, speed=2.0):
        print("[Action] Stepping forward...")
        self.right_motorz.setVelocity(speed)
        self.left_motorz.setVelocity(speed)
        self._run_for_duration(duration)
        self.stop()

    def turn_left(self, duration=1.0, speed=2.0):
        print("[Action] Turning left...")
        self.right_motorz.setVelocity(speed)
        self.left_motorz.setVelocity(-speed)
        self._run_for_duration(duration)
        self.stop()

    def stop(self):
        self.right_motorz.setVelocity(0)
        self.left_motorz.setVelocity(0)
        print("[Info] Motors stopped.")

    def _run_for_duration(self, duration_sec):
        steps = int((duration_sec * 1000) / self.TIME_STEP)
        for _ in range(steps):
            self.robot.step(self.TIME_STEP)

    def run(self):
        print("[Start] Controller running...")

        # 設定腳部關節初始角度
        self.set_leg_orientation()

        # 等待穩定
        for _ in range(30):  # 多等幾個 time step
            self.robot.step(self.TIME_STEP)

        # 前進一段
        self.step_forward(duration=3.0)
        '''
        # 左轉一段
        self.turn_left(duration=1.0)

        # 再前進
        self.step_forward(duration=1.5)
        '''
        print("[Done] 動作完成。")


if __name__ == "__main__":
    controller = OttoNinjaController()
    controller.run()
