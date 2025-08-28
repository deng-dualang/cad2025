import math
from controller import Robot

def theta1(angle):
    # theta1 轉動角度順時針為正
    return -angle

def theta2(angle):
    # theta2 轉動角度逆時針為正
    return angle
    
def run_robot():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    motor1 = robot.getDevice('theta1')
    motor2 = robot.getDevice('theta2')

    # 設為「位置控制模式」
    motor1_angle_deg = theta1(174.6)  # 角度
    motor2_angle_deg = theta2(5.4)  
    
    motor1_angle_rad = math.radians(motor1_angle_deg)  # 轉為弧度
    motor2_angle_rad = math.radians(motor2_angle_deg)  # 轉為弧度
    

    motor1.setPosition(motor1_angle_rad)
    motor2.setPosition(motor2_angle_rad)
    motor1.setVelocity(1.0)  # 可調整轉動速度
    motor2.setVelocity(1.0)  # 可調整轉動速度

    while robot.step(timestep) != -1:
        pass

if __name__ == "__main__":
    run_robot()