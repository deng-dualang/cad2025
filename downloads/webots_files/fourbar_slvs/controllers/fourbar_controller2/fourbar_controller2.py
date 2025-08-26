from controller import Robot

def run_robot():
    # Create the Robot instance
    robot = Robot()

    # Get simulation time step
    timestep = int(robot.getBasicTimeStep())

    # Get motor device
    motor1 = robot.getDevice('joint_motor1')
    motor2 = robot.getDevice('joint_motor2')
    #motor3 = robot.getDevice('joint_motor3')

    # Set motor for continuous rotation
    motor1.setPosition(float('inf'))
    motor1.setVelocity(1.0)
    motor2.setPosition(float('inf'))
    motor2.setVelocity(1.0)
    """
    motor3.setPosition(float('inf'))
    motor3.setVelocity(1.0)
    """

    # Main control loop
    while robot.step(timestep) != -1:
        pass

if __name__ == "__main__":
    run_robot()