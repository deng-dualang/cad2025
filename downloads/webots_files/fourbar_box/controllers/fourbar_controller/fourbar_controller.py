from controller import Robot

def run_robot():
    # Create the Robot instance
    robot = Robot()

    # Get simulation time step
    timestep = int(robot.getBasicTimeStep())

    # Get motor device
    motor = robot.getDevice('motor')

    # Set motor for continuous rotation
    motor.setPosition(float('inf'))
    motor.setVelocity(1.0)

    # Main control loop
    while robot.step(timestep) != -1:
        pass

if __name__ == "__main__":
    run_robot()