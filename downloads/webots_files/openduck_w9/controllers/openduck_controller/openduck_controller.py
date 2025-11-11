from controller import Robot, Motor, PositionSensor
import math

# Create Robot instance
robot = Robot()

# Get the time step of the current world
timestep = int(robot.getBasicTimeStep())

# Get motors t1 and t2
t1 = robot.getDevice('t1')
t2 = robot.getDevice('t2')
t3 = robot.getDevice('t3')

t1_degree = 20
t2_degree = 20
t3_degree = 90

# Get position sensors for t1 and t2
t1_sensor = robot.getDevice('t1_sensor')  # Use getDevice for the sensor
t2_sensor = robot.getDevice('t2_sensor')  # Use getDevice for the sensor
t3_sensor = robot.getDevice('t3_sensor')  # Use getDevice for the sensor

# Enable position sensors
t1_sensor.enable(timestep)
t2_sensor.enable(timestep)
t3_sensor.enable(timestep)

# Convert 10 degrees to radians (Webots uses radians for position)
t1_target_position = math.radians(t1_degree)
t2_target_position = math.radians(t2_degree)
t3_target_position = math.radians(t3_degree)

# Set motors to position control mode
t1.setPosition(t1_target_position)
t2.setPosition(t2_target_position)
t3.setPosition(t3_target_position)

# Set motor velocities
t1.setVelocity(1.0)
t2.setVelocity(1.0)
t3.setVelocity(1.0)

# Run simulation until motors reach target position
while robot.step(timestep) != -1:
    # Check if both motors have reached approximately the target position
    if (abs(t1_sensor.getValue() - t1_target_position) < 0.01 and
        abs(t2_sensor.getValue() - t2_target_position) < 0.01 and
        abs(t3_sensor.getValue() - t3_target_position) < 0.01):
        # Stop motors
        t1.setVelocity(0.0)
        t2.setVelocity(0.0)
        t3.setVelocity(0.0)
        break
