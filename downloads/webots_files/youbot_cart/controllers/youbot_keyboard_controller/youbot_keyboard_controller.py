from controller import Robot, Keyboard

# Constants
TIME_STEP = 32  # Simulation time step in milliseconds
WHEEL_RADIUS = 0.1  # Radius of the wheels in meters (10cm)
L = 0.471  # Half of the robot's length in meters
W = 0.376  # Half of the robot's width in meters
MAX_VELOCITY = 10.0  # Maximum velocity allowed for the wheels

# Initialize the robot
robot = Robot()

# Initialize the keyboard
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

# Get motor devices
wheel1 = robot.getDevice("wheel1")  # Front-right wheel
wheel2 = robot.getDevice("wheel2")  # Front-left wheel
wheel3 = robot.getDevice("wheel3")  # Rear-right wheel
wheel4 = robot.getDevice("wheel4")  # Rear-left wheel

# Set motors to velocity control mode
for wheel in [wheel1, wheel2, wheel3, wheel4]:
    wheel.setPosition(float('inf'))  # Enable velocity control
    wheel.setVelocity(0)  # Set initial velocity to 0

def set_wheel_velocity(v1, v2, v3, v4):
    """Set the velocity of all wheels."""
    wheel1.setVelocity(v1)
    wheel2.setVelocity(v2)
    wheel3.setVelocity(v3)
    wheel4.setVelocity(v4)

# Main loop
print("Use arrow keys to control the robot.")
print("Up: Move forward, Down: Move backward, Left: Turn left, Right: Turn right.")
print("Press 'Q' to quit.")

while robot.step(TIME_STEP) != -1:
    key = keyboard.getKey()  # Read the key pressed

    if key == Keyboard.UP:
        # Move forward
        velocity = MAX_VELOCITY
        set_wheel_velocity(velocity, velocity, velocity, velocity)
    elif key == Keyboard.DOWN:
        # Move backward
        velocity = -MAX_VELOCITY
        set_wheel_velocity(velocity, velocity, velocity, velocity)
    elif key == Keyboard.RIGHT:
        # Turn right
        velocity = MAX_VELOCITY
        set_wheel_velocity(-velocity, velocity, -velocity, velocity)
    elif key == Keyboard.LEFT:
        # Turn left
        velocity = MAX_VELOCITY
        set_wheel_velocity(velocity, -velocity, velocity, -velocity)
    elif key == ord('Q') or key == ord('q'):
        # Quit the program
        print("Exiting...")
        break
    else:
        # Stop the wheels when no key is pressed
        set_wheel_velocity(0, 0, 0, 0)