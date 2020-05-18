"""hippoer controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, Emitter, Receiver, Keyboard

SPEED = 5
TS = 7.005751618

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
TIME_STEP = int(robot.getBasicTimeStep())

keyboard = robot.getKeyboard()
keyboard.enable(TIME_STEP)

emitter = robot.getEmitter('emitter')
receiver = robot.getReceiver('receiver')
receiver.enable(TIME_STEP)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
    
while robot.step(TIME_STEP) != -1:
    if robot.getTime() > 1.0:
        break

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    Speeds = [0,0,0,0]

    # Process sensor data here.
    
    key = keyboard.getKey()
    while (key > 0):
        if key == keyboard.UP:
            Speeds = [SPEED, SPEED, SPEED, SPEED]
        elif key == keyboard.DOWN:
            Speeds = [-SPEED, -SPEED, -SPEED, -SPEED]
        elif key == keyboard.RIGHT:
            Speeds = [-SPEED, SPEED, SPEED, -SPEED]
        elif key == keyboard.LEFT:
            Speeds = [SPEED, -SPEED, -SPEED, SPEED]
        elif key == keyboard.SHIFT + keyboard.RIGHT:
            Speeds = [-SPEED, SPEED, -SPEED, SPEED]
        elif key == keyboard.SHIFT + keyboard.LEFT:
            Speeds = [SPEED, -SPEED, SPEED, -SPEED]
        elif key == keyboard.SHIFT + keyboard.LEFT:
            Speeds = [SPEED, -SPEED, SPEED, -SPEED]
        key = keyboard.getKey()
    
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    for i in range(4):
        wheels[i].setVelocity(Speeds[i])

# Enter here exit cleanup code.
