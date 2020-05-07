"""my_controller_tut_6_python controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']

for i in range(4):
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
    
avoidObstacleCounter = 0
ds = []
dsNames = ['ds_right', 'ds_left']

for i in range(2):
    ds.append(robot.getDistanceSensor(dsNames[i]))
    ds[i].enable(timestep)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    leftSpeed = 6.28
    rightSpeed = 6.28

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    
    if avoidObstacleCounter > 0:
        avoidObstacleCounter -= 1
        leftSpeed = 6.28
        rightSpeed = -6.28
    else:  # read sensors
        for i in range(2):
            if ds[i].getValue() < 950.0:
                avoidObstacleCounter = 30
        

    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)

# Enter here exit cleanup code.
