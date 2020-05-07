from controller import Robot, Motor, Emitter, Receiver, Keyboard

def process(message):
    
    message = str(message)
    message = message.replace('b\'','')
    
    #print(message)
    parsed = message.split()
    if parsed[0] == 'Hippo:':
        print('Message Not for me, ignored')
    else:
        print(message)
       

MAX_SPEED = 20

robot = Robot()

TIME_STEP = int(robot.getBasicTimeStep())

keyboard = robot.getKeyboard()
keyboard.enable(TIME_STEP)

emitter = robot.getEmitter('emitter')
receiver = robot.getReceiver('receiver')
receiver.enable(TIME_STEP)

wheels = []
wheelsNames = ['wheel1', 'wheel2', 'boop']
for i in range(3):
    wheels.append(robot.getMotor(wheelsNames[i]))
    if i == 2:
        wheels[i].setPosition(0.0)
    else:
        wheels[i].setPosition(float('inf'))
        wheels[i].setVelocity(0.0)

while robot.step(TIME_STEP) != -1:
    if robot.getTime() > 1.0:
        break

push = 0

while robot.step(TIME_STEP) != -1:

    leftSpeed = 0
    rightSpeed = 0

    if receiver.getQueueLength() > 0:
        message = receiver.getData()
        #print(message)
        process(message)
        receiver.nextPacket()


    key = keyboard.getKey()
    while (key > 0):
        if key == keyboard.UP:
            leftSpeed = MAX_SPEED
            rightSpeed = MAX_SPEED
        elif key == keyboard.DOWN:
            leftSpeed = -MAX_SPEED
            rightSpeed = -MAX_SPEED
        elif key == keyboard.RIGHT:
            leftSpeed = MAX_SPEED
            rightSpeed = 0*MAX_SPEED
        elif key == keyboard.LEFT:
            leftSpeed = 0*MAX_SPEED
            rightSpeed = MAX_SPEED
        elif key == keyboard.SHIFT + keyboard.RIGHT:
            leftSpeed = MAX_SPEED
            rightSpeed = -MAX_SPEED
        elif key == keyboard.SHIFT + keyboard.LEFT:
            leftSpeed = -MAX_SPEED
            rightSpeed = MAX_SPEED
        elif key == keyboard.SHIFT + keyboard.DOWN:
            push = .84
        elif key == keyboard.SHIFT + keyboard.UP:
            push = 0
        key = keyboard.getKey()
        

    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setPosition(push)

