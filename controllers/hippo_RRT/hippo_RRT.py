from controller import Robot, Motor, Emitter, Receiver, Keyboard

# This function needs to process messages recieved from other members, all messages should have
# sender id as first element as well as message type as second element, with all other info following
# these two elements, returns an int that represents a code as to what should be done based on message
def process(message):
    message = str(message)
    message = message.replace('b\'','')
    
    #print(message)
    parsed = message.split()
    if parsed[0] == 'Hippo:':
        print('Message Not for me, ignored')
    else:
        print(message)

# This function should send messages to other members and messages should follow the format as
# described above
def send(message):
    return 1
       
#Max angular rotation of Hound wheels
MAX_SPEED = 14
MAX_SPEEDX = 8
MAX_SPEEDZ = 0
Wheel23 = MAX_SPEEDX + MAX_SPEEDZ
Wheel14 = MAX_SPEEDX - MAX_SPEEDZ

#Initializes Robot and its parts
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
TIME_STEP = int(robot.getBasicTimeStep())

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

# Initializes all state machine variables
start_mode = True
done_mode = path_mode = stall_mode = wait_mode = push_mode = r_mode = False

# This is to ensure that a robot does not send multiple messages when one is good enough
sent_done = sent_request = False

#Initializes variables for path following mode
# Index: Index into the trajectory list
# Counter: Keeps track how long a input has been inputted since instructions are for a given
#          amount of time (Always a multiple of a time step) 
path = [[Wheel14,Wheel23,Wheel23,Wheel14,663],[0,0,0,0,5],[MAX_SPEED,-MAX_SPEED,MAX_SPEED,-MAX_SPEED,329],[0,0,0,0,5], "Push"]
print(path)
index = 0
counter = 0
path_mode = True
start_mode = False

# This is the main loop for the controller
while robot.step(TIME_STEP) != -1:

    # This initial if statement is the receiver code that handles messages from other swarm members
    # Checks if a message was recieved and if so it processes that message, either entering stall mode
    # or path mode depending on what was recieved, codes are as follows
    # -1: Hawk setup complete, begin waiting for instructions
    # 1: All members have recieved full path, time to move
    # 2-n: Stall order received, stall according to situation (this will change once actual code is here)
    # 0: Message does not effect state, continue as you were 
    if receiver.getQueueLength() > 0:
        message = receiver.getData()
        code = process(message)
        if code == 1:
            path_mode = True
            wait_mode = False
        elif code == -1:
            wait_mode = True
        elif (code != 0 and path_mode):
            path_mode = False
            stall_mode = True
            stall_code = code
        receiver.nextPacket()
    
    if start_mode:
        wheels[0].setVelocity(0)
        wheels[1].setVelocity(0)
        wheels[2].setVelocity(0)
        wheels[3].setVelocity(0)
    
    # Code for done mode, final state for Hound
    # Sets the velocities to zero and does not not deviate from this behaviour and will send message
    # To others that it is done (will only send this message once)0
    elif done_mode:
        wheels[0].setVelocity(0)
        wheels[1].setVelocity(0)
        wheels[2].setVelocity(0)
        wheels[3].setVelocity(0)
        if not sent_done:
            print("I am done")
            sent_done = True
            #send(Done)
            
    # This runs the push protocol to clear buttons, this is simply sets pusher down for a second and then
    # it enters the wait mode automatically, resets necessary variables at end                
    elif push_mode:
        counter = counter + 1
        if counter > 311:
            push_mode = False
            wait_mode = True
            counter = 0
            print("Pushed it")
        if counter > 275:
            wheels[0].setVelocity(-8)
            wheels[1].setVelocity(-8)
            wheels[2].setVelocity(-8)
            wheels[3].setVelocity(-8)
        elif counter > 270:
            wheels[0].setVelocity(0)
            wheels[1].setVelocity(0)
            wheels[2].setVelocity(0)
            wheels[3].setVelocity(0)
        else:
            wheels[0].setVelocity(8)
            wheels[1].setVelocity(8)
            wheels[2].setVelocity(8)
            wheels[3].setVelocity(8)
            
    # This section is dedicated to waiting for other memebers to do their job such as clear obstacles and
    # wait for new RRT orders
    elif wait_mode:
        wheels[0].setVelocity(0)
        wheels[1].setVelocity(0)
        wheels[2].setVelocity(0)
        wheels[3].setVelocity(0)
        if not sent_request:
            sent_request = True
            #send('Request')
            
            
    # This section is to handle stalls, idea is that depending on the messages between cars
    # this section will act accordingly to the stall necessary
    elif stall_mode:
        z = True
        # Decide what to do using stall code
        # Once done it must turn off stall mode and return to path_mode resuming instructions so it must
        # have a consistent state with the RRT by the time stall ends (i.e if someone must reverse the end
        # of the reverse must be consistent with the RRT path 
        
        
        
    # This section is to handle paths, this simply consumes instructions and moves along the instruction list
    # Stopping when it hits the done cap.
    elif path_mode:
        current = path[index]
        if type(current) == str:
            path_mode = False
            counter = 0
            index = 0
            path = []
            wheels[0].setVelocity(0)
            wheels[1].setVelocity(0)
            wheels[2].setVelocity(0)
            wheels[3].setVelocity(0)
            if current == "Done":
                done_mode = True
            elif current == "Push":
                push_mode = True
            elif current == "Wait":
                wait_mode = True
        else: 
            counter = counter + 1
            if counter >= current[4]:
                index = index + 1
                counter = 0
            wheels[0].setVelocity(current[0])
            wheels[1].setVelocity(current[1])
            wheels[2].setVelocity(current[2])
            wheels[3].setVelocity(current[3])