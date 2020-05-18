"""emitter_receiver_py controller."""
import struct
import threading
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Emitter, Receiver, Motor

class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        

# create the Robot instance.
robot = Robot()
commu_channel = 1
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
file = open("my_file.txt","w")
emit = robot.getEmitter("emitter")
receive = robot.getReceiver("receiver")
receive.enable(timestep) #enable the device

emit.setChannel(commu_channel)
a,b,c = 5,6,7

li = [1,2,3,4,5,6.7]
def set_list(li):
    lol = []
    for i in li:
        lol.append(i)
    return lol     

list1 = set_list(li)
fmt = "<%sd" % len(list1)
ss = '0.1 1.2 n 1.1 1.3 n'
def set_string(ss):
    return ss

def convert_to_binary(string_):
    return bytes(set_string(string_),'utf-8')

def send_trajectory(current_node, next_node):
    message = ''
    message = str(current_node.x) + ' ' + str(current_node.y) + ' n '
    message = message + str(next_node.x) + ' ' + str(current_node.y) + ' n'
    bin_data = convert_to_binary(message)
    emit.send(bin_data)

print("gg\n")
lock = threading.Lock()

class Thread_example:
    #def __init__(self):
    def hello_1(self):
        #lock.acquire()
        print("hello1\n")
        #self.count = self.count + 1
        #lock.release()
        print("hello1\n")
    def hello_2(self):
        #lock.acquire()
        print("hello2\n")
        #self.count = self.count + 1
        #lock.release()
        print("hello2\n")
    def run(self):
        t1 = threading.Thread(target=self.hello_1)
        t2 = threading.Thread(target=self.hello_2)
        t1.start()
        t2.start()
        t1.join()
        t2.join()
        #print(self.count)

#===========================================================collisionclass============================

#======================================================end-collision===================================================
    
#print(send_trajectory(Point(1.2,2.3),Point(1.1,2.2)))    
mess = bytes(set_string(ss),'utf-8')
#mm = struct.pack('{}s'.format(len(mess)),mess)
#print(mm)
message1 = struct.pack(fmt,*list1)
message2 = struct.pack('ii',3,2)
#message3 = struct.pack("s","type A")
message4 = struct.pack(fmt,*list1)
print(message4)
message5 = struct.pack('3s',b'end')
type_A = [3,4,5,6]
#c is for current node, n is for next node, a is for
m_l = [0,0,'n',1,1,'a',type_A]
#for i in m_l:
#    if isinstance(i,list):
#        for j in i:
#            message = message + struct.pack('i',j)
#    if isinstance(i,str):
#        message = message + struct.pack('c',i)
#    if isinstance(i,int):
#        message = message + struct.pack('i',i)
        
#fmt = fmt + '\n'
#cn = []
#result = []
#store_index = []
#for i in range(len(mmm)):
#   if mmm[i] == n:
 #       store_index_
    
   # result.append(i)    
  #  

#def pack_node_2send(current_node,)

me = "hippo"
receiv_traj_flag = True
my_trajectory = []

#parse the string trajectory to real trajectory
def str_traj_2_reaL_traj(trajectory):
    real_traj = []
    trajectory = trajectory[1:]
    print(trajectory)
    node = []
    i = 0
    while (i < len(trajectory)):
        if trajectory[i] == 'n':
            real_traj.append(node)
            node = []
            i = i + 1
        elif trajectory[i] == 'a':
            typea = []
            i = i + 1
            while(trajectory[i] != 'b'):
                typea.append([float(trajectory[i]),float(trajectory[i+1])])
                i = i + 2
            node.append(typea)
        elif trajectory[i] == 'b':
            typeb = []
            i = i + 1
            while(trajectory[i] != 'c'):
                typeb.append([float(trajectory[i]),float(trajectory[i+1])])
                i = i + 2
            node.append(typeb)
        elif trajectory[i] == 'c':
            typec = []
            i = i + 1
            while(trajectory[i] != 'n'):
                typeb.append([float(trajectory[i]),float(trajectory[i+1])])
                i = i + 2
            node.append(typeb)                    
        elif trajectory[i] == 'e':
            break
        else:
            node.append(float(trajectory[i]))
            i = i + 1
    return real_traj       
            
while robot.step(timestep) != -1:
	#receiving trajecotry from hawk, only once time
	if receiv_traj_flag:
		while(receive.getQueueLength() > 0):
			bin_data = receive.getData()
			str_data = str(bin_data)[2:][:-1]
			str_data = str_data.split()
			if str_data[0] == me:
				receiv_traj_flag = False
				#print(str_data)
				my_trajectory = str_traj_2_reaL_traj(str_data)
				print(my_trajectory)
				break
			receive.nextPacket()
			
	
    
    # Read the sensors:
    
    #emit.send(message2)
    #print("hello emiter \n")    
    #emit.send(message4)
    #print("hello emiter \n") 
    #emit.send(message5)
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

# Enter here exit cleanup code.

