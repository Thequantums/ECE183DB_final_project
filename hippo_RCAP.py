"""emitter_receiver_py controller."""
import threading
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Emitter, Receiver, Motor, GPS, Compass
import math
import numpy as np
import collision_geometry as cg
        
#========================hippopy_code, code for driving================================================

#plateform_speed is the speed of the body in m/s for SPEED define as 4.0 bellow
SPEED = 4.0
DISTANCE_TOLERANCE = 0.02
ANGLE_TOLERANCE = 0.02

K1 = 80.0
K2 = 1.0
K3 = 1.0

timestep =  32

#============code for tiny math===========================

class Vector2:
	def __init__(self,u,v):
		self.u = u
		self.v = v

class Vector3:
	def __init__(self,u,v,w):
		self.u = u
		self.v = v
		self.w = w

class Matrix33:
	def __init__(self,a,b,c):
		self.a = a
		self.b = b
		self.c = c

def vector3_set_values(vect, u, v, w):
	vect.u = u
	vect.v = v
	vect.w = w
	

def matrix33_set_values(m,au,av,aw,bu,bv,bw,cu,cv,cw):
	vector3_set_values(m.a,au,av,aw)
	vector3_set_values(m.b,bu,bv,bw)
	vector3_set_values(m.c,cu,cv,cw)


def matrix33_set_identity(m):
	matrix33_set_values(m,1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)


def matrix33_mult_vector3(res, m, v):
	res.u = m.a.u * v.u + m.b.u * v.v + m.c.u * v.w
	res.v = m.a.v * v.u + m.b.v * v.v + m.c.v * v.w
	res.w = m.a.w * v.u + m.b.w * v.v + m.c.w * v.w

def vector2_norm(v):
	return math.sqrt(v.u * v.u + v.v * v.v)


def vector2_minus(v,v1,v2):
	v.u = v1.u - v2.u
	v.v = v1.v - v2.v

def vector2_angle(v1,v2):
	return math.atan2(v2.v, v2.u) - math.atan2(v1.v, v1.u)

def bound(v,a,b):
	if v > b:
		return b
	elif v < a:
		return a
	else:
		return v

#==========================end_tiny math code====================================

lock = 0
node_num = 0

class goto_struct:
	def __init__(self,v,a,r):
		self.v_target = v
		self.alpha = a
		self.reached = r

class node:
	def __init__(self,x,z,alpha):
		self.x = x
		self.z = z
		self.alpha = alpha


init_node = node(0,0,0)
init_lock = 0
robot = Robot()

motors = []
motors.append(robot.getMotor("wheel1"))
motors.append(robot.getMotor("wheel2"))
motors.append(robot.getMotor("wheel3"))
motors.append(robot.getMotor("wheel4"))

gps = robot.getGPS("gps")
compass = robot.getCompass("compass")
gps.enable(timestep)
compass.enable(timestep)
goto_data = goto_struct(Vector2(0.0,0.0),0.0,False)

def base_set_wheel_speeds_helper(speeds):
	for i in range(len(motors)):
		motors[i].setPosition(float('+inf'))
		motors[i].setVelocity(speeds[i])

def base_goto_set_target(x,z,alpha):
	goto_data.v_target.u = x
	goto_data.v_target.v = z
	goto_data.alpha = alpha
	goto_data.reached = False

def base_reset():
	speeds = [0.0, 0.0, 0.0, 0.0]
	base_set_wheel_speeds_helper(speeds)

def base_goto_run():
	v_gps = Vector2(gps.getValues()[0], gps.getValues()[2])
	v_front = Vector2(compass.getValues()[0], compass.getValues()[1])
	v_right = Vector2(-v_front.v, v_front.u)
	v_north = Vector2(1.0,0.0)
	
	v_dir = Vector2(0.0,0.0)
	vector2_minus(v_dir,goto_data.v_target, v_gps)
	distance = vector2_norm(v_dir)
	
	theta = vector2_angle(v_front, v_north)
	delta_angle = theta - goto_data.alpha
	
	transform = Matrix33(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0))
	matrix33_set_identity(transform)
	transform.a.u = v_front.u;
	transform.a.v = v_right.u;
	transform.b.u = v_front.v;
	transform.b.v = v_right.v;
	transform.c.u = -v_front.u * v_gps.u - v_front.v * v_gps.v;
	transform.c.v = -v_right.u * v_gps.u - v_right.v * v_gps.v;
	
	v_target_tmp = Vector3(goto_data.v_target.u, goto_data.v_target.v, 1.0)
	v_target_rel = Vector3(0,0,0)
	matrix33_mult_vector3(v_target_rel, transform, v_target_tmp)
	
	speeds = [0.0, 0.0, 0.0, 0.0]
	speeds[0] = -delta_angle / np.pi * K1
	speeds[1] = delta_angle / np.pi * K1
	speeds[2] = -delta_angle / np.pi * K1
	speeds[3] = delta_angle / np.pi * K1
	
	speeds[0] = speeds[0] + v_target_rel.u * K2
	speeds[1] = speeds[1] + v_target_rel.u * K2
	speeds[2] = speeds[2] + v_target_rel.u * K2
	speeds[3] = speeds[3] + v_target_rel.u * K2
	
	speeds[0] = speeds[0] - v_target_rel.v * K3
	speeds[1] = speeds[1] + v_target_rel.v * K3
	speeds[2] = speeds[2] + v_target_rel.v * K3
	speeds[3] = speeds[3] - v_target_rel.v * K3

	for i in range(4):
		speeds[i] = speeds[i] / (K1 + K2 + K3)
		speeds[i] = speeds[i] * SPEED
		speeds[i] = speeds[i] * 200.0
		speeds[i] = bound(speeds[i], -SPEED, SPEED)
		
	base_set_wheel_speeds_helper(speeds)
	
	#if distance < DISTANCE_TOLERANCE and delta_angle < ANGLE_TOLERANCE and delta_angle > -ANGLE_TOLERANCE:
	#	goto_data.reached = True
	#("distance: ", distance)
	if distance < DISTANCE_TOLERANCE:
		goto_data.reached = True
	

def base_goto_reached():
	return goto_data.reached
	
def base_goto_straight_to_direction(init_x, init_z, x_t, z_t):
	#print("init x, init y, x_t , z_t: ", init_x, init_z, x_t, z_t)
	global init_lock, node_num, init_node
	x = x_t - init_x
	z = z_t - init_z
	if x == 0:
		if z >= 0:
			angle = -3.141/2
		else:
			angle = 3.141/2
	else:
		angle = math.atan(z/x)
		#print("atan: ", angle)
		if x >= 0 and z >= 0:
			angle = angle * -1
		if x < 0 and z >= 0:
			angle = -3.141 - angle
		if x < 0 and z < 0:
			angle = 3.141 - angle
		if x > 0 and z < 0:
			angle = angle * -1

	#file.write("angle: %f " %angle)
	base_goto_set_target(x_t, z_t, angle)
	base_goto_run()
	if base_goto_reached():
		#print("hello")
		base_reset()
		init_lock = 0
		node_num = node_num + 1

def base_run(x,z,alpha):
	global init_lock, init_node
	if init_lock == 0:
		init_node.x = gps.getValues()[0]
		init_node.z = gps.getValues()[2]
	init_lock = 1
	base_goto_straight_to_direction(init_node.x , init_node.z, x, z)
	

#==========================End_code for Driving============================================================


# create the Robot instance.
commu_channel = 1
# get the time step of the current world.
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
receive.enable(timestep)
emit.setChannel(commu_channel)

#===========================================================collisionclass============================
#!/usr/bin/python3
import threading

#my_trajectory has a list of nodes.
#each node is a list as [x,y,parent_node.x,parent_node.y,type A, type B, type C]
#type A or B or C has a form [[x,y],[x1,y1],[x2,y2],[x3,y3],...]

#three types of conflict nodes, A,B,C
#Type A: attemp_to_go_conflict_node (Crossing path) (list)
#Type B: driving conflict node (list)
#Type C: arriving conflict node (list)

#All robot id list, now we have two robots. Hound's ID is 2. Hippo's ID is 1

my_state = 0 #starting node is always 0
my_trajectory = []
receiv_traj_flag = True #set it true so the robot is waiting to receive its trajecotry

me = "11" #code for receiving hound trajectory
priority_id = "hippo"
reply_R2_info = [] #for path quary
reply_R2_state = [] #for state quary
receive_reply = True

my_turn = False

lock = threading.Lock()
my_id = 1
my_radius = 0.6
hound_radius = 0.6

my_waiting_state = 1 #0 for transitioning state
knowing_other_is_stalled = 0 # 1 if i know other my_waiting_state = 1
hound_arrived_at_a_node = 0
agreement = 0
R2_location = []

hound_block_state = False
hippo_block_state = False
fall_back = False

hound_arrived = False

execute_fall_back = False
 
class Collision_avoidance:

	def get_state(self):
		global my_state
		lock.acquire()
		mstate = my_state
		lock.release()
		return mstate
		
	def set_state(self,index):
		global my_state
		lock.acquire()
		index = int(index)
		print("Hippo state changes to ", index)
		my_state = index
		lock.release()
		
	def convert_bin_to_string(self,st):
		str_data = str(st)[2:][:-1]
		return str_data

	def convert_string_to_bin(self,string_):
		return bytes(string_,'utf-8')


	#This function is to run in parallel with the execute main. It keeps listenning for sending from other robots and invoke handlers coorespondingly
	#Using lock to lock some shared variables between the threads
	#*****Import Note about communication******: Even with the same channel number, each robot has their own seperate queue. They dont share the same channel.

	def reply(self, type, sender_id):
		if type == "q_state":
			st = sender_id + ' ' + 'q_state_ans'  + ' ' + str(my_waiting_state) + ' ' + str(knowing_other_is_stalled) + ' ' + str(my_id)
		if type == "q_path":
			current_state = self.get_state()
			if current_state == (len(my_trajectory) - 1):
				#if the current_state is the goal state
				current_x, current_y, next_x, next_y = my_trajectory[current_state][0], my_trajectory[current_state][1],my_trajectory[current_state][0],my_trajectory[current_state][1]
			else:
				current_x, current_y, next_x, next_y = my_trajectory[current_state][0], my_trajectory[current_state][1], my_trajectory[current_state+1][0], my_trajectory[current_state+1][1]
			st = sender_id + ' ' + 'q_path_ans' + ' ' + str(current_x) + ' ' + str(current_y) + ' ' + str(next_x) + ' ' + str(next_y) + ' ' + str(my_id)
		print("hound replies back: ", st)
		st = self.convert_string_to_bin(st)
		emit.send(st)


	def reply_handler(self, data):
		global reply_R2_state, receive_reply, reply_R2_info
		if data[0] == "q_state_ans":
			lock.acquire()
			reply_R2_state = [int(data[1]), int(data[2])]
			receive_reply = True
			lock.release()
		if data[0] == "q_path_ans":
			lock.acquire()
			reply_R2_info = [float(data[1]), float(data[2]), float(data[3]), float(data[4])]
			receive_reply = True
			lock.release()

	def location_handler(self,sender_id):
		st = sender_id + ' ' + 'location_anw'  + ' ' + str(my_trajectory[self.get_state()][0]) + ' ' + str(my_trajectory[self.get_state()][1])
		st = self.convert_string_to_bin(st)
		emit.send(st)
		
	def listen(self):
		global my_trajectory, receiv_traj_flag, agreement, my_turn, R2_location, receive_reply, hound_block_state, fall_back, execute_fall_back, hound_arrived
		while robot.step(timestep) != -1:
			#receiving trajecotry from hawk, only once time
			if receiv_traj_flag:
				while(receive.getQueueLength() > 0):
					bin_data = receive.getData()
					str_data = self.convert_bin_to_string(bin_data)
					str_data = str_data.split()
					#print("string data: ", str_data)
					if str_data[0] == me:
						receiv_traj_flag = False
						#setting my_trajectory
						my_trajectory = self.str_traj_2_reaL_traj(str_data)
						print(my_trajectory)
						#print("hound trajectory length is: ", len(my_trajectory))
						receive.nextPacket()
						break
					receive.nextPacket()	
			else:
				if receive.getQueueLength() > 0:
					bin_data = receive.getData()
					str_data = self.convert_bin_to_string(bin_data)
					str_list = str_data.split()
					#print("hound receiver: ", str_list)
					if float(str_list[0]) == my_id:
						str_list = str_list[1:] #strip my_id field off
						if str_list[0] == "urturn":
							if self.get_state() != len(my_trajectory) - 1:
								print("hippo receive my_turn")
							my_turn = True
							receive.nextPacket() #remove the packet from communication channel
						elif str_list[0] == "location":
							self.location_handler(str_list[1])
							receive.nextPacket() #remove the packet from communication channel
						elif str_list[0] == "location_anw":
							R2_location = [float(str_list[1]), float(str_list[2])]
							receive_reply = True
							receive.nextPacket() #remove the packet from communication channel
						elif str_list[0] == "blocked":
							print("hound is blocked")
							hound_block_state = True
							receive.nextPacket() #remove the packet from communication channel
						elif str_list[0] == "unblocked":
							hound_block_state = False
							receive.nextPacket()
						elif str_list[0] == "arrived":
							hound_arrived = True
							receive.nextPacket()
						elif str_list[0] == "fall":
							print("hippo receives fall back command")
							execute_fall_back = True
							receive.nextPacket()
						else:
							receive.nextPacket()#remove the packet from communication channel	
					else:
						receive.nextPacket()
						

	#A wrapper function for drive function
	#drive function that takes the robot to x,z, Once it reached the state x,z, it sets the reached flag to true
	#This function only return after the car has reached the specified node.
	def goto_target(self,i):
		#drive to the target node in the my_trajectory
		global my_waiting_state
		i = int(i)
		finish_one_drive = True
		print("hippo is driving to: ", i)
		goto_x, goto_y = my_trajectory[i][0], my_trajectory[i][1]
		my_waiting_state = 0 #i'm going out
		while robot.step(timestep) != -1:
			if base_goto_reached() == False or finish_one_drive == True:
				base_run(goto_x, goto_y, 1)
				finish_one_drive = False
			else:
				my_waiting_state = 1 #arrived, switch to stalled state
				if i == (len(my_trajectory) - 1):
					self.send_i_arrived()
				print("hippo is arriving at state: ", i)
				self.set_state(i) 
				break
				

	#parse the string trajectory to real trajectory
	#each node has the form of [current_x, current_y, parent_x, parent_y, type_a, type_b, type_c]
	#typea and typeb and typec has the form as [[x,y], [x1,y1], [x2,y2],...]
	def str_traj_2_reaL_traj(self,trajectory):
		real_traj = []
		trajectory = trajectory[1:]
		#print(trajectory)
		node = []
		i = 0
		while (i < len(trajectory)):
			if trajectory[i] == 'n':
				real_traj.append(node)
				node = []
				i = i + 1                   
			elif trajectory[i] == 'e':
				break
			else:
				node.append(float(trajectory[i]))
				i = i + 1
		return real_traj


	def quary_state(self):
		global receive_reply, reply_R2_state, knowing_other_is_stalled
		quary = "1 q_state 2" #1 is hippo id, q_state is question type, 2 is my id
		bin_data = self.convert_string_to_bin(quary)
		receive_reply = False
		emit.send(bin_data)
		while robot.step(timestep) != -1:
			print("hound stuck here")
			if receive_reply == True:
				break
		if reply_R2_state[0] == 1:
			knowing_other_is_stalled = 1
		else:
			knowing_other_is_stalled = 0
		temp_reply = reply_R2_state
		reply_R2_state = []
		return temp_reply
	
	def quary_path(self):
		global receive_reply, reply_R2_info
		quary = "1 q_path 2" #1 is hippo id, q_state is question type, 2 is my id
		bin_data = self.convert_string_to_bin(quary)
		receive_reply = False
		emit.send(bin_data)
		while robot.step(timestep) != -1:
			if receive_reply == True:
				break
		temp_reply = reply_R2_info
		reply_R2_info = []
		return temp_reply

	def check_two_path_overlaps(self,R2_path):
		i = self.get_state()
		if i == (len(my_trajectory) -1):
			#i am at goal node
			my_p1, my_p2 = cg.Point(my_trajectory[i][0],my_trajectory[i][1]), cg.Point(my_trajectory[i][0],my_trajectory[i][1])
		else:
			my_p1, my_p2 = cg.Point(my_trajectory[i][0],my_trajectory[i][1]), cg.Point(my_trajectory[i+1][0],my_trajectory[i+1][1])
		R2_p1, R2_p2 = cg.Point(R2_path[0], R2_path[1]), cg.Point(R2_path[2],R2_path[3])
		
		return cg.check_if_two_path_overlaps(my_p1, my_p2, my_radius, R2_p1, R2_p2, hound_radius)
	
	def send_agreement(self):
		st = "2 ready 2"
		st = self.convert_string_to_bin(st)
		emit.send(st)

	def send_your_turn_message(self):
		global my_turn
		my_turn = False
		st = "2 urturn 1"
		st = self.convert_string_to_bin(st)
		emit.send(st)		

	def quary_location(self):
		global R2_location, receive_reply
		st = "2 location 1"
		st = self.convert_string_to_bin(st)
		receive_reply = False
		emit.send(st)
		while robot.step(timestep) != -1:
			#print("hound stuck here")
			if receive_reply == True:
				break
		temp_reply = R2_location
		R2_location = []
		return temp_reply		

	def path_has_obstacle(self,R2_info):
		i = self.get_state()
		my_current, my_next = cg.Point(my_trajectory[i][0],my_trajectory[i][1]), cg.Point(my_trajectory[i+1][0],my_trajectory[i+1][1])
		R2_current = cg.Point(R2_info[0],R2_info[1])
		return cg.check_path(my_current, my_next, my_radius, R2_current, hound_radius)

	def send_iamblocked(self):
		st = "2 blocked 1"
		st = self.convert_string_to_bin(st)
		emit.send(st)			
	
	def fall_back_success(self):
		st = "2 fall 1"
		st = self.convert_string_to_bin(st)
		emit.send(st)		

	def send_iam_unblocked(self):
		st = "2 unblocked 1"
		st = self.convert_string_to_bin(st)
		emit.send(st)	
	
	def send_i_arrived(self):
		st = "2 arrived 1"
		st = self.convert_string_to_bin(st)
		emit.send(st)	
	
	def execute_main(self):
		global my_turn, hippo_block_state, fall_back, execute_fall_back
		fall_back_once = True
		#length = len(my_trajectory)
		while robot.step(timestep) != -1:
			if receiv_traj_flag == False:
				#print("hello i'm hound")
				#print(my_trajectory)
				#already received the my_trajectory.
				#i = self.get_state()
				if self.get_state() != (len(my_trajectory) - 1):
					#Not arrived at goal state yet
					#print("hippo is stuck here")
					if self.get_state() == 0 and priority_id == "hound":
						self.goto_target(self.get_state()+1)
						self.send_your_turn_message()
					else:
						if hound_arrived == True:
							self.goto_target(self.get_state()+1)
						elif my_turn == True:
							print("hippo turn")
							R2_info = self.quary_location()
							if self.path_has_obstacle(R2_info):
								hippo_block_state = True
								print("hippo is blocked")
								self.send_your_turn_message()
								self.send_iamblocked()
							else:
								hippo_block_state = False
								self.send_iam_unblocked()
								self.goto_target(self.get_state()+1)
								self.send_your_turn_message()
						else:
							if hound_block_state == True and hippo_block_state == True:
								print("hippo is facing deadlock")
								if priority_id == "hippo":
									#fall back
									if  execute_fall_back == True and self.get_state() - 1 >= 0:
										print("hippo executing fall back:")
										self.goto_target(self.get_state() - 1)
										self.send_your_turn_message()
										fall_back = True
										execute_fall_back = False
										self.fall_back_success()
								else:
									while robot.step(timestep) != -1:
										if fall_back == True:
											fall_back = False
											break
				else:
					pass
					#print("Hippo has arrived at goal node")
	
	def run(self):
		t1 = threading.Thread(target=self.execute_main)
		t2 = threading.Thread(target=self.listen)
		t1.start()
		t2.start()
		t1.join()
		t2.join()

th = Collision_avoidance()
th.run()

#======================================================end-collision===================================================	
    
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
