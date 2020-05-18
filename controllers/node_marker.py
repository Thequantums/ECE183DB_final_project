"""node_marker controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

from controller import Robot, Emitter, Receiver, Motor 
import collision_geometry as cg
from matplotlib import pyplot as plt
import numpy as np
# create the Robot instance.
robot = Robot()

communication_channel = 1
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
file = open("my_file.txt","w")
emit = robot.getEmitter("emitter")
receive = robot.getReceiver("receiver")
emit.setChannel(communication_channel)
receive.enable(timestep) #enable receiver
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
hound_radius = 0.1
hippo_radius = 0.2

def RRT():
	hippo_path = [[0.5,0.5,0.5,0.5], [1,1,0.5,0.5], [1.5,1.3,1,1], [0.5,2,1.5,1.3], [3,2.4,0.5,2], [3.6,2,3,2.4], [3.8,3,3.6,2], [4,5,3.8,3]]
	hound_path = [[2.5,0,2.5,0],[2.5,1,2.5,0],[1.5,0.5,2.5,1],[0.9,1.5,1.5,0.5],[2,2,0.9,1.5],[1.8,3.5,2,2],[2.6,2,1.8,3.5],[4.5,2.7,2.6,2]]
	return hound_path, hippo_path

s = "0.5 0.5 0.5 0.5 a 0 0 0 0 b 0 0 0 0 c 0 0 0 0 n 0.5 0.5 0.5 0.5 a 0 0 0 0 b 0 0 0 0 c 0 0 0 0 n e"

def conv_trajectory2strformat(traj):
    message = ""
    for i in traj:
        message = message + str(i[0]) + ' ' + str(i[1]) + ' ' + str(i[2]) + ' ' + str(i[3]) + ' ' + 'a' + ' '
        for j in i[4]:
            message = message + str(j[0]) + ' ' + str(j[1]) + ' '
        message = message + 'b' + ' '
        for j in i[5]:
            message = message + str(j[0]) + ' ' + str(j[1]) + ' '
        message = message + 'c' + ' '
        for j in i[6]:
            message = message + str(j[0]) + ' ' + str(j[1]) + ' '
        message = message + 'n '
    message = message + 'e'
    return message

def convert_to_binary(string_):
    return bytes(string_,'utf-8')

def send_trajectory2hound_hippo(hound_path,hippo_path):
	str_format_hound_path = conv_trajectory2strformat(hound_path)
	str_format_hound_path = 'hound ' + str_format_hound_path
	str_format_hippo_path = conv_trajectory2strformat(hippo_path)
	str_format_hippo_path = ' hippo ' + str_format_hippo_path
	bin_data_hound = convert_to_binary(str_format_hound_path)
	bin_data_hippo = convert_to_binary(str_format_hippo_path)
	emit.send(bin_data_hound)
	emit.send(bin_data_hippo)

#================main()==================#


send_once = True

while robot.step(timestep) != -1:
	if send_once:
		hound_path, hippo_path = RRT()
		for i in range(len(hound_path)):
			hound_path[i].extend(([],[],[]))

		for i in range(len(hippo_path)):
			hippo_path[i].extend(([],[],[]))

		#Mark the conflict nodes and exchange information between hound and hippo
		for i in range(len(hound_path) - 1):
			init_hound = cg.Point(hound_path[i][0], hound_path[i][1]) #point of current node of hound
			final_hound = cg.Point(hound_path[i + 1][0], hound_path[i + 1][1]) #point of next node of hound
			for j in range(len(hippo_path) - 1):
				init_hippo = cg.Point(hippo_path[j][0], hippo_path[j][1]) #point of current node of hippo
				final_hippo = cg.Point(hippo_path[j + 1][0], hippo_path[j + 1][1]) #point of next node of hippo
				if cg.doIntersect(init_hound, final_hound, init_hippo, final_hippo):
					# paths crossed between hound and hippo. This create a type A conflict
					hound_path[i][4].append([init_hippo.x, init_hippo.y])
					hippo_path[j][4].append([init_hound.x, init_hound.y])
				if cg.check_two_circles_intersect(init_hound,hound_radius,init_hippo,hippo_radius):
					# if a hound and a hippo occupied a node with their respective radii, then it overlaps. Type C conflict
					hound_path[i][6].append([init_hippo.x, init_hippo.y])
					hippo_path[j][6].append([init_hound.x, init_hound.y])
				if cg.check_robot_is_moving(init_hound, final_hound, hound_radius, init_hippo, hippo_radius):
					# hound is moving from inital node to next node, check if hippo is stationary in j node can overlap the path hound is moving. Type B conflict
					hound_path[i][5].append([init_hippo.x, init_hippo.y])
				if cg.check_robot_is_moving(init_hippo, final_hippo, hippo_radius, init_hound, hound_radius):
					# hippo is moving from inital node to next node, check if hound is stationary in i node can overlap the path hippo is moving. Type B conflict
					hippo_path[j][5].append([init_hound.x, init_hound.y])

		#check type C conflicts for goal node of hound and hippo
		hound_goal_node = cg.Point(hound_path[len(hound_path) -1][0],hound_path[len(hound_path) -1][1])
		hippo_goal_node = cg.Point(hippo_path[len(hippo_path) -1][0], hippo_path[len(hippo_path) -1][1])
		for i in range(len(hippo_path)):
			init_hippo = cg.Point(hippo_path[i][0], hippo_path[i][1]) #point of current node of hippo
			if cg.check_two_circles_intersect(hound_goal_node,hound_radius,init_hippo,hippo_radius):
				hound_path[len(hound_path) - 1][6].append([init_hippo.x, init_hippo.y])
		for i in range(len(hound_path)):
			init_hound = cg.Point(hound_path[i][0], hound_path[i][1])
			if cg.check_two_circles_intersect(hippo_goal_node, hippo_radius, init_hound, hound_radius):
				hippo_path[len(hippo_path) - 1][5].append([init_hound.x, init_hound.y])
		send_trajectory2hound_hippo(hound_path,hippo_path)
		send_once = False #only send one time
	#while (True):
		# each node has the form of: x, y, parentx, parenty, type A conflicts nodes, type B conflict nodes, type C conflict nodes.
		#Type A: attemp_to_go_conflict_node (Crossing path) (list)
		#Type B: driving conflict node (list)
		#Type C: arriving conflict node (list)
		# 7th field is to store the potential nodes of colliding while driving
		#Append three empty lists to each nodes. The three lists are for storing type A, type B, type C conflicts nodes
		
		
		
		
		
#print(hippo_path)
#print("\n\n")
#print(hound_path)
#print("hello")
#send_trajectory2hound_hippo(hound_path,hippo_path)

#fig, ax = plt.subplots()
#line, = ax.plot([], [], lw=3)
#ax.grid()
#theta = np.linspace(0,2*np.pi,100)
#ax.set_ylim(0,6)
#ax.set_xlim(0,6)

#def circle(cx,cy,r):
#    x = r * np.cos(theta) + cx
#    y = r * np.sin(theta) + cy
#   return x,y

#line formed by two points from p1 to p2
#def line(p1,p2):
#    if p2.x - p1.x == 0:
#        quality = abs(p2.y - p1.y) * 50
#        y = np.linspace(p1.y,p2.y,quality)
#        r = y.shape
#        x = np.full((r[0],1),p1.x)
#        return x,y
#    if p2.y - p1.y == 0:
#        quality = abs(p2.x - p1.x) * 50
#        x = np.linspace(p1.x,p2.x,quality)
#        r = x.shape
#        y = np.full((r[0],1),p1.y)
#        return x,y
#    m = (p2.y - p1.y)/(p2.x-p1.x)
#    b = p1.y  - m*p1.x
#    quality = abs(p1.x - p2.x) * 50
#    x = np.linspace(p1.x,p2.x,quality)
#    y = m * x + b
#    return x,y

#hound_for_plt,hippo_for_plt = RRT()

#def plt_my_path(path,radius):
#   for i in range(len(path) - 1):
#        cx, cy = circle(path[i][0],path[i][1],radius)
#       lx, ly = line(cg.Point(path[i][0],path[i][1]),cg.Point(path[i+1][0],path[i+1][1]))
#      ax.plot(cx,cy)
#     ax.plot(lx,ly)

#    gx, gy = circle(path[len(path)-1][0],path[len(path)-1][1],radius)
#   ax.plot(gx,gy)

#plt_my_path(hound_for_plt,hound_radius)
#plt_my_path(hippo_for_plt,hippo_radius)

#if hound_for_plt[1][0] - hound_for_plt[0][0] == 0:
#    anim_quality = abs(hound_for_plt[0][1] - hound_for_plt[1][1]) * 50
#elif hound_for_plt[1][1] - hound_for_plt[0][1] == 0:
#    anim_quality = abs(hound_for_plt[0][0] - hound_for_plt[1][0]) * 50
#else:
#    anim_quality = abs(hound_for_plt[0][0] - hound_for_plt[1][0])*50

#index_i = 0

#def animate(i):
#    real_i = i * 0.2
#    if i == anim_quality:
#        global  index_i = index_i + 1

#    inc_x = i
#    x, y = circle(hound_for_plt[index_i][0] + ,hound_for_plt[index_i][1] + ,hound_radius)

#anim = FuncAnimation(fig, animate, init_func=init,frames=simulation_step,interval=speed)
#plt.show()
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

# Enter here exit cleanup code.
