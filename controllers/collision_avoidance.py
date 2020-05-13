#!/usr/bin/python3
import threading

#three types of conflict nodes, A,B,C
#Type A: attemp_to_go_conflict_node (Crossing path) (list)
#Type B: driving conflict node (list)
#Type C: arriving conflict node (list)
#trajectory has a form of x,y,delta,parent,A,B,C

#All robot id list, now we have two robots. Hound's ID is 2. Hippo's ID is 1
all_robot_id = [1,2]
Listen = 0
ExecuteProtocol = 1
deadlock_typeA = 'A'
deadlock_typeB = 'B'

class Collision_avoidance:
    def __init__(self, my_id, other_id, inital_state = 0, reached = False):
        self.my_id, self.other_id, self.current_state, self.reached = my_id, other_id, inital_state, reached
        self.trajectory = []
        self.backoff_index_cache = -1


    def set_state(self, node):
        self.current_state = node

    def get_state(self):
        return self.current_state

        #This function should called to set trajectory that was obtained from Hawk
    def set_trajectory(trajectory):
        self.trajectory = trajectory

        #senderID is the id of the sender, index is the node i need to get to that place
    def backoff_handler(self, senderID, index):
        #filter out many backoff command request from other robot.
        if self.backoff_index_cache == -1:
            self.backoff_index_cache = index
            self.goto_target(index)
            if (self.reached_node()) {
                self.set_state(index)
                self.backoff_index_cache = -1 #reset cache to allow new backoff command
            }

        #i is the id of the robot to send to
        #This is a function to handle  type 1 request from other robot. It replies back information neccessary for the sender robot to resolve collision
    def reply_type_1(self,i):
        #reply its current state
        current_state = self.get_state()
        if current_state == (len(self.trajectory) - 1):
            #if the current_state is the goal state
            reply = [self.trajectory[current_state][0],self.trajectory[current_state][1],self.trajectory[current_state][0],self.trajectory[current_state][1]]
        else:
            current_x = trajectory[current_state][0]
            current_y = trajectory[current_state][1]
            next_x = trajectory[current_state+1][0]
            next_y = trajectory[current_state+1][1]
            reply = [current_x,current_y,next_x,next_y]
        reply.append(self.my_id)
        return reply

        #This a function to handler type 2 request from other robot. It replies back information neccessary for the sender robot to resolve deadlock
    def reply_type_2(self,i):
        #reply its current state
        current_state = self.get_state()
        if current_state == (len(self.trajectory) - 1):
            #if the current_state is the goal state
            reply = [self.trajectory[current_state][0],self.trajectory[current_state][0],self.trajectory[current_state][0],self.trajectory[current_state][0]]
        else:
            current_x, current_y, next_x, next_y = self.trajectory[current_state][0], self.trajectory[current_state][1], self.trajectory[current_state+1][0], self.trajectory[current_state+1][1]

        type_A_conflict, type_B_conflict, type_C_conflict = trajectory[current_state][4], trajectory[current_state][5], trajectory[current_state][6]
        reply.extend([type_A_conflict,type_B_conflict,type_C_conflict,self.my_id, self.get_state()])
        return reply


    #This is a general handler for request from other robot. It forwards to type 1 handler or tpyp 2 handler based on its type
    def reply(myrobot_id,i,type):
        #packing the four variables
        #send it to the other robot
        if type == 1:
            return reply_type_1(myrobot_id,i)
        else:
            return reply_type_2(myrobot_id,i)

    #send quary to another robot, three type of query: asking for reply type 1, asking for reply type 2, command to backoff
    #I/O blocking call until it gets back the reply back, except the command to backoff
    #send
    def send(self,robotid,i,type):
        if type == query_type1:

        r2_info = []
        return r2_info

        #drive function that takes the robot to x,y,delta. Once it reached the state (x,y,delta), it sets the reached flag to true
    def drive_function(self,x,y,delta):
        return 1

        #A wrapper function for drive function
    def goto_target(self,i):
        reached = False
        #drive to the target node in the trajectory
        drive_function(trajectory[i][0],trajectory[i][1],trajectory[i][2])
        return

        #This function determines if a robot's current_node causes a type A conflict to the owner of collision_packs
    def is_typeA_conflict(self,collision_packs,current_node):
        for i in collision_packs:
            if i == current_node:
                return True
        return False

    #check againts its current node and next node
    #This function determine if a robot's current node and its next node causes a type B to the owner of collision_packs.
    #This function can also use to check for type C collision too
    def is_typeBorC_conflict(self,collision_packs,current_node,next_node):
        for i in collision_packs:
            if i == current_node or i == next_node:
                return True
        return False

    #This function returns how many collisions in type A, type B, type C the robot has while it's at the state it stations
    def has_conflict(self, index_node):
        len_A, len_B, len_C = len(self.trajectory[index_node][4]), len(self.trajectory[index_node][5]), len(self.trajectory[index_node[6])
        return len_A,len_B,len_C

    def reached_state(self):
        return self.reached

    #need type 2 reply
    #The function returns true if a deadlock occurs
    def deadlock(self,i,R2_info):
        my_current_node, my_A_pack, my_B_pack, my_C_pack = [self.trajectory[i][0], self.trajectory[i][1]], self.trajectory[i][4], self.trajectory[i][5], self.trajectory[i][6]
        if i == (len(self.trajectory) - 1):
            #if the current node is the goal node, next node is equal to the current node.
            my_next_node = [self.trajectory[i][0],sefl.trajectory[i][1]]
        else:
            my_next_node = [self.trajectory[i+1][0],self.trajectory[i+1][1]]

        R2_current_node, R2_next_node, R2_A_pack, R2_B_pack, R2_C_pack, R2_id =[R2_info[0],R2_info[1]], [R2_info[2],R2_info[3]], R2_info[4], R2_info[5], R2_info[6], R2_info[7]

        if (is_typeBorC_conflict(my_B_pack,R2_current_node,R2_next_node) or is_typeBorC_conflict(my_C_pack,R2_current_node,R2_next_node)) and \
           (is_typeB_conflict(R2_B_pack,my_current_node,my_next_node) or is_typeC_conflict(R2_C_pack,my_current_node,my_next_node)):
            return True

        if is_typeA_conflict(my_A_pack,R2_current_node) and (is_typeB_conflict(my_B_pack,R2_current_node, R2_next_node) or is_typeC_conflict(my_C_pack,R2_current_node,R2_next_node)):
            return True
        return False

    def deadlock_resolution(self,R2_info):
        R2_id = R2_info[7]
            if get_my_id() > R2_id:
                #send command to R2 to back off, since my ID is greater. R2_info[8] - 1 is the parent node of the other robot
                send("backoff", self.my_id, R2_info[8] - 1)
        #else then waiting for the other robot to send command to back off
        return

    #This function is to run in parallel with the execute main. It keeps listenning for sending from other robots and invoke handlers coorespondingly
    #Using lock to lock some shared variables between the two
    def listen():
        return

    def execute_main(self):
        set_trajectory([]) #setting the trajectory obtained from Hawk
        length = len(trajectory)
        while(true):
            i = get_state()
            if i != length:
                len_A,len_B,len_C = has_conflict(i)
                if len_A == 0 and len_B == 0 and len_C == 0:
                    #there is no collision, go ahead and drive to the next node
                    goto_target(i+1)
                    if reached_node():
                        set_state(i+1) #update the state of the robot to the next state
                else:
                    #There is a conflict, so send query to ask where its state and its next state,conflict A, B, C
                    #R2 is the destined robot ID, R0 is my ID
                    #Asking for type 2 info. type 2 info is for deadlock resolution
                    R2_info = send(R2,R0,2)
                    #check deadlock before forward to check conflict
                    if deadlock(i,R2_info):
                        while deadlock(i,R2_info):
                            deadlock_resolution(R2_info) #handling the deadlock resolution
                            R2_info = send(R2,R0,2) #keeping querying infor about robot 2
                            i = get_state() #since i can chagne here because of the command from other robot for me to move
                    else:
                        #not a deadlock situation
                        R2_info = send(R2,R0,1)
                        while is_typeBorC_conflict(trajectory[i][5],R2_info) or is_typeBorC_conflict(trajectory[i][6], R2_info):
                            #if there is a type B and type C conflict, then wait until robot 2 is out of conflict zone
                            R2_info = send(R2,R0,1)
                            i = get_state()
                        if is_typeA_conflict(trajectory[get_state()][4],R2_info):
                            #type A and type B or C occurs together, then it's deadlock solved by above deadlock resolution
                            if R2_info[7] > myid:
                                #if the other robot's id is bigger, then let the other robot go and i wait.
                                while is_typeA_conflict(trajectory[i][4],R2_info):
                                    #while the other robot is still transitioning
                                    R2_info = send(R2,R0,1)
                                    i = get_state()
                                #Time for me to move
                                goto_target(i+1)
                                if reached_node():
                                    set_state(i+1)
                            else:
                                #myid is bigger i have priority to go
                                goto_target(i)
                                if reached_node():
                                    set_state(i+1)
    def run(self):
        t1 = threading.Thread(target=self.execute_main)
        t2 = threading.Thread(target=self.listen)
        t1.start()
        t2.start()
        t1.join()
        t2.join()

#get the trajectory from hawk through receiver


if __name__ == '__main__':
    thread_lock = threading.Lock()
    robot = Collision_avoidance(all_robot_id[0], all_robot_id[1])
    robot.run()
