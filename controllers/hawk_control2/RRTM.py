import math
import random
import sys

import matplotlib.pyplot as plt
import numpy as np
class rrt():


    def __init__(self, origin = [250, 0, 0, 0,'',0], maxcoords = [500,500], stepsize = 5, N = 10000, obstacles = [[0, 0, 100, 500], [400, 0, 500, 500], [200, 300, 400, 325],
                     [100, 350, 250, 375]], goal = [140, 400, 150, 410], obstacletype = 'vertex', live = False, divis = 1,pixPerM = 10):
        self.origin = origin  # Origin point, in form x,y,parent
        self.maxcoords = maxcoords  # Max values of field. x,y form. Assumes bottom left is 0,0
        self.N = N  # Iterations to run
        self.obstacles = obstacles  # Configuration space, including obstacles. In the form of an array with indeces representing x and y coordinates
        # scaled up by the scale variable, in order to make the system sufficiently continuous. Rectangles only, in form xmin, ymin, xmax, ymax
        self.goal = goal  # goal. Rectangles only, in form xmin, ymin, xmax, ymax
        self.nodesList = [origin]  # list of all nodes
        self.obstacletype = obstacletype    #Whether we're using the vertex obstacle type (manual entry) or the array type (import from obstacleFinder)
        self.live = live    #Whether we're using the "live" plotter or the end-time plotter
        self.divis = divis #draw every divis changes
        self.sweetener = 100 #determines how often to push the tree towards the goal state. EG a value of 100 means it sets the random point to the goal
        self.scaler = pixPerM
        #every 100 iterations
        if self.live:   #Turns on interactive plotting if in live mode
            plt.ion()


    def eucldist(self,node1, node2):  # returns the euclidian distance between two nodes
        dist = math.sqrt(pow(node1[0] - node2[0], 2) + pow(node1[1] - node2[1], 2))
        return dist

    def finddist(self,node1, node2, typec):  # returns the euclidian distance between two nodes
        if typec == 'HOUND':
            Vmax = .5 * self.scaler #Max distance for one second movement
            delta_max = 3.039003906 #max radians for one second rotation
            theta_path = math.atan2(node2[1] - node1[1], node2[0] - node1[0])
            theta_diff1 = theta_path - node1[2]
            if theta_diff1 > math.pi:
                theta_diff1 = theta_diff1 - 2*math.pi
            elif theta_diff1 <= -math.pi:
                theta_diff1 = theta_diff1 + 2*math.pi
                
            if node1[1] == node2[1] and node1[0] == node2[0]:
                theta_path = node2[2]
                
            theta_diff2 = node2[2] - theta_path
            if theta_diff2 > math.pi:
                theta_diff2 = theta_diff2 - 2*math.pi
            elif theta_diff2 <= -math.pi:
                theta_diff2 = theta_diff2 + 2*math.pi
        else:
            Vmax = .4 * self.scaler#Max distance for one second movement
            delta_max = 0.596807115 #max radians for one second rotation
            theta_diff1 = 0 # Hippo has no theta one to cover so its zero
            theta_diff2 = node2[2] - node1[2]
            if theta_diff2 > math.pi:
                theta_diff2 = theta_diff2 - 2*math.pi
            elif theta_diff2 <= -math.pi:
                theta_diff2 = theta_diff2 + 2*math.pi

        return (self.eucldist(node1, node2) / Vmax) + (theta_diff1/delta_max + theta_diff2/delta_max)


    def randomPoint(self):  # generates a random point. Uses a larger space than the actual Configuration space in order to increase steps towards the outside of the space
        point = [random.uniform(0, self.maxcoords[0]), random.uniform(0, self.maxcoords[1]),random.uniform(0,2*math.pi),0,'',0]
        return point

    def obsCheck(self, point,
                 obstacles):  # checks if the point is inside an obstacle. if it is, returns the origin. ##FUTURE## return the closest allowed point
        if self.obstacletype == 'vertex':
            for o in obstacles:
                if (((o[0] < point[0] + self.robotRadius < o[2]) or (o[0] < point[0] - self.robotRadius < o[2])) and (
                        (o[1] < point[1] + self.robotRadius < o[3]) or (o[1] < point[1] - self.robotRadius < o[3]))):
                    return True
        elif self.obstacletype == 'array':  # array type obstacles not currently using robot radius
            xflr = math.floor(
                point[0]) - 1  # create floor and ceilinged variables to make sure we cover all possible cases.
            yflr = math.floor(
                point[1]) - 1  # This is because we need to reference the obstacles array, which needs discrete indeces.
            xcl = math.ceil(point[0]) - 1
            ycl = math.ceil(point[1]) - 1
            xmax = obstacles.shape[0] - 1
            ymax = obstacles.shape[1] - 1

            theta = point[2] * 180 / (2 * math.pi)
            theta = round(theta / 5)
            theta = theta % 36

            if xflr >= xmax or xcl >= xmax:  # make sure bounds are not violated
                xflr = xcl = xmax
            if yflr >= ymax or ycl >= ymax:
                yflr = ycl = ymax

            if (obstacles[xflr][yflr][theta] or obstacles[xflr][ycl][theta] or obstacles[xcl][yflr][theta] or
                    obstacles[xcl][ycl][
                        theta]):  # if the rounded location (via any rounding scheme) is a wall (True in the obstacle array), say so
                return True
        return False



    def pathClear(self, startnode, endnode, obs):   #determines if a path is clear using obsCheck. Does this by canvassing a rectangle with the two input points in opposite corners.
        deadzone = 1    #deadzone because axis perpindicular paths break everything for some reason
        if self.obsCheck(endnode,obs):  #Don't bother if the endpoint is not allowed
            return True
        diffx = (endnode[0] - startnode[0])
        diffy = (endnode[1] - startnode[1])
        if diffx > 0 + deadzone:        #This is in order to use range(), which won't decrement unless it has a negative step
            stepx = 1                   #includes a deadzone to prevent issues with going perpindicular to axis (not sure why this is an issue, but this solves it)
        elif diffx < 0 - deadzone:
            stepx = -1
        else:
            stepx = 0

        if diffy > 0 + deadzone:
            stepy = 1
        elif diffy < 0 - deadzone:
            stepy = -1
        else:
            stepy = 0
                        #Don't allow perfectly perpindicular to axis motion, because of issues mentioned previously
        if stepx==0 and stepy==0:
            return False

        if stepx == 0:
            return True

        if stepy == 0:
            return True

        for x in range(round(startnode[0]), round((startnode[0] + diffx)), stepx):          #check every node in a rectangle with startnode and endnode as opposite corners to verify path
            for y in range(round(startnode[1]), round((startnode[1] + diffy )), stepy):
                if self.obsCheck([x,y],obs):
                    return True
        return False

    def takestepHOUND(self,startnode, targetnode, nodes):  # finds a point one unit step from startnode, in the direction of targetnode. Takes "node" in order to set new node's parent node in node[2]
        dt = 0.008        #Timestep in seconds
        endtime = 1.6     #endtime in seconds
        instructionVector = []  #vector for instructions
        # Full spin and Full velocity
        theta_dot = 3.039003906
        V = .5 * self.scaler
        currentPos = [startnode[0],startnode[1],startnode[2],self.nodesList.index(startnode),'',0]  #initialize currentPosition to startPosition
        prevPos = currentPos    #initialize prevPos to currentPos
        theta_path = math.atan2(targetnode[1] - startnode[1], targetnode[0] - startnode[0]) #calculate theta_path for the given two points

        xvel = V*math.cos(theta_path)   #calculate X and Z components of velocity
        zvel = V*math.sin(theta_path)
        
        # Hound Movement come in three stages, rotate, move, rotate
        stage1 = True
        stage2 = False
        stage3 = False
        
        # Check if some stages can be bypassed
        if abs(currentPos[2] - theta_path) < theta_dot * dt or abs(currentPos[2] + math.pi - theta_path) < theta_dot * dt or abs(currentPos[2] - math.pi - theta_path) < theta_dot * dt:
            stage1 = False
            stage2 = True
            if self.eucldist(currentPos,targetnode) < V*dt:
                stage2 = False
                stage3 = True
              
        # Counter keeps track of number of timesteps for a given input  
        counter = 0
        
        # Theta diff is to find out whether you should turn left or right to align theta for the path
        theta_diff = theta_path - startnode[2]
        if theta_diff > math.pi:
            theta_diff = theta_diff - 2*math.pi
        elif theta_diff <= -math.pi:
            theta_diff = theta_diff + 2*math.pi
        path_right = True
        if theta_diff < 0:
            path_right = False    
        
        # Final diff is to find out whether you should turn left or right to align theta for the final spot
        final_diff = targetnode[2] - theta_path
        if final_diff > math.pi:
            final_diff = final_diff - 2*math.pi
        elif theta_diff <= -math.pi:
            final_diff = final_diff + 2*math.pi 
        final_right = True
        if final_diff < 0:
            final_right = False

        # Incremental step
        for t in range(0,int(endtime/dt)):  #for each time step
            if self.obsCheck(currentPos, self.obstacles):   #catch if currently in an obstacle
                return prevPos      #Invalidate entire path and return start node
                
            # Stage one aligns theta for path
            # If theta is aligned for path, append instruction and reset variables
            # Else take a step
            if stage1:
                if abs(currentPos[2] - theta_path) < theta_dot * dt:
                    currentPos[2] = theta_path
                    stage2 = True
                    stage1 = False
                    t = t - 1 # This was a decision and not a step so draw t back by one
                    if path_right:
                        instructionVector.append([4, -4, counter])
                    else:
                        instructionVector.append([-4, 4, counter])
                    counter = 0
                    instructionVector.append([0, 0, 25])
                else:
                    counter = counter + 1
                    if path_right:
                        currentPos[2] = currentPos[2] + theta_dot * dt
                    else:
                        currentPos[2] = currentPos[2] - theta_dot * dt
            
            # Stage two moves along path           
            elif stage2:
                if self.eucldist(currentPos,targetnode) < V*dt:
                    currentPos[0] = targetnode[0]
                    currentPos[1] = targetnode[1]
                    stage3 = True
                    stage2 = False
                    t = t - 1
                    instructionVector.append([20, 20, counter])
                    counter = 0
                    instructionVector.append([0, 0, 15])
                else:
                    counter = counter + 1
                    currentPos[0] = currentPos[0] + xvel * dt
                    currentPos[1] = currentPos[1] + zvel * dt
               
            # Stage three aligns theta for final spot     
            elif stage3:
                if abs(currentPos[2]-targetnode[2]) < theta_dot*dt:
                    currentPos[2] = targetnode[2]
                    break
                else:
                    counter = counter + 1
                    if final_right:
                        currentPos[2] = currentPos[2] + theta_dot * dt
                    else:
                        currentPos[2] = currentPos[2] - theta_dot * dt
        
        # Gotta check that last instruction                
        if self.obsCheck(currentPos, self.obstacles):    #catch if currently in an obstacle
            return prevPos                               #Invalidate all if not good
        
        # This if ladder catches final instructions since loop will terminate mid instruction
        if stage3:
            if final_right:
                instructionVector.append([4, -4, counter])
            else:
                instructionVector.append([-4, 4, counter])
            instructionVector.append([0, 0, 25])
        elif stage2:
            instructionVector.append([20, 20, counter])
            instructionVector.append([0, 0, 15])
        elif stage1:
            if path_right:
                instructionVector.append([4, -4, counter])
            else:
                instructionVector.append([-4, 4, counter])
            instructionVector.append([0, 0, 25])

        # Setup for return and return
        currentPos[4] = instructionVector
        return currentPos

    def takestepHIPPO(self,startnode, targetnode, nodes):  # finds a point one unit step from startnode, in the direction of targetnode. Takes "node" in order to set new node's parent node in node[2]
        dt = 0.008        #Timestep in seconds
        endtime = 1.6     #endtime in seconds
        instructionVector = []  #vector for instructions
        # Full spin and Full velocity
        theta_dot = 0.596807115
        V = .4 * self.scaler
        currentPos = [startnode[0],startnode[1],startnode[2],self.nodesList.index(startnode),'',0]  #initialize currentPosition to startPosition
        prevPos = currentPos    #initialize prevPos to currentPos
        theta_path = math.atan2(targetnode[1] - startnode[1], targetnode[0] - startnode[0]) #calculate theta_path for the given two points

        xvel = V*math.cos(theta_path)   #calculate X and Z components of velocity
        zvel = V*math.sin(theta_path)
        
        # Hippo Movement come in two stages move then rotate
        stage1 = True
        stage2 = False
        
        # Check if some stages can be bypassed
        if self.eucldist(currentPos,targetnode) < V*dt:
            stage1 = False
            stage2 = True
              
        # Counter keeps track of number of timesteps for a given input  
        counter = 0
        
        # Theta diff is to find out inputs for wheels
        theta_diff = theta_path - startnode[2]
        if theta_diff > math.pi:
            theta_diff = theta_diff - 2*math.pi
        elif theta_diff <= -math.pi:
            theta_diff = theta_diff + 2*math.pi
        Vx = V*math.cos(theta_diff)
        Vz = V*math.sin(theta_diff)
        wheels14 = Vx - Vz
        wheels23 = Vx + Vz   
        
        # Final diff is to find out whether you should turn left or right to align theta for the final spot
        final_diff = targetnode[2] - startnode[2]
        if final_diff > math.pi:
            final_diff = final_diff - 2*math.pi
        elif theta_diff <= -math.pi:
            final_diff = final_diff + 2*math.pi 
        final_right = True
        if final_diff < 0:
            final_right = False

        # Incremental step
        for t in range(0,int(endtime/dt)):  #for each time step
            if self.obsCheck(currentPos, self.obstacles):   #catch if currently in an obstacle
                return prevPos      #Invalidate entire path and return start node
            
            # Stage two moves along path           
            elif stage1:
                if self.eucldist(currentPos,targetnode) < V*dt:
                    currentPos[0] = targetnode[0]
                    currentPos[1] = targetnode[1]
                    stage2 = True
                    stage1 = False
                    t = t - 1
                    instructionVector.append([wheels14, wheels23, wheels23, wheels14, counter])
                    counter = 0
                    instructionVector.append([0, 0, 0, 0, 5])
                else:
                    counter = counter + 1
                    currentPos[0] = currentPos[0] + xvel * dt
                    currentPos[1] = currentPos[1] + zvel * dt
               
            # Stage three aligns theta for final spot     
            elif stage2:
                if abs(currentPos[2]-targetnode[2]) < theta_dot*dt:
                    currentPos[2] = targetnode[2]
                    break
                else:
                    counter = counter + 1
                    if final_right:
                        currentPos[2] = currentPos[2] + theta_dot * dt
                    else:
                        currentPos[2] = currentPos[2] - theta_dot * dt
        
        # Gotta check that last instruction                
        if self.obsCheck(currentPos, self.obstacles):    #catch if currently in an obstacle
            return prevPos                               #Invalidate all if not good
        
        # This if ladder catches final instructions since loop will terminate mid instruction
        if stage2:
            if final_right:
                instructionVector.append([14, -14, 14, -14, counter])
            else:
                instructionVector.append([-14, 14, -14, 14, counter])
            instructionVector.append([0, 0, 0, 0, 5])
        elif stage1:
            instructionVector.append([wheels14, wheels23, wheels23, wheels14, counter])
            instructionVector.append([0, 0, 0, 0, 5])

        # Setup for return and return
        currentPos[4] = instructionVector
        return currentPos

    def takestep(self, startnode, targetnode, nodes):  # finds a point one unit step from startnode, in the direction of targetnode. Takes "node" in order to set new node's parent node in node[2]
        # You get one second to move
        time_left = 1

        # Full spin and Full velocity
        theta_dot = 2.979/8
        V = 126.6/2

        # Get angle from start node to end node
        theta_path = math.atan2(targetnode[1] - startnode[1], targetnode[0] - startnode[0])
        if startnode[1] == targetnode[1] and startnode[0] == targetnode[0]:
            theta_path = targetnode[2]
            print(startnode[2],targetnode[2])

        if theta_path < 0:
            theta_path = theta_path + 2*math.pi

        # Difference between start angle and path angle
        theta_diff_1 = abs(theta_path-startnode[2])
        if theta_diff_1 > math.pi:
            theta_diff_1 = abs(theta_diff_1-2*math.pi)
        if (startnode[2] - theta_diff_1) % (2*math.pi) == theta_path:
            theta_diff_1 = 0 - theta_diff_1
        time_turn_1 = abs(theta_diff_1)/theta_dot

        # Use up time to turn that way
        if time_turn_1 > time_left:
            time_turn_1 = time_left
        time_left = time_left - time_turn_1

        # Euclidian distance of path from start to target
        euclid = self.eucldist(startnode,targetnode)

        # Use up time to move along path
        time_to_move = euclid/V
        if time_to_move > time_left:
            time_to_move = time_left
        time_left = time_left - time_to_move

        # Difference bewteen path angle and ending angle
        theta_diff_2 = abs(theta_path-targetnode[2])
        if theta_diff_2 > math.pi:
            theta_diff_2 = abs(theta_diff_2-2*math.pi)
        if (theta_path - theta_diff_2) % (2*math.pi) == targetnode[2]:
            theta_diff_2 = 0 - theta_diff_2

        # Use up time to turn to ending angle
        time_turn_2 = abs(theta_diff_2)/theta_dot
        if time_turn_2 > time_left:
            time_turn_2 = time_left
        time_left = time_left - time_turn_2

        # Decide whether your turning right or left and adjust angle at max speed
        if theta_diff_1 < 0:
            new_theta = startnode[2] - theta_dot*time_turn_1
        else:
            new_theta = startnode[2] + theta_dot*time_turn_1

        # Move along path at max speed
        newx = startnode[0] + V*math.cos(theta_path)*time_to_move
        newy = startnode[1] + V*math.sin(theta_path)*time_to_move

        # Decide whether your turning right or left and adjust angle at max speed
        if theta_diff_2 < 0:
            new_theta = new_theta - theta_dot * time_turn_2
        else:
            new_theta = new_theta + theta_dot * time_turn_2

        new_theta = new_theta % (2*math.pi)


        string_in = ''
        if time_turn_1 >= .01:
            if theta_diff_1 < 0:
                string_in = string_in + 'PWML=149, PWMR=50, t=' + str(time_turn_1) + '\n'
            if theta_diff_1 > 0:
                string_in = string_in + 'PWML=40, PWMR=149, t=' + str(time_turn_1) + '\n'
        if time_to_move >= .01:
            string_in = string_in + 'PWML=149, PWMR=149, t=' + str(time_to_move) + '\n'
        if time_turn_2 >= .01:
            if theta_diff_2 < 0:
                string_in = string_in + 'PWML=149, PWMR=50, t=' + str(time_turn_2) + '\n'
            if theta_diff_2 > 0:
                string_in = string_in + 'PWML=40, PWMR=149, t=' + str(time_turn_2) + '\n'

        # Set up new node
        newnode = [newx, newy, new_theta, nodes.index(startnode), string_in,0]
        newnode[5] = startnode[5] + self.finddist(startnode, newnode)


        if self.pathClear(startnode, newnode, self.obstacles):
            checkednode = self.origin
        else:
            checkednode = newnode
        return checkednode


    def findclosest(self,nodes, newnode, typec):  # finds the closest node to newnode in nodelist nodes
        distances = []
        for i in nodes:
            distances.append(self.finddist(i,newnode, typec))

        return nodes[distances.index(min(distances))]


    def checkgoal(self,nodelist, node, goal):  # checks if the point is within the goal. if not, it sets goalfound to false. if it is, it returns the node path it took and sets goalfound to false
        goalpath = []
        tracenode = []

        if (node[0] == goal[0] and node[1] == goal[1] and node[2] == goal[2]): #if the node is in the correct range
            goalfound = True        #set goal flag
            tracenode = node        #stores "winning" node in tracenode
            goalpath.append(tracenode)  #adds tracenode to the goal trajectory
            while (tracenode[3] != 0):  #while we havent yet hit the origin
                tracetemp = nodelist[tracenode[3]]  #get the parent node of tracenode
                goalpath.append(tracetemp)      #append parent to trajectory
                tracenode = tracetemp       #set the parent node to the current node and repeat
            goalpath.append(self.origin)  # since the loop breaks once we hit the origin, add the origin to the end
        else:
            goalfound = False

        return [goalfound, goalpath]

    def initplot(self, goal, obstacles):  # initializes plot by drawing obstacles, goal, and origin as well as setting the axes
        if self.obstacletype == 'vertex':
            for o in obstacles:
                obsbox = [[o[0], o[2], o[2], o[0], o[0]], [o[1], o[1], o[3], o[3], o[1]]]
                plt.plot(obsbox[0], obsbox[1], 'r')
        plt.scatter(goal[0], goal[1],s=25, c='g')
        plt.scatter(self.origin[0], self.origin[1], s=25, c='r')
        plt.xlim(0, self.maxcoords[0])
        plt.ylim(0, self.maxcoords[1])


    def drawparentlines(self, nodelist):    #draws connecting lines between parent and child nodes
        filterlist = [self.origin]
        print(self.origin)
        for n in nodelist:
            if n[3]==-1:
                pass
            else:
                filterlist.append(n)
        for node in filterlist:             #plot each "jump" from child node to parent node
            if(node == nodelist[node[3]]):
                pass
            else:
                jumplist = [node, nodelist[node[3]]]
                plt.plot([jumplist[0][0],jumplist[1][0]],[jumplist[0][1],jumplist[1][1]],'b')
                if self.live and nodelist.index(node) % self.divis == 0:   #Draw these consecutively if in live mode
                    plt.draw()
                    plt.pause(0.0001)

    def getOrigin(self):
        return self.origin
    def rrt(self,dynamics,verbose = False, plotting = False):   #Main implementation of RRT
        xg=[]
        yg=[]
        print('Here?')
        self.initplot(self.goal, self.obstacles)    #initialize plot
        print('Maybe Here?')

        for k in range(0, self.N):      #create (or attempt to create) N nodes
            if k % self.sweetener != 0:
                xrand = self.randomPoint()  #choose a random point
            else:
                xrand = self.goal
            print('maybeeee Here?')
            xnear = self.findclosest(self.nodesList, xrand, dynamics)     #find the nearest node to the random point
            #xnear = self.findclosestOPT(self.nodesList, xrand)
            if dynamics == 'HOUND':
                xnew = self.takestepHOUND(xnear, xrand, self.nodesList)  #take one step towards the random point from the nearest node and create a new node
            else:
                xnew = self.takestepHIPPO(xnear, xrand,
                                          self.nodesList)  # take one step towards the random point from the nearest node and create a new node
            #xnew = self.optimize(self.nodesList,xnew)
            [goalbool, goalpath] = self.checkgoal(self.nodesList, xnew, self.goal)  #check the new node to see if it's in the goal zone
            if (goalbool):
                [xg, yg, tg, zg, sg,cg] = list(zip(*goalpath))     #If it does succeed, stop creating new nodes and return the goal path
                print('PATH FOUND')
                break
            else:   #otherwise, just add it to the list and continue
                self.nodesList.append(xnew)
            if verbose == True: #debug info, only dump if verbose
                print(k)
        print('where do we go now')
        if plotting == True:# Plot if that's enabled

            self.drawparentlines(self.nodesList)

            if goalbool:
                plt.plot(xg, yg, 'y')
            if self.live:   #If live, draw the goal to the live graph
                 plt.draw()
                 plt.pause(0.01)
                 plt.ioff()
            plt.show()

            xg = (np.array(xg)).tolist()   #scale trajectory down from increased scale
            yg = (np.array(yg)).tolist()
            trajectory = []
            for i in range(0,len(xg)):
                trajectory.append([xg[i],yg[i],tg[i],sg[i]])

        return trajectory[::-1] #return the trajectory to the goal (reverse it, its in goal -> origin order until this line