import math
import random
import sys

import matplotlib.pyplot as plt
import numpy as np
class rrt():


    def __init__(self, origin = [250, 0, 0, 0,'',0], maxcoords = [500,500], stepsize = 5, N = 10000, obstacles = [[0, 0, 100, 500], [400, 0, 500, 500], [200, 300, 400, 325],
                     [100, 350, 250, 375]], goal = [140, 400, 150, 410], obstacletype = 'vertex', live = False, divis = 1,scale = 10,arb = False):
        self.origin = origin  # Origin point, in form x,y,parent
        self.maxcoords = maxcoords  # Max values of field. x,y form. Assumes bottom left is 0,0
        self.N = N  # Iterations to run
        self.obstacles = obstacles  # Configuration space, including obstacles. In the form of an array with indeces representing x and y coordinates
        # scaled up by the scale variable, in order to make the system sufficiently continuous. Rectangles only, in form xmin, ymin, xmax, ymax
        self.goal = goal  # goal. Rectangles only, in form xmin, ymin, xmax, ymax
        self.nodesList = [origin]  # list of all nodes
        self.robotRadius = 20*scale    #Radius of the circular robot estimate
        self.obstacletype = obstacletype    #Whether we're using the vertex obstacle type (manual entry) or the array type (import from obstacleFinder)
        self.live = live    #Whether we're using the "live" plotter or the end-time plotter
        self.divis = divis #draw every divis changes
        self.scale = scale  #scale-up constant for graph
        self.sweetener = 50 #determines how often to push the tree towards the goal state. EG a value of 100 means it sets the random point to the goal
        #every 100 iterations
        self.isArbitrary = arb    #picks a random start and end point
        if self.live:   #Turns on interactive plotting if in live mode
            plt.ion()

    def eucldist(self,node1, node2):  # returns the euclidian distance between two nodes
        dist = math.sqrt(pow(node1[0] - node2[0], 2) + pow(node1[1] - node2[1], 2))
        return dist


    def finddist(self,node1, node2):  # returns the euclidian distance between two nodes
        Vmax = 126.6 #Max distance for one second movement
        delta_max = 2.979 #max radians for one second rotation

        theta_path = math.atan2(node2[1] - node1[1], node2[0] - node1[0])
        if node1[1] == node2[1] and node1[0] == node2[0]:
            theta_path = node2[2]
        if theta_path < 0:
            theta_path = theta_path + 2 * math.pi
        theta_diff_1 = abs(theta_path - node1[2])
        if theta_diff_1 > math.pi:
            theta_diff_1 = abs(theta_diff_1 - 2 * math.pi)
        theta_diff_2 = abs(theta_path - node2[2])
        if theta_diff_2 > math.pi:
            theta_diff_2 = abs(theta_diff_2 - 2 * math.pi)

        return (self.eucldist(node1, node2) / Vmax) + (theta_diff_1/delta_max + theta_diff_2/delta_max)

        # vect1 = [math.acos(node1[0]), math.asin(node1[1])]  # getting units vector for xnear
        # vect2 = [node2[0] - node1[0], node2[1] - node1[1]]  # getting vector for path from xnear to  newnode
        # unit_vect1 = vect1 / np.linalg.norm(vect1)  # make it a unit vector, already a unit vector
        # unit_vect2 = vect2 / np.linalg.norm(vect2)  # make it a unit vector
        # delta_direction_angle = np.arccos(np.dot(unit_vect1, unit_vect2))  # getting angles
        # vect3 = [math.acos(node2[0]), math.asin(node2[1])]  # unit vector for the xnew
        # delta_after_reached = np.acrcos(np.dot(unit_vect2, vect3))
        # dist = (self.eucldist(node1, node2) / Vmax) + (abs(delta_direction_angle) / delta_max + (abs(delta_after_reached) / delta_max))
        # return dist


    def randomPoint(self):  # generates a random point. Uses a larger space than the actual Configuration space in order to increase steps towards the outside of the space
        point = [random.uniform(0, self.maxcoords[0]), random.uniform(0, self.maxcoords[1]),random.uniform(0,2*math.pi),0,'',0]
        return point


    def obsCheck(self, point, obstacles):  # checks if the point is inside an obstacle. if it is, returns the origin. ##FUTURE## return the closest allowed point
        if self.obstacletype == 'vertex':
            for o in obstacles:
                if (((o[0] < point[0] + self.robotRadius < o[2]) or (o[0] < point[0] - self.robotRadius < o[2])) and (
                        (o[1] < point[1] + self.robotRadius < o[3]) or (o[1] < point[1] - self.robotRadius < o[3]))):
                    return True
        elif self.obstacletype == 'array':  #array type obstacles not currently using robot radius
            xflr = math.floor(point[0]) - 1 #create floor and ceilinged variables to make sure we cover all possible cases.
            yflr = math.floor(point[1]) - 1 #This is because we need to reference the obstacles array, which needs discrete indeces.
            xcl = math.ceil(point[0]) - 1
            ycl = math.ceil(point[1]) - 1
            xmax = obstacles.shape[0]-1
            ymax = obstacles.shape[1]-1

            if xflr >= xmax or xcl >= xmax: #make sure bounds are not violated
                xflr = xcl = xmax
            if yflr >= ymax or ycl >= ymax:
                yflr = ycl = ymax

            if(obstacles[xflr][yflr] or obstacles[xflr][ycl] or obstacles[xcl][yflr] or obstacles[xcl][ycl]): #if the rounded location (via any rounding scheme) is a wall (True in the obstacle array), say so
                return True
        return False


    def createArb(self):
        self.origin = self.randomPoint()
        while (self.obsCheck(self.origin, self.obstacles)):
            self.origin = self.randomPoint()
        self.nodesList = [self.origin]


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

        for x in range(round(startnode[0]), round((startnode[0] + diffx+(self.robotRadius*stepx))), stepx):          #check every node in a rectangle with startnode and endnode as opposite corners to verify path
            for y in range(round(startnode[1]), round((startnode[1] + diffy+(self.robotRadius*stepy) )), stepy):
                if self.obsCheck([x,y],obs):
                    return True
        return False


    def takestep(self, startnode, targetnode, nodes,dynamics):  # finds a point one unit step from startnode, in the direction of targetnode. Takes "node" in order to set new node's parent node in node[2]
        if dynamics == 'hippo':
            print('hippo dynamics')
            if self.pathClear(startnode, targetnode, self.obstacles):
                return self.origin
            else:
                #hippo has infinite time to move
                #hippo can go from every x',y',delta' to x, y, delta  if there is no obstacle along the path.
                #but we can restrict how far the target node can be from this start node. This is so we can get many nodes along the path which helps improve dynamic obtacle avoidance
                v = 126.6 # fake value
                time_to_move = 1 #in simulation it will take more than 1 sec because of angle rotation. will have to change that.
                euclid = self.eucldist(startnode,targetnode)

                # limit move time to only 1 second
                time_to_move = euclid/V
                if time_to_move > time_left:
                    time_to_move = time_left

                # Get angle from start node to end node
                theta_path = math.atan2(targetnode[1] - startnode[1], targetnode[0] - startnode[0])
                if startnode[1] == targetnode[1] and startnode[0] == targetnode[0]:
                    theta_path = targetnode[2]

                if theta_path < 0:
                    theta_path = theta_path + 2*math.pi

                newx = startnode[0] + V*math.cos(theta_path)*time_to_move
                newy = startnode[1] + V*math.sin(theta_path)*time_to_move

                newnode = [newx, newy, targetnode[2], nodes.index(startnode), "the drive function will handle", startnode[5] + self.finddist(startnode, newnode)]
                return newnode
        else:
            print('hippo dynamics')
            #put dynamics here
            #print('FAILSAFE')
             
            # You get one second to move
            time_left = 1

            # Full spin and Full velocity
            theta_dot = 2.979
            V = 126.6

            # Get angle from start node to end node
            theta_path = math.atan2(targetnode[1] - startnode[1], targetnode[0] - startnode[0])
            if startnode[1] == targetnode[1] and startnode[0] == targetnode[0]:
                theta_path = targetnode[2]
                #print(startnode[2],targetnode[2])

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

        #dist = self.finddist(startnode, targetnode) #make sure startnode and targetnode are actually different
        #if dist != 0:
        #    newx = ((targetnode[0] - startnode[0]) / dist) * self.stepsize #set new x and y to one unit step, multiplied bu stepsize to expand the step if desired
        #    newy = ((targetnode[1] - startnode[1]) / dist) * self.stepsize
        #    theta = 0## DUMB CODE FIX LATER
        #    newnode = [newx + startnode[0], newy + startnode[1],theta , nodes.index(startnode)]    #sets x and y, as well as storing the parent node
        #    if self.pathClear(startnode, newnode, self.obstacles):  #Check for obstacles in the path, returns the origin as a dummy node if the point is invalid
        #        checkednode = self.origin
        #    else:
        #        checkednode = newnode
        #else:
        #    checkednode = self.origin
        #return checkednode
            
        
       


    def findclosest(self,nodes, newnode):  # finds the closest node to newnode in nodelist nodes
        distances = []
        for i in nodes:
            distances.append(self.finddist(i,newnode))

        return nodes[distances.index(min(distances))]


    def findclosestOPT(self,nodes, newnode):  # finds the closest node to newnode in nodelist nodes
        radius = 2
        distances = []
        for i in nodes:
            distances.append(self.finddist(i,newnode))
        closest = nodes[distances.index(min(distances))]
        cheapest = closest
        for i in nodes:
            if self.finddist(closest, i) < radius:
                if i[5] < cheapest[5]:
                    cheapest = i
        return cheapest


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
        for n in nodelist:  #Filters out repeated points for speed of drawing
            if n[3] == 0:
                pass
            else:
                filterlist.append(n)

        for node in nodelist:             #plot each "jump" from child node to parent node
            if(node == nodelist[node[3]]):
                pass
            else:
                jumplist = [node, nodelist[node[3]]]
                plt.plot([jumplist[0][0],jumplist[1][0]],[jumplist[0][1],jumplist[1][1]],'b')
                if self.live and nodelist.index(node) % self.divis == 0:   #Draw these consecutively if in live mode
                    plt.draw()
                    plt.pause(0.0001)


    def optimize(self,nodes,newnode): #method reserved if we can get RRT* to function
        radius = 2
        for i in nodes:
            if self.finddist(newnode, i) < radius:
                if i[5]+self.finddist(i,newnode) < newnode[5]:
                    newnode[5] = i[5]+self.finddist(i,newnode)
                    newnode[3] = nodes.index(i)
        return newnode

    def rrt(self, verbose = False, plotting = False, dynamics = 'hound'):   #Main implementation of RRT
        xg=[]
        yg=[]
        if (self.isArbitrary):
            self.createArb()
        self.initplot(self.goal, self.obstacles)    #initialize plot


        for k in range(0, self.N):      #create (or attempt to create) N nodes
            if k % self.sweetener != 0:
                xrand = self.randomPoint()  #choose a random point
            else:
                xrand = self.goal

            xnear = self.findclosest(self.nodesList, xrand)     #find the nearest node to the random point
            #xnear = self.findclosestOPT(self.nodesList, xrand)
            xnew = self.takestep(xnear, xrand, self.nodesList,dynamics)  #take one step towards the random point from the nearest node and create a new node
            #xnew = self.optimize(self.nodesList,xnew)
            [goalbool, goalpath] = self.checkgoal(self.nodesList, xnew, self.goal)  #check the new node to see if it's in the goal zone
            if (goalbool):
                [xg, yg, tg, zg, sg,cg] = list(zip(*goalpath))     #If it does succeed, stop creating new nodes and return the goal path
                print('PATH FOUND')
                break
            else:   #otherwise, just add it to the list and continue
                self.nodesList.append(xnew)
            #if verbose == True: #debug info, only dump if verbose
                #print(k)
        if plotting == True:# Plot if that's enabled
            self.drawparentlines(self.nodesList)
            if goalbool:
                plt.plot(xg, yg, 'y')
            if self.live:   #If live, draw the goal to the live graph
                plt.draw()
                plt.pause(0.01)
                plt.ioff()
            plt.show()
            xg = (np.array(xg) / self.scale).tolist()   #scale trajectory down from increased scale
            yg = (np.array(yg) / self.scale).tolist()
            trajectory = []
            for i in range(0,len(xg)):
                trajectory.append([xg[i],yg[i],tg[i],sg[i]])
        #each node in the form of (x,y,delta,parent)
        return trajectory[::-1] #return the trajectory to the goal (reverse it, its in goal -> origin order until this line
