import sys

import matplotlib.pyplot as plt
import RRT
import obstaclefinder
import math
import numpy as np
robotradius = 25
scale = 5      #set scale-up to increase obstacle accuracy


def runRRT(dynamics, robotSize, data ,start = [0,50],end = [1950,1650]):
    #set scale-up to increase obstacle accuracy
    data = data[::-1,:]
    plt.imshow(data,interpolation='nearest') #show 2D representation of map
    img = obstaclefinder.imgToObs()  # create img from imported picture
    obs = img.obsSpaceGen(robotSize, data)

    #initialize RRT
    r = RRT.rrt(N = 5000,obstacles = obs.T, obstacletype = 'array', maxcoords = obs.shape,
            origin = start+[0,0,'',0],goal = end+[0], live = False, divis = 10)
    #Perform RRT
    trajectory = r.rrt( dynamics,plotting = True,verbose = True)
    #print trajectory
    path = []
    for x in trajectory:
        for item in x[3]:
            path.append(item)
    return path




if __name__ == "__main__":
    img = obstaclefinder.imgToObs(
        imagepath="C:/Users/Cooper/PycharmProjects/ECE183DA/lab3/maze.bmp")  # create img from imported picture
    [image, obsimg] = img.obsfind(scale,
                                  robotradius / scale)  # takes scale up factor and obstacle expansion factor and produces display array and obstacle array
    plt.imshow(image.T, interpolation='nearest')  # show 2D representation of map
    origin = [35 * scale, 215 * scale, 0, 0, '', 0]
    # initialize RRT
    r = RRT.rrt(N=5000, obstacles=obsimg, obstacletype='array', maxcoords=image.shape,
                origin=origin, goal=[125 * scale, 30 * scale, math.pi / 2], live=False, divis=5, scale=scale)
    # print('ORIGIN',r.origin)
    # test = r.origin
    # Perform RRT
    trajectory = r.rrt('HAWK', verbose=True, plotting=True)
    # print('ORIGIN AFTER',r.origin,'ORIGIN BEFORE', test)
    # print trajectory
    # print(trajectory)
    # print input list
    path = []
    for x in trajectory:
        for item in x[3]:
            path.append(item)
    print(path)
    # print(r.outputList)

