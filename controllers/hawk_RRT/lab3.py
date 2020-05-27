import sys

import matplotlib.pyplot as plt
import RRTM
import obstaclefinder
import math
import numpy as np

def runRRT(dynamics, robotSize, data ,start = [0,50],end = [1950,1650]):
    data = data[::-1, :]
    plt.imshow(data.T, interpolation='nearest')  # show 2D representation of map
    img = obstaclefinder.imgToObs()  # create img from imported picture
    if dynamics == 'HOUND':
        scaleFactor = 3
    elif dynamics == 'HIPPO':
        scaleFactor = 12
    obs = img.obsSpaceGen(robotSize, data, scaleFactor, debug=False)
    plt.imshow(data,interpolation='nearest') #show 2D representation of map
    #initialize RRT
    print(obs[0].shape)
    r = RRTM.rrt(N = 10000,obstacles = obs.T, obstacletype = 'array', maxcoords = obs[0].shape,
            origin = start+[math.pi/2,0,'',0],goal = end+[0], live = False, divis = 10)
    #Perform RRT
    trajectory = r.rrt(dynamics,plotting=True)
    #print trajectory
    return trajectory

#(verbose = True, plotting=True, dynamics)