import sys

import matplotlib.pyplot as plt
import RRTM
import obstaclefinder
import numpy as np

def runRRT(dynamics, robotSize, data ,start = [0,50],end = [1950,1650]):
    data = data[::-1, :]
    print('hey')
    plt.imshow(data.T, interpolation='nearest')  # show 2D representation of map
    print('woah')
    img = obstaclefinder.imgToObs()  # create img from imported picture
    print('hey now')
    if dynamics == 'HOUND':
        print('hound')
        scaleFactor = 3
    elif dynamics == 'HIPPO':
        print('hippo')
        scaleFactor = 12
    obs = img.obsSpaceGen(robotSize, data, scaleFactor, debug=False)
    print('wass good bb')
    plt.imshow(data,interpolation='nearest') #show 2D representation of map
    print('can you dig it')
    #initialize RRT
    print(obs[0].shape)
    r = RRTM.rrt(N = 5000,obstacles = obs.T, obstacletype = 'array', maxcoords = obs[0].shape,
            origin = start+[0,0,'',0],goal = end+[0], live = False, divis = 10)
    #Perform RRT
    trajectory = r.rrt(dynamics,plotting = True,verbose=True)
    #print trajectory
    return trajectory

#(verbose = True, plotting=True, dynamics)