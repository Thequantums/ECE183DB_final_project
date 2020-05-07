import sys

import matplotlib.pyplot as plt
import RRT
#import obstaclefinder
import numpy as np

def runRRT(dynamics , scale, data ,start = [0,50],end = [1950,1650]):
    #set scale-up to increase obstacle accuracy
    #img = obstaclefinder.imgToObs() #create img from imported picture
    #[image,obsimg] = img.obsfind(scale,4) #takes scale up factor and obstacle expansion factor and produces display array and obstacle array

    plt.imshow(data.T,interpolation='nearest') #show 2D representation of map

    #initialize RRT
    r = RRT.rrt(N = 5000,obstacles = data, obstacletype = 'array', maxcoords = data.T.shape,
            origin = start+[0,0,'',0],goal = end+[0], live = True, divis = 10, scale = scale, arb = False)
    #Perform RRT
    trajectory = r.rrt(True, True, dynamics)
    #print trajectory
    return trajectory

#(verbose = True, plotting=True, dynamics)
