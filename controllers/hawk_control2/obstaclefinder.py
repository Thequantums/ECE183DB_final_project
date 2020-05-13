##anku255
from tkinter import Image


import numpy as np
import cv2
import matplotlib.pyplot as plt
import math
from PIL import Image as im

class imgToObs():


    def __init__(self,imagepath = "C:/Users/bobbe/PycharmProjects/ECE183DA/lab3/maze.bmp"):
        self.image = cv2.imread(imagepath)

    def showimage(self):
        plt.imshow( self.image)
        plt.show()
        cv2.waitKey(0)
        cv2.destroyAllWindows()


    def obsfind(self,scale, expand = 6):
        #Grayscale the image and convert to a boolean array (true for obstacle, false for free space)
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        imagearray = np.transpose(np.array(gray))
        imagearray = imagearray < 255

        templine = []
        temparray = []
        #Make a copy of the image to expand
        obsexparray = imagearray.copy()

        # Scale Obstacles to account for robot radius
        expand = math.ceil(expand)
        linei = 0
        pixi = 0
        for line in imagearray:
            for pixel in line:
                if pixel == True:
                    for miniline in range(linei - expand - 1, linei + expand + 1):
                        for minipix in range(pixi - expand - 1, pixi + expand + 1):
                            if 0 < miniline < obsexparray.shape[0] - 1 and 0 < minipix < obsexparray.shape[1] - 1:
                                obsexparray[miniline][minipix] = True
                pixi = pixi + 1
            templine = []
            pixi = 0
            linei = linei + 1

        #For actual obstacle array
        #Scale up array via duplication
        biggerimageOarray = []
        biggerimgOarrayline = []
        for line in obsexparray.tolist():
            for p in line:
                for i in range(0,scale):
                    biggerimgOarrayline.append(p)
            for i in range(0, scale):
                biggerimageOarray.append(biggerimgOarrayline)
            biggerimgOarrayline = []
        #return Larger true/false array
        obarray = np.array(biggerimageOarray)

        #For display array (original, needs to be scaled the same for plot to work)
        #Scale up array via duplication
        biggerimagearray = []
        biggerimgarrayline = []
        for line in imagearray.tolist():
            for p in line:
                for i in range(0,scale):
                    biggerimgarrayline.append(p)
            for i in range(0, scale):
                biggerimagearray.append(biggerimgarrayline)
            biggerimgarrayline = []
        #return Larger true/false array
        array = np.array(biggerimagearray)


        print("ARRAY PROCESSING DONE")
        return [array,obarray]

    def padToMatch(self, input, toMatch):
        [xmatch, ymatch] = np.shape(toMatch)
        [xin, yin] = np.shape(input)
        output = np.pad(input, ((0, xmatch - xin), (0, ymatch - yin)), 'edge')
        return output

    def obsSpaceGen(self,robotShape,obsSpace):
        #Get x/y shape of robot (assuming rectangular)
        [x,y] = robotShape
        xcent = math.ceil(x/2)
        ycent = math.ceil(y/2)
        dist = max(xcent,ycent)
        #pad obsspace to prevent rollover
        print(np.shape(obsSpace))
        obs = np.pad(obsSpace,((dist,dist),(dist,dist)),'constant',constant_values=0)
        print(np.shape(obs))
        #pad robot shape with zeros, to fix rotation
        robot = np.ones((y, x))
        space = np.zeros((y, x))
        newspace = np.concatenate((space, robot, space), 0)
        empty = np.concatenate((space, space, space))
        robotSpace = np.hstack((empty, newspace, empty))
        RS = im.fromarray(robotSpace)
        robotAngles = []
        # Create array of robot angle shifts
        for i in range(0, 180):
            robotAngles.append(self.padToMatch(np.array(RS.rotate(i)), obs))

        print(obs)
        plt.imshow(obs,interpolation='nearest')
        plt.show()
        #check all angles for robotAngles
        newobs = []
        for x in range(0, np.shape(obs)[1]):
            newobsy = []
            for y in range(0,np.shape(obs)[0]):
                newobst = []
                for i in range(0, 180,45):
                    newobst.append(np.any(np.logical_and(obs, robotAngles[i])))
                obs = np.roll(obs,-1,axis = 0)
                newobsy.append(newobst)
                print('Y:',y)
            obs = np.roll(obs, -1, axis = 1)
            newobs.append(newobsy)
            print("X:",x)
        newobs = np.array(newobs)
        newobs = np.roll(newobs,2*dist,axis=0)
        newobs = np.roll(newobs,2*dist, axis=1)
        newobs = newobs[0:-2*dist,0:-2*dist]
        print("done")
        print(np.shape(np.array(newobs)))
        [d1,d2,d3] = np.shape(newobs)
        print(np.any(newobs))
        deobs = np.dsplit(np.array(newobs),d3)
        print(deobs)
        print(np.shape(np.squeeze(deobs[0])))
        plt.imshow(np.squeeze(deobs[0]).T,interpolation='nearest')
        plt.show()
        plt.imshow(np.squeeze(deobs[1]).T, interpolation='nearest')
        plt.show()
        print(newobs)

if __name__ == "__main__":
    o = imgToObs(imagepath = "C:/Users/Cooper/PycharmProjects/183Final/controllers/hawk_control2/testmaze.png")
    gray = cv2.cvtColor(o.image, cv2.COLOR_BGR2GRAY)
    plt.imshow(np.array(gray))
    plt.show()
    o.obsSpaceGen([20,20],np.logical_not(np.array(gray)))