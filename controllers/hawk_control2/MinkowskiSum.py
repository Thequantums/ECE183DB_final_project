import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy import signal as s
import math
from PIL import Image as im

def mSum(space,obj):
    pass






if __name__ == "__main__":
    imagepath="C:/Users/Cooper/PycharmProjects/183Final/controllers/hawk_control2/testmaze.png"
    img = cv2.imread(imagepath)
    space = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    space = np.logical_not(np.array(space))
    robot = np.ones((10, 10))
    #plt.imshow(space)
    #plt.show()
    Min = s.convolve2d(space,robot)
    Min = np.logical_not(np.logical_not(Min))
    plt.imshow(Min)
    plt.show()


