from Robot import *
import numpy as np

if __name__ == "__main__":

    #Definition of Denavit Hartenberg matrix
    j1:list = [0,0.575,0.175,np.pi/2]
    j2:list = [0,0,0.89,0]
    j3:list = [0,0,0,np.pi/2]
    j4:list = [0,1.05,0,- np.pi/2]
    j5:list = [0,0,0,np.pi/2]
    j6:list = [0,0.185,0,0]
    dhMatrix = np.array([j1,j2,j3,j4,j5,j6])

    #T Matrix: Containing the data of the point to reach.
    T = 

    #Limits of each joint
    ql1:list = [185,-185]*np.pi/180
    ql2:list = [-105,130]*np.pi/180
    ql3:list = [-165,120]*np.pi/180
    ql4:list = [-180,180]*np.pi/180
    ql5:list = [-125,125]*np.pi/180
    ql6:list = [-350,350]*np.pi/180
    qlims = np.array([ql1,ql2,ql3,ql4,ql5,ql6])

    R = Robot(dhMatrix,T)


