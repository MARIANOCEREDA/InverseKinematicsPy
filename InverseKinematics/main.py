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
    f1:list = [0.7959,0.2612,0.5461,0.6795]
    f2:list = [0.0177,0.8917,-0.4522,0.6699]
    f3:list = [-0.6051,0.3696,0.7051,2.6586]
    f4:list = [0,0,0,1]
    T = np.array([f1,f2,f3,f4])

    #Offset array
    qOffset = np.array([0,45,45,0,0,0,0])*np.pi/180

    #Limits of each joint
    ql1:List[float] = [185,-185]
    ql2:List[float] = [-105,130]
    ql3:List[float] = [-165,120]
    ql4:List[float] = [-180,180]
    ql5:List[float] = [-125,125]
    ql6:List[float] = [-350,350]
    qlims = np.array([ql1,ql2,ql3,ql4,ql5,ql6])*np.pi/180

    #Tool matrix
    toolMatrix = np.identity(4)

    #Base matrix
    baseMatrix = np.identity(4)

    #Last q
    qant = np.zeros(6)

    #Coding of inverse kinematics
    R = Robot("KUKA 6DOF",dhMatrix,qlims,toolMatrix,qOffset,baseMatrix,T)

    # Joints for position 
    p04 = R.getQ1()
    p14 = R.getQ2(p04)
    p24 = R.getQ3(p04)
    
    #Joints for orientation
    R.getQ456(qant)

    #Final matrix with all the possible values
    print(R.qfinal)





