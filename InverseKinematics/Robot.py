import numpy as np
from Matrix import invHomog
from typing import List,Dict 
import math

class Robot:
    def __init__(self,dhMatrix,qlims,toolMatrix,qOffset,baseMatrix,T) -> None:
        self.q = list()
        self.dhMatrix = dhMatrix
        self.qlims = qlims
        self.toolMarix = toolMatrix
        self.qoffset = qOffset
        self.qlast = np.zeros((6))
        self.baseMatrix = baseMatrix
        self.qfinal = np.zeros((6,8))
        self.T = T
    
    def calculatePosition(self) -> np.array:
        '''
        It calculates the position of the point p04, which is located in the wistle of the robot. The
        wistle is the intersection between the las 3 axis. We work using STATIC TYPING.
        '''
        a:np.array = invHomog(self.baseMatrix)*self.T
        self.T:np.array = a*invHomog(self.toolMarix)
        p04:np.array = self.T[:3,3]- self.dhMatrix[5,1]* self.T[:3,3]
        return p04

    def q1Calculate(self,p04) -> np.array:
        '''
        It calculates the possible values for the first joint. In this case, we have 2 possible values.
        '''
        q1 = np.zeros(2)
        q1[0] = math.atan2(p04[1],p04[0])

        #Depending on the value of q1[0], the value of q1[1] will change
        if(q1[0]>0):
            q1[1]=q1[0]-np.pi
        else:
            q1[1]=q1[0]+np.pi
        
        #Now we fill the matrix qFinal 
        self.qfinal[0,:3] = q1[0]
        self.qfinal[0,4:7] = q1[1]
        return q1
    
    def q2Calculate(self,q1):
        '''
        It calculates 4 possible values for the second joint. They depend on the q1 value.
        '''
        
