import numpy as np
from Matrix import invHomog,trotx,troty,trotz,transl
from typing import List,Dict 
import math

class Robot:
    def __init__(self,dhMatrix,qlims,toolMatrix,qOffset,baseMatrix,T) -> None:
        self.q = list()
        self.dh = dhMatrix
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
        p04:np.array = self.T[:3,3]- self.dh[5,1]* self.T[:3,3]
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
        self.qfinal[0,4:8] = q1[1]
        return q1
    
    def q2Calculate(self,p04,q1):
        '''
        It calculates 4 possible values for the second joint. They depend on the q1 value.
        '''
        alphaVerify = list()
        q2 = np.zeros(4)
        j=0
        for i in range(2):
            #First, we get the new homogeneous matrix, then we get p14 point
            T1a = np.matmul(trotz(q1[i]),transl([0,0,self.dh[0,1]]))
            T1b = np.matmul(T1a,transl([self.dh[0,2], 0, 0]))
            T1 = np.matmul(T1b,trotx(self.dh[0,3]))
            p14 = np.matmul(invHomog(T1),np.array([p04[0],p04[1],p04[2],1]))

            #Secondly, we calculate the beta angle, L1,L2 and the diagonal. 
            beta = math.atan2(p14[1],p14[0])
            L2 = self.dh[1,2] #Longitud eslabon 2
            L3 = self.dh[3,1] #Longitud eslabon 3
            diag = math.sqrt(math.pow(p14[0],2) + math.pow(p14[1],2))
            alpha = math.acos((math.pow(L2,2)-math.pow(L3,2)+math.pow(diag,2))/(2*diag*L2))
            isreal = True

            #If the result is or not complex will change the results at the end.
            if type(alpha) == "complex":
                isreal = False
            alphaVerify.append(isreal)
            q2[j]=beta-alpha.real #Obtenemos 2 valores de q2 para cada valor de q1
            j+=1
            q2[j]=beta+alpha.real
            j+=1
        
        #Finally, we fill the q2 array with the previously obtained values
        self.qfinal[1,:]=[q2[0],q2[0],q2[1],q2[1],q2[2],q2[2],q2[3],q2[3]]
        return q2,p14
