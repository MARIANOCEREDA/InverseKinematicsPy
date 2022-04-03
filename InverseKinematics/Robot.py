import numpy as np
from Matrix import invHomog,trotx,troty,trotz,transl
from typing import List,Dict 
import math

class Robot:
    def __init__(self,dhMatrix,qlims,toolMatrix,qOffset,base,T) -> None:
        # Joints
        self.q1 = np.array(2)
        self.q2 = np.array(4)
        self.q3 = np.array(4)

        # Robot Features
        self.dh = dhMatrix
        self.qlims = qlims
        self.tool = toolMatrix
        self.qoffset = qOffset
        self.qlast = np.zeros((6))
        self.base = base
        self.qfinal = np.zeros((6,8))

        # Matrix containing xyz point to be reached by the tool
        self.T = T
    
    def calculatePosition(self) -> np.array:
        '''
        It calculates the position of the point p04, which is located in the wistle of the robot. The
        wistle is the intersection between the last 3 axis. We work using STATIC TYPING.
        '''
        a:np.array = invHomog(self.base)*self.T
        self.T:np.array = a*invHomog(self.tool)
        p04:np.array = self.T[:3,3]- self.dh[5,1]* self.T[:3,3]
        return p04

    def getQ1(self) -> None:
        '''
        It calculates the possible values for the first joint. In this case, we have 2 possible values.
        '''
        p04 = self.calculatePosition()
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
        self.q1 = q1
        return p04
    
    def getQ2(self,p04):
        '''
        It calculates 4 possible values for the second joint. They depend on the q1 value.
        '''
        alphaVerify = list()
        q2 = np.zeros(4)
        j=0
        for i in range(2):
            #First, we get the new homogeneous matrix, then we get p14 point
            T1a = np.matmul(trotz(self.q1[i]),transl([0,0,self.dh[0,1]]))
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
        
        #Finally, we fill out the q2 array with the previously obtained values
        self.qfinal[1,:]=[q2[0],q2[0],q2[1],q2[1],q2[2],q2[2],q2[3],q2[3]]
        self.q2 = q2

        return p14
    
    def getQ3(self,p04) -> np.array:
        '''
         It calculates 4 possible values for the third joint. They depend on the q1 and q2 value.
        '''
        q3 = np.zeros(4)
        j=0
        for i in range(4):
            # Calculate T1a in 2 different ways depending on i
            if i in [0,1]:
                 T1a = np.matmul(trotz(self.q1[0]),transl([0,0,self.dh[0,1]]))
            elif i in [2,3]:
                 T1a = np.matmul(trotz(self.q1[1]),transl([0,0,self.dh[0,1]]))

            # Calculate T2
            T1b = np.matmul(T1a,transl([self.dh[0,2],0,0]))
            T1 = np.matmul(T1b,trotx(self.dh[0,3]))
            T12a = np.matmul(trotz(self.q2[i]),transl([0,0,self.dh[1,1]]))
            T12b = np.matmul(T12a,transl([self.dh[1,2],0,0]))
            T12 = np.matmul(T12b,trotx(self.dh[0,3]))
            T2 = np.matmul(T1,T12)

            # Calculate the first 2 values for q1
            p24 = np.matmul(invHomog(T2),np.array([p04[0],p04[1],p04[2],1]))
            q3[j] = math.atan2(p24[1],p24[0])+np.pi/2
            j+=1

        # We fill out the matrix containing all the possible results wit Q3
        self.qfinal[2,:]= [q3[0],q3[0],q3[1],q3[1],q3[2],q3[2],q3[3],q3[3]]

        self.q3 = q3
        return p24