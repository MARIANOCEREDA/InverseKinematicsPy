from asyncio import constants
from tkinter.tix import DisplayStyle
import numpy as np
from Matrix import *
from typing import List,Dict 
import math

EPS = np.finfo(float).eps

class Robot:
    def __init__(self,name,dhMatrix,qlims,toolMatrix,qOffset,base,T) -> None:
        # Name
        self.name = name 

        # Position Joints
        self.q1 = np.array(2,dtype='float32')
        self.q2 = np.array(4,dtype='float32')
        self.q3 = np.array(4,dtype='float32')

        # Orientation Joints
        self.q4 = np.zeros(2,dtype='float32')
        self.q5 = np.zeros(2,dtype='float32')
        self.q6 = np.zeros(2,dtype='float32')

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
            T1 = matmul(trotz(self.q1[i]),transl([0,0,self.dh[0,1]]),transl([self.dh[0,2], 0, 0]),trotx(self.dh[0,3]))
            p14 = matmul(invHomog(T1),np.array([p04[0],p04[1],p04[2],1]))

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
        q2tot = list(map(lambda x,y:[x,y],q2,q2))
        q2flatter = [item for lista in q2tot for item in lista]
        self.qfinal[1,:]= q2flatter
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
            T1 = matmul(T1a,transl([self.dh[0,2],0,0]),trotx(self.dh[0,3]))
            T12 = matmul(trotz(self.q2[i]),transl([0,0,self.dh[1,1]]),transl([self.dh[1,2],0,0]),trotx(self.dh[0,3]))
            T2 = matmul(T1,T12)

            # Calculate the first 2 values for q1
            p24 = np.matmul(invHomog(T2),np.array([p04[0],p04[1],p04[2],1]))
            q3[j] = math.atan2(p24[1],p24[0])+np.pi/2
            j+=1

        # We fill out the matrix containing all the possible results wit Q3
        q3tot = list(map(lambda x,y:[x,y],q3,q3))
        q3flatter = [item for lista in q3tot for item in lista]
        self.qfinal[2,:]= q3flatter
        self.q3 = q3
        
        return p24
    
    def getQ456(self,qant:np.array) -> np.array:
        '''
        It calculates the last 3 joint values.E.g the rotation joints
        '''

        k = 0 
        for i in range(0,7,2):
            T10 = Adh(self.dh,self.qfinal[0,i])
            T21 = Adh(self.dh,self.qfinal[1,i])
            T32 = Adh(self.dh,self.qfinal[2,i])
            T63 = matmul(invHomog(T32),invHomog(T21),invHomog(T10),self.T)

            # Degenerate solution
            if abs(T63[2,2]-1) < EPS:
                self.q4[0] = qant[3]
                self.q5[0] = 0
                self.q6[0] = math.atan2(T63[1,0], T63[0,0]) - self.q4[0]
                self.q4[1] = self.q4[0]
                self.q5[1] = 0
                self.q6[1] = self.q6[0]

            #Normal solution
            else:
                self.q4[0] = math.atan2(-T63[1,2],-T63[0,2])
                if (self.q4[0]>0):
                    self.q4[1] = self.q4[0] - np.pi
                else:
                    self.q4[1] = self.q4[0] + np.pi
                #q5 = np.zeros(1,2)
                #q6 = q5
                for m in range(0,1,1):
                    T43 = Adh(self.dh, self.q4[m])
                    T46 = matmul(invHomog(T43),T63)
                    self.q5[m] = math.atan2(-T46[0,2],T46[1,2])+np.pi
                    T45 = matmul(trotz(self.q5[m]),transl([0,0,self.dh[4,1]]),transl([self.dh[4,2],0,0]),trotx(self.dh[4,3]))
                    T56 = matmul(invHomog(T45),T46)
                    self.q6[m] = math.atan2(T56[1,0], T56[0,0])
                self.qfinal[3,k:k+1]=self.q4[1:2] 
                self.qfinal[4,k:k+1]=self.q5[1:2] 
                self.qfinal[5,k:k+1]=self.q6[1:2]
                k+=2
                self.qfinal = self.qfinal - matmul(self.qoffset.transpose(),np.ones(7))




