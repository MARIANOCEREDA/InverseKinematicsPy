import numpy as np
from Matrix import invHomog

class Robot:
    def __init__(self,T) -> None:
        self.q = list()
        self.dhMatrix = np.array()
        self.qlims = list()
        self.toolMarix = np.array()
        self.qoffset = np.array()
        self.qlast = np.zeros((6))
        self.baseMatrix = np.array()
        self.qfinal = np.array()
        self.T = T
    
    def calculatePosition(self) -> list():
        a = invHomog(self.baseMatrix)*self.T
        self.T = a*invHomog(self.toolMarix)
        p04= self.T[:3,3]- self.dhMatrix[5,1]* self.T[:3,3]
        return p04

    def q1Calculate() ->list:
        pass