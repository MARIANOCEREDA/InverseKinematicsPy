from array import array
from ctypes import sizeof
import numpy as np
import math
from functools import reduce


def invHomog(matrix) -> np.array:
    '''
    It executes the homogeneus inverse of the matrix. It is better to do in this way, instead of the normal
    inverse
    '''
    i = np.identity(4)
    partialM = matrix[:3,:3]
    i[:3,:3] = np.transpose(partialM)
    i[:3,3]= np.matmul(-i[:3,:3],matrix[:3,3])
    return i

def trotx(alpha) -> np.array:
    '''
    It creates a rotation matrix aroung  the x axis 
    '''
    cosa = math.cos(alpha)
    sina = math.sin(alpha)
    rotMatrix = np.identity(4)
    rotMatrix[1,1]=cosa
    rotMatrix[2,2]=cosa
    rotMatrix[2,1]=sina
    rotMatrix[1,2]=-sina
    return rotMatrix

def troty(alpha) -> np.array:
    '''
    It creates a rotation matrix around  the y axis 
    '''
    cosa = math.cos(alpha)
    sina = math.sin(alpha)
    rotMatrix = np.identity(4)
    rotMatrix[0,0]=cosa
    rotMatrix[2,2]=cosa
    rotMatrix[2,0]=-sina
    rotMatrix[0,2]=sina
    return rotMatrix

def trotz(alpha) -> np.array:
    '''
    It creates a rotation matrix aroung  the z axis 
    '''
    cosa = math.cos(alpha)
    sina = math.sin(alpha)
    rotMatrix = np.identity(4)
    rotMatrix[0,0]=cosa
    rotMatrix[1,1]=cosa
    rotMatrix[0,1]=-sina
    rotMatrix[1,0]=sina
    return rotMatrix
    

def transl(xyzTrans:list) -> np.array:
    '''
    It translates the identity matrix to the xyzTrans point.
    '''
    xyz = np.array(xyzTrans)
    translMatrix = np.identity(4)
    translMatrix[:3,3]=xyz
    return translMatrix

def matmul(*args) -> np.array:
    '''
    Simplify the operation of multipliying multiple matrix
    '''
    result = np.array(reduce(lambda m1,m2:np.matmul(m1,m2),args))

    return result


def Adh(dh,q) -> np.array:
    '''
    It calculates the homogeneus matrix for different given q values
    '''
    dh2T = matmul(trotz(q),transl([0,0, dh[0,1]]),transl([dh[0,2], 0, 0]),trotx(dh[0,3]))

    return dh2T