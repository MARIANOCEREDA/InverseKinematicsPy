from traceback import print_tb
import numpy as np
import math

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
    It translates the matrix to the xyzTrans point.
    '''
    xyz = np.array(xyzTrans)
    translMatrix = np.identity(4)
    translMatrix[:3,3]=xyz
    return translMatrix
