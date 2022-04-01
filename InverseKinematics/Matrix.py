from traceback import print_tb
import numpy as np

def invHomog(matrix) -> np.array:
    i = np.identity(4)
    partialM = matrix[:3,:3]
    i[:3,:3] = np.transpose(partialM)
    i[:3,3]= np.matmul(-i[:3,:3],matrix[:3,3])
    return i
