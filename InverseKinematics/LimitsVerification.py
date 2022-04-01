from asyncio import FastChildWatcher
from typing import Dict
import numpy as np

def limitsVerification(qlims:np.array,q:np.array) -> Dict:
    '''
    This function verify if the introduced vector of joint values are inside the bounds of the workspace.
    If true, it will return the array again.
    If not, it will return the array with new join values inside the bounds of the workspace.
    '''
    qCorrect = np.array((1,7))
    for i,item in np.nditer(q):
        if(item<qlims[item,0] or item>qlims[i,1]):
            condition:bool = True
            if item < 0:
                qCorrect[i] = qlims[i,0]
            elif item > 0:
                qCorrect[i] = qlims[i,0]
        else:
            qCorrect = item
            condition = False

    return {"condition":True,"q":qCorrect}
