import numpy as np
from numpy import linalg as LA



def SampleUnitNBall(dim = 3,num = 1):

    u = np.random.normal(0, 1, (num, dim + 2))
    norm = LA.norm(u, axis = -1,keepdims = True)
    u = u/norm
    if num == 1: return u[0,:dim]
    return u[:,:dim]



class EllipsoidSampler:

    def __init__(self,center,axes = [],rot = []):
     
        self.dim = center.shape[0]
        self.center = center
        self.rot = rot
        if len(rot) == 0: self.rot = np.eye(self.dim)
        if len(axes) == 0: axes = [1]*self.dim
        self.L = np.diag(axes)

    def sample(self,num = 1):
        xball = SampleUnitNBall(self.dim,num)
        xellip = (self.rot@self.L@xball.T).T + self.center
        return xellip


class InformedSampler:
  
    def __init__(self, goal, start):
        self.dim = goal.shape[0]
        self.cmin = LA.norm(goal - start)
        center = (goal + start)/2
        C = self.RotationToWorldFrame(goal, start)
        self.ellipsampler = EllipsoidSampler(center,rot = C)

    def sample(self, cmax, num = 1):
 
        r1 = cmax/2
        ri = np.sqrt(cmax**2 - self.cmin**2)/2
        axes = [r1]+ [ri]*(self.dim - 1)
        self.ellipsampler.L = np.diag(axes)
        return self.ellipsampler.sample(num)

    def RotationToWorldFrame(self, goal, start):

        E1 = (goal - start) / self.cmin
        W1 = [1]+[0]*(self.dim - 1)
        M = np.outer(E1,W1)
        U, S, V = LA.svd(M)
        middleM = np.eye(self.dim)
        middleM[-1,-1] = LA.det(U)*LA.det(V)
        C = U@middleM@V.T
        return C
