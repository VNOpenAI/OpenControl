import numpy as np
import scipy 
from .utils import get_coefficient

class Luenberger():

    algorithms = ['Ropernecker','Arckerman']

    def __init__(self,pole,system,algo):
        """this class design the Luenberger Observer for LTI system

        Args:
            pole : list
                Desire poles of observer
            system : OpenControl.classiccontrol.linearsystem.LTI
                The object of Controller
            algo : string
                
        """
        assert algo in ['Ropernecker','Arckerman'], "algo must be in list of ['Ropernecker','Arckerman']"
        self.pole = pole
        self.pole.sort() 
        self.algo = algo  
        self.system = system


    def Roppernecker(self):
        
        observerable = self.system.is_observerble()
        if not observerable:
            print('the system is unobserverable')
            return
        eigvals,eigvectors = np.linalg.eig(self.system.A)
        eigvals = eigvals.tolist()
        a= []
        t = []
        for index,s in enumerate(self.pole):
            if s in eigvals:
                a.append(eigvectors[:,index].reshape(-1,1))
                t_index = np.zeros((self.system.inputs_shape,1))
                t.append(t_index)
            else:
                a.append( np.linalg.inv(s*np.eye(self.system.states_shape)- self.system.A)@self.system.C)
                t_index = np.ones((self.system.inputs_shape,1))
                t.append(t_index)            
        a = np.concatenate(a,axis=1)
        t = np.concatenate(t,axis=1)
        R = - t @ np.linalg.inv(a)
        return R

    def Arckerman(self,):
        #@if self._B is None:
        #    raise ValueError('please provide B matrix')
        A = self.system._A 
        B = self.system.C
        M = B
        ndim = self.system._states_shape
        if ndim==1:
            if np.linalg.matrix_rank(B) == 1:
                return True
            else:
                return False
        X = A @ B
        M = np.hstack([M,X])
        for i in range(ndim-2):
            X = A @ X
            M = np.hstack([M,X])   
        M = np.linalg.inv(M)
        s = M[-1].reshape(1,-1)
        S = 0 
        coefficient = get_coefficient(self.pole).reshape(-1)
        coefficient = coefficient.tolist()
        coefficient.append(1)
        for i in range(ndim+1):
            coeffi = coefficient[i]          
            S += coeffi*(s@np.linalg.matrix_power(A,i))
        return S
