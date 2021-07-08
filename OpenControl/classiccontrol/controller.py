import numpy as np 
from abc import ABC,abstractmethod
import math
import scipy.linalg
import random
from .utils import get_coefficient
    
class StateFeedBackController(ABC):
    """Abstract class for controller based on state feedback 
    """
    @abstractmethod
    def compute(self):
        """Return the matrix of controller
        """
        pass


class PoleStatement(StateFeedBackController):
    """Design controller, move the eigvalues of system to desire

    Args:
        StateFeedBackController ([type]): [description]
    """
    def __init__(self,pole,system,algo='Arckerman'):
        """Inherit SateFeedBackController 


        Args:
            pole ([list]): [description]
            system ([OpenControl.classiccontrol.linearsystem.LTI]): [the object of Controller]
            algo ([strings]): select algorithm to design controller. Defaults to 'Arckerman'.
        """
        assert algo in ['Ropernecker','Arckerman'], "algo must be in list of ['Ropernecker','Arckerman']"
        self.pole = pole
        self.pole.sort() 
        self.algo = algo
        if self.system.inputs_shape != 1 and algo=='Arckerman':
            print('The Arckerman method only design for 1D_input system. Automatically switch to Ropernecker method')
            self.algo = 'Ropernecker'
        self.system = system

    def _Roppernecker(self,):
        """This function Implement Controller, Designed based on Ropernecker algorithms

        Raises:
            ValueError: if system has no matrix B

        Returns:
            [np.ndarray]: 2D array, matrix of controller. got the shape (input_shape,state_shape)
        """
        
        if self.system._B is None:
            raise ValueError('matrix B must be provided')
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
                a.append( np.linalg.inv(s*np.eye(self.system.states_shape)- self.system.A)@self.system.B)
                t_index = np.ones((self.system.inputs_shape,1))
                t.append(t_index)
                
        a = np.concatenate(a,axis=1)
        t = np.concatenate(t,axis=1)
        R = - t @ np.linalg.inv(a)
        return R

    def _Arckerman(self,):
        if self.system._B is None:
            raise ValueError('please provide B matrix')
        A = self.system._A 
        B = self.system._B
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



    def compute(self):
        controlability = self.system.is_controlable()
        if not controlability:
            print('system is not controlable')
            return None
        if self.algo =='Arckerman':
            return self._Arckerman()
        else: 
            return self._Roppernecker()

class LQR(StateFeedBackController):

    def __init__(self,system,E,F):
        self.system = system
        self.E = E 
        self.F = F 
    
    def compute(self):
        q = self.E 
        a = self.system.A
        b = self.system.B
        r = self.F 
        P = scipy.linalg.solve_continuous_are(a=a,b=b,q=q,r=r)
        R = scipy.linalg.pinv(self.F)@(self.system.B.T)@P 
        return R