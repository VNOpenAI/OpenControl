import numpy as np
from abc import ABC, abstractmethod
import scipy
from scipy import linalg
import scipy.linalg

class BIBO(ABC):

    @abstractmethod
    def conclusion(self):

        pass


class Gerschgorin(BIBO):

    def __init__(self,A: np.ndarray):
        self.A = A 
    def conclusion(self)-> int: 
        """[summary]
        """
        A = self.A
        diag = np.diag(A)
        B = np.sum(np.abs(A), axis=1)
        C = B - np.abs(diag) + diag
        result = np.all((C) <=0)
        
        if result:
            return 1 
        else: 
            return 0

class Lyapunov(BIBO):

    def __init__(self, A, P=None, Q=None):
        """[summary]

        Args:
            A ([type]): [description]
            P ([type], optional): [description]. Defaults to None.
            Q ([type], optional): [description]. Defaults to None.
        """
        self.A = A
        shape = A.shape
        assert (P is None) ^ (Q is None), "please fill either P or Q"
        self. P = P
        self. Q = Q
        if P is not None:
            assert P.shape == A.shape, "Invalid shape of matrix P"
        else:
            assert Q.shape == A.shape, "Invalid shape of matrix Q"
    @staticmethod
    def is_definite_positive_matrix(M):
        """[summary]

        Args:
            M ([type]): [description]

        Returns:
            [type]: [description]
        """
        condition1 = np.all(np.linalg.eigvals(M) > 0)
        condition2 = np.allclose(M,M.T)
        return  condition1 and condition2

    def conclusion(self):
        """[summary]

        Returns:
            [type]: [description]
        """
        if self.P is not None:
            Q = -(self.P @ self.A + self.A.T @ self. P)
            print(f'Q={Q}')
            result = Lyapunov.is_definite_positive_matrix(Q)
            
        else:
            P = linalg.solve_sylvester(a=self.A.T, b=self.A, q=-self.Q)
            print(f'P={P}')
            result = Lyapunov.is_definite_positive_matrix(M=P)
        if result: 
            return 1
        else:
            return 0

class Hurwitz(BIBO):
    """[summary]

    Args:
        BIBO ([type]): [description]
    """
    def __init__(self,A):
        self.A = A

    def conclusion(self,):
        result =  np.all(scipy.linalg.eigvals(self.A).real <0)
        if result:
            return 1 
        else: 
            return -1




