import numpy as np
from abc import ABC, abstractmethod
import scipy
from scipy import linalg
import scipy.linalg

class BIBO(ABC):
    """Abtract class for implement the algorithms. Which dertermine
        the stability of system
    """
    @abstractmethod
    def conclusion(self) -> int:
        """conclude about the stability of system.
            This function implement the algorithms.
        Returns:
            [int]: if 1, system is stable
                   if 0, this algorithms unable to conclude about stability os system
                   if -1, system is unstable
        """
        pass


class Gerschgorin(BIBO):

    def __init__(self,A: np.ndarray):
        self.A = A 
    def conclusion(self)-> int: 
        """conclude about the stability of system.
            This function implement the algorithms.
        Returns:
            [int]: if 1, system is stable
                   if 0, this algorithms unable to conclude about stability os system
                   if -1, system is unstable
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
            A : nxn 2D-np.ndarray
                system matrix
            P : 2D-array
                an define-positive matrix, needed matrix solve Ricati equations. If solution is define positive matrix,
                system is stable. if P is provide, Q must be None. Defaults to None.
            Q : 2D-array
                the same as description of arg P,if Q is provide, P must be None. Defaults to None.   
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
        """Determine a maxtrix if 

        Args:
            M : 2D-ndarray

        Returns:
            [Boolean]: True if matrix is definite positive
        """
        condition1 = np.all(np.linalg.eigvals(M) > 0)
        condition2 = np.allclose(M,M.T)
        return  condition1 and condition2

    def conclusion(self):
        """conclude about the stability of system.
            This function implement the algorithms.
        Returns:
            [int]: if 1, system is stable
                   if 0, this algorithms unable to conclude about stability os system
                   if -1, system is unstable
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
    def __init__(self,A):
        self.A = A

    def conclusion(self,):
        """conclude about the stability of system.
            This function implement the algorithms.
        Returns:
            [int]: if 1, system is stable
                   if 0, this algorithms unable to conclude about stability os system
                   if -1, system is unstable
        """
        result =  np.all(scipy.linalg.eigvals(self.A).real <0)
        if result:
            return 1 
        else: 
            return -1




