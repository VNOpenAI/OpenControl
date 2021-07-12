import numpy as np
import random
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

def get_coefficient(s):
    """Finding Coefficient of a term in Polynomial with predefine solution
    Args:
        s (list): solution of Polynomial
    Returns:
        np.ndarray: [description]
    """
    len_ = len(s)
    A = np.zeros((len_,len_))
    b = np.zeros((len_,1))
    for i in range(len_):
        giatri = random.randint(0,100)
        tich = 1 
        for index,nghiem in enumerate(s):
            tich = tich*(giatri-nghiem)
        tich = tich - math.pow(giatri,len_)
        for j in range(len_):
            A[i,j] = math.pow(giatri,j)
        b[i][0]=tich 
    x = scipy.linalg.pinv(A)@b
    return x