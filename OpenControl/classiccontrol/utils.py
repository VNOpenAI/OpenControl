import numpy as np
import json
def is_definite_positive_matrix(M):
    condition1 = np.all(np.linalg.eigvals(M) > 0)
    condition2 = np.allclose(M,M.T)
    return  condition1 and condition2

def read_config(config_path=''):
    with open(config_path,'r') as f:
        config = json.load(f)
    return config
def get_coefficient(s):
    #s =set(s)
    
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