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
