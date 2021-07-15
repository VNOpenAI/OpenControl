### This examples the simulation examples in the paper of Yu Jiang and Zhong-Ping Jiang, the very first paper using ADP for completely unknown continues-time linear systems. We simulate with OpenControl, Off Policy ADP Controller. The results that is the same as the paper's results demonstrate our python packages.

### you can check the pape at https://www.sciencedirect.com/science/article/abs/pii/S0005109812003664

### run simulation with the bash script: python TurbochargedDieselEngine.py

from OpenControl.ADP_control import LTI, LTIController
from matplotlib import pyplot as plt
import numpy as np


# define system
A = np.array([[-0.4125, -0.0248, 0.0741, 0.0089, 0, 0], \
                [101.5873, -7.2615, 2.7608, 2.8068, 0, 0],
                [0.0704, 0.0085, -0.0741, -0.0089, 0, 0.02],
                [0.0878, 0.2672, 0, -0.3674, 0.0044, 0.3962],
                [-1.8414, 0.099, 0, 0, -0.0343, -0.033],
                [0, 0, 0, -359.0, 187.5364, -87.0316]])
B = np.array([[-0.0042, 0.0064], [-1.0360, 1.5849], [0.0042, 0], [0.1261, 0], [0, -0.0168], [0, 0]])
system = LTI(A,B)

# set simulation parameters
max_step = 1e-3
t_sim = (0, 5)
x0 = np.array([20, 5, 10, 2, -1, -2])
sample_time = 1e-3
system.setSimulationParam(max_step=max_step, t_sim=t_sim, x0=x0, sample_time=sample_time)

# pass system to controller
controller = LTIController(system)
# set parameters for policy
Q = np.diag([1, 1, 0.1, 0.1, 0.1, 0.1]); R = np.eye(2); K0 = np.zeros((2,6))
data_eval = 0.01; num_data = 200
freq = ((np.random.rand(2,100)-0.5)*100).astype(int)
explore_noise = lambda t: 100*np.sum(np.sin(freq*t), axis=1)

controller.setPolicyParam(K0=K0, Q=Q, R=R, data_eval=data_eval, num_data=num_data, explore_noise=explore_noise)

# take simulation and get the results
stop_thres = 0.03
max_iter = 30
K, P = controller.offPolicy(stop_thres=stop_thres,max_iter=max_iter)
print('Off policy applied \n')
print('Optimal policy K = \n', K, '\n Optimal value P = \n', P)