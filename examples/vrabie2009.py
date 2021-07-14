### This examples the simulation examples in the paper of Vrabie and F.Lewis, the very first paper in in the ADP control area using neural networks. But in this python implementation, we dont use the algorithm in the paper because the original one requires knowledge the system dynamics. Instead, we simulate with OpenControl, Off Policy ADP Controller. The results that is the same as the paper's results demonstrate our python packages.

### you can check the pape at https://www.sciencedirect.com/science/article/abs/pii/S0893608009000446

### run simulation with the bash script: python vrabie2009.py


from OpenControl.ADP_control import NonLin, NonLinController
from matplotlib import pyplot as plt
import numpy as np

########################## Example 1
print("Example 1")
# define system
def dot_x(t,x,u):
    # dynamic of the system
    x1 = x[0]
    x2 = x[1]

    # coefficient   

    # system dynamics
    dx1 = -x1 + x2
    dx2 = -0.5*(x1+x2)+0.5*x2*np.sin(x1)**2 + np.sin(x1)*u

    dx = [dx1, dx2]
    return dx

dimension = (2,1)
sys = NonLin(dot_x, dimension)

# setup simulation
t_start = 0; t_stop = 20
x0 = np.array([-1, 1])
sample_time = 0.01

sys.setSimulationParam(t_sim=(t_start, t_stop), sample_time=sample_time, x0=x0)

# define and setup controller
ctrl = NonLinController(sys)
u0 = lambda x: -1.5*np.sin(x[0])*(x[0] + x[1])
data_eval = 0.1; num_data = 30
explore_noise = lambda t: 0.2*np.sum(np.sin(np.array([1, 3, 7, 11, 13, 15])*t))

phi_func = lambda x: np.array([x[0]**2, x[0]*x[1], x[1]**2])

ctrl.setPolicyParam(data_eval=data_eval, num_data=num_data, explore_noise=explore_noise, u0=u0, phi_func=phi_func)

# run simuilation
Wc, Wa = ctrl.offPolicy()
print("The optimal weight of the critic")
print(Wc)
# for visualizing the results, using tensorboard




########################## Example 2
print("example 2")

def dot_x(t,x,u):
    # dynamic of the system
    x1 = x[0]
    x2 = x[1]

    # coefficient   

    # system dynamics
    dx1 = -x1 + x2 + 2*x2**3
    dx2 = -0.5*(x1+x2)+0.5*x2*(1 + 2*x2**2)*np.sin(x1)**2 + np.sin(x1)*u

    dx = [dx1, dx2]
    return dx

dimension = (2,1)
sys = NonLin(dot_x, dimension)

t_start = 0; t_stop = 20
x0 = np.array([-1, 1])
sample_time = 0.01

sys.setSimulationParam(t_sim=(t_start, t_stop), sample_time=sample_time, x0=x0)

ctrl = NonLinController(sys)
def u0(x):
    x1 = x[0]
    x2 = x[1]
    return -0.5*np.sin(x1)*(3*x2 - 0.2*x1**2*x2 + 12*x2**3)

data_eval = 0.1; num_data = 40
explore_noise = lambda t: 0.2*np.sum(np.sin(np.array([1, 3, 7, 11, 13, 15])*t))

# use default phi_func
def phi_func(x):
    x1 = x[0]
    x2 = x[1]
    return np.array([x1**2, x1*x2, x2**2, x1**4, x1**3*x2, x1**2*x2**2, x1*x2**3, x2**4])

def q_func(x):
    x1 = x[0]
    x2 = x[1]
    return x1**2 + x2**2 + 2*x2**4

ctrl.setPolicyParam(data_eval=data_eval, num_data=num_data, explore_noise=explore_noise, u0=u0, q_func=q_func, phi_func=phi_func)

Wc, Wa = ctrl.offPolicy()
print("The optimal weight of the critic")
print(Wc)
# for visualizing the results, using tensorboard