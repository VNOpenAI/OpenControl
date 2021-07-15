### This examples the simulation examples in the book of Zong Ping Jiang and Yu Jiang, chapter 3. We simulate with OpenControl, Off Policy ADP Controller. The results that is the same as the book's results demonstrate our python packages.

### you can check the book at https://www.wiley.com/en-us/Robust+Adaptive+Dynamic+Programming-p-9781119132646

### run simulation with the bash script: python CarSuspension.py

from OpenControl.ADP_control import NonLin, NonLinController
from matplotlib import pyplot as plt
import numpy as np

# the car suspension system
def dot_x(t,x,u):
    # Dynamics of the syspension system
    x1 = x[0]
    x2 = x[1]
    x3 = x[2]
    x4 = x[3]

    # Coefficients
    mb = 300    # kg
    mw = 60     # kg
    bs = 1000   # N/m/s
    ks = 16000 # N/m
    kt = 190000 # N/m
    kn = ks/10  # N/m

    # System Dynamics
    dx1 = x2
    dx2 = -(ks*(x1-x3)+kn*(x1-x3)**3)/mb - (bs*(x2-x4)-10000*u)/mb
    dx3 = x4
    dx4 =  (ks*(x1-x3)+kn*(x1-x3)**3)/mw + (bs*(x2-x4)-kt*x3-10000*u)/mw

    # Combine the output
    dx = [dx1, dx2, dx3, dx4]
    return dx   

dimension = (4,1)
system = NonLin(dot_x, dimension)

t_sim = (0,5)
algo = 'RK45'
max_step = 1e-3
sample_time = 1e-3
x0 = [0.1,-5,0.2,2]
system.setSimulationParam(max_step=max_step, algo=algo, t_sim=t_sim, x0=x0, sample_time=sample_time)

Controller = NonLinController(system)
u0 = lambda x: 0
data_eval = 0.01
num_data = 80       # at leats n_phi+n_psi
explore_noise = lambda t: 0.2*np.sum(np.sin(np.array([1, 3, 7, 11, 13, 15])*t))
Controller.setPolicyParam(data_eval=data_eval, num_data=num_data, explore_noise=explore_noise, u0=u0)

Controller.feedback()
Wc, Wa = Controller.offPolicy()

xplot = np.array(Controller.x_plot)
tplot = np.array(Controller.t_plot)
xplot_unlearn = np.array(Controller.x_plot_unlearn)
tplot_unlearn = np.array(Controller.t_plot_unlearn)

len = xplot.shape[0]
data =np.vstack((xplot[:,0],xplot_unlearn[:len,0])).T

plt.plot(tplot, data)
# plt.plot(tplot, xplot[:,0])
plt.show()