import numpy as np  
from scipy import integrate
from ..visualize import Logger 

class LTI():
    """
    This class present state-space LTI system.
    
    Attributes: 
        dimension (tuple): (n_state, n_input).
        model (dict): {A, B, C, D, dimension}.
        max_step (float, optional): define max step for ODEs solver algorithms. Defaults to 1e-3.
        algo (str, optional): RK45, RK23 or DOP853 . Defaults to 'RK45'.
        t_sim (tuple, optional): time for simualtion (start, stop). Defaults to (0,10).
        x0 (1xn array, optional): the initial state. Defaults to np.ones((n,)).
        sample_time (float, optional): the sample time. Defaults to 1e-2.
        
    
    """   
    def __init__(self, A, B, C=1, D=0):
        """Setup a LTI system in the form of state-space model

        Args:
            A (nxn array): the state matrix A
            B (nxm array): the input matrix B
            C (array, optional): the state output matrix C. Defaults to 1.
            D (array, optional): the input output matrix D. Defaults to 0.
            
        Note:
            The A, B matrix must be initialized with compatible dimension.
            
        """
        self.A = A
        self.B = B
        if len(self.B.shape)==1:
            self.B = np.expand_dims(self.B, axis=1)
            
        self.model_valid, self.dimension = self._check_model()
        self.C = C
        self.D = D
        self.model = {'A': self.A, 'B': self.B, 'C': C, 'D': D, 'dimension': self.dimension}
        
    def _check_model(self):          
        dimension = []
        model_valid = False
        a1,a2 = self.A.shape
        if a1 != a2:
            return model_valid
        else: n_states = a1
        
        b1,b2 = self.B.shape
        if b1 != a1:
            return model_valid 
        else: 
            n_inputs = b2
            model_valid=True
        dimension = [n_states, n_inputs]
        return model_valid, dimension

    def setSimulationParam(self, max_step=1e-3, algo='RK45', t_sim=(0,10), x0=None, sample_time = 1e-2):
        # fixed step_size
        """Run this function before any simulations

        Args:
            max_step (float, optional): define max step for ODEs solver algorithms. Defaults to 1e-3.
            algo (str, optional): RK45, RK23 or DOP853 . Defaults to 'RK45'.
            t_sim (tuple, optional): time for simualtion (start, stop). Defaults to (0,10).
            x0 (1xn array, optional): the initial state. Defaults to np.ones((n,)).
            sample_time (float, optional): the sample time. Defaults to 1e-2.
        """
        self.max_step = max_step
        self.algo = algo
        self.t_sim = t_sim
        if np.all(x0==None):
            self.x0 = np.ones((self.dimension[0],))
        else: self.x0 = x0
        self.sample_time = sample_time
       
    def integrate(self, x0, u, t_span):      
        if len(u.shape)==1:
            u = np.expand_dims(u, axis=1)
        
        dx_dt = lambda t,x,u: self.A.dot(x) + np.squeeze(self.B.dot(u), axis=1)
        
        result = integrate.solve_ivp(fun=dx_dt, args=(u,), y0=x0, t_span=t_span, method=self.algo, max_step=self.max_step, dense_output=True)

        return result.t, result.y.T
    
class NonLin():
    """This class represent non-linear system by ``ODEs``. 
    
    Attributes:
        dot_x (func(t,x,u)): the dx/dt function, return 1D array output
        dimension (tuple): (n_state, n_input) 
        max_step (float, optional): define max step for ODEs solver algorithms. Defaults to 1e-3.
        algo (str, optional): RK45, RK23 or DOP853 . Defaults to 'RK45'.
        t_sim (tuple, optional): time for simualtion (start, stop). Defaults to (0,10).
        x0 (1xn array, optional): the initial state. Defaults to np.ones((n,)).
        sample_time (float, optional): the sample time. Defaults to 1e-2.
    """
    def __init__(self, dot_x, dimension):
        """Setup non-linear system

        Args:
            dot_x (func(t,x,u)): the dx/dt function, return 1D array output
            dimension (tuple): (n_state, n_input) 
        """
        self.dot_x  = dot_x
        self.dimension = dimension      # (n_state, n_input)
        
    def setSimulationParam(self, max_step=1e-3, algo='RK45', t_sim=(0,10), x0=None, sample_time = 1e-2):
        """Run this function before any simulations

        Args:
            max_step (float, optional): define max step for ODEs solver algorithms. Defaults to 1e-3.
            algo (str, optional): RK45, RK23 or DOP853 . Defaults to 'RK45'.
            t_sim (tuple, optional): time for simualtion (start, stop). Defaults to (0,10).
            x0 (1xn array, optional): the initial state. Defaults to np.ones((n,)).
            sample_time (float, optional): the sample time. Defaults to 1e-2.
        """
        # fixed step_size
        self.max_step = max_step
        self.algo = algo
        self.t_sim = t_sim
        if np.all(x0==None):
            self.x0 = np.ones((self.dimension[0],))
        else: self.x0 = x0
        self.sample_time = sample_time
        
    def integrate(self, x0, u, t_span, t_eval=None):      
        if len(np.array(u).shape)==1:
            u = np.expand_dims(u, axis=1)
        
        result = integrate.solve_ivp(fun=self.dot_x, args=(u,), y0=x0, t_span=t_span, t_eval=t_eval, method=self.algo, max_step=self.max_step, dense_output=True)
        return result.t, result.y.T