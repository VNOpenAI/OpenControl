import numpy as np
from numpy.testing._private.utils import assert_array_max_ulp 
from scipy import integrate
import scipy.linalg
import scipy
from . import bibo
import matplotlib.pyplot as plt
class LTI():


    bibo_algorithm = [
        "gerschgorin",# : bibo.Gerschgorin,
        "lyapunov" ,#: bibo.Lyapunov,
        "hurwitz" ,
    ]
    bibo_result = {
        -1 : "System is not stable",
        0 : "Unable to conclude about system's stability",
        1 : "System is stable"
    }

    def __init__(self,**kwargs):
        """[summary]
        """
        
        assert "A" in kwargs, "matrix A must be provided"
        A = kwargs.get('A')
        B = kwargs.get('B')
        C = kwargs.get('C')
        D = kwargs.get('D')  
        for i in ['A','B','C','D']:   
            if kwargs.get(i) is not None:
                assert isinstance(kwargs.get(i),np.ndarray), f"Invalid data type of {i}"
                assert kwargs.get(i).ndim==2, f'Invlid ndim of matrix {i}, {i}.ndim must be 2'
        if B is not None:
            assert A.shape[0] == A.shape[1] and A.shape[0] ==B.shape[0]  , f'Invalid shape of matrix A,B, \n A.shape ={A.shape} and B.shape={B.shape}'
            self._inputs_shape = B.shape[1]
        self._A = A 
        self._B = B
        self._C = C 
        self._D = D
        self._states_shape = A.shape[0]
        
        #self._outputs_shape = C.shape[0]
        #input_function = kwargs.get('u')
        #self._x0 = kwargs.get('x0')
        #if self._x0 is not None:
        #    self._x0 = self._x0.reshape(-1,1)
        self._max_step = kwargs.get('max_step')
        
    @property
    def states_shape(self,) -> int:
        return self._states_shape


    @property
    def inputs_shape(self,) -> int:
        if hasattr(self,'_inputs_shape'):
            return self._inputs_shape
        else:
            return None

    @property
    def outputs_shape(self,) -> int:
        if hasattr(self,'_outputs_shape'):
            return self._outputs_shape
        else:
            return None
    @property
    def max_step(self):
        return self._max_step

    @property
    def A(self,):
        return self._A 
    @property
    def B(self,):
        return self._B 
    @property
    def C(self,):
        return self._C 
    @property
    def D(self):
        return self._D

    @property
    def dimension(self,) -> list:
        """[summary]

        Returns:
            list: [description]
        """
        return self.states_shape, self.inputs_shape, self.outputs_shape
    



    def eigvals(self):
        return scipy.linalg.eigvals(self._A)
    
    def is_stable(self,algorimth='hurwitz', **kwagrs) -> int:
        assert algorimth in LTI.bibo_algorithm, f"Invalid algorithm, must be  \
                                                        in {LTI.bibo_algorithm}"
        if algorimth=='gerschgorin': #Gerschgorin
            std = bibo.Gerschgorin(self._A)
            result = std.conclusion()
            print(LTI.bibo_result[result])     
            return result
        if algorimth=='lyapunov':
            P = kwagrs.get('P')
            Q = kwagrs.get('Q')
            std = bibo.Lyapunov(A=self._A, P=P, Q=Q)
            result = std.conclusion()
            print(LTI.bibo_result[result])     
            return result
        if algorimth=='hurwitz':
            std = bibo.Hurwitz(A=self._A)
            result = std.conclusion()
            print(LTI.bibo_result[result])     
            return result
      
  
        
    def is_controlable(self,algorimth='kalman', **kwagrs) -> bool:
        """[summary]

        Args:
            algorimth (str, optional): [description]. Defaults to 'kalman'.

        Raises:
            ValueError: [description]

        Returns:
            bool: [description]
        """
        if self._B is None:
            raise ValueError('please provide B matrix')
        A = self._A 
        B = self._B
        M = B
        ndim = self._states_shape
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
        #print(f'M.shape = {M.shape}')
        #print(M)
        #print(ndim)
        if np.linalg.matrix_rank(M)==ndim:
            return True 
        else:
            return False 

    def is_observable(self,) -> bool:
        assert self._C is not None, 'please fill matrix C to calculate observability'
        A = self._A
        C = self._C 
        M = C 
        ndim = self._states_shape
        if ndim==1:
            if np.linalg.matrix_rank(C) == 1:
                return True
            else:
                return False
        X = C @ A
        if ndim == 2:
            M = np.vstack([C,X])
        for i in range(ndim-2):
            X = X @ A 
            M = np.vstack([M,X])
        if np.linalg.matrix_rank(M)==ndim:
            return True 
        else: 
            return False


    def setup_simulink(self, max_step=1e-3, algo='RK45', t_sim=(0,10), x0=None, sample_time = 1e-2):
        # fixed step_size
        """Run this function before any simulations

        Args:
            max_step (float, optional): define max step for ODEs solver algorithms. Defaults to 1e-3.
            algo (str, optional): RK45, RK23 or DOP853 . Defaults to 'RK45'.
            t_sim (tuple, optional): time for simualtion (start, stop). Defaults to (0,10).
            x0 (1xn array, optional): the initial state. Defaults to np.ones((n,)).
            sample_time (float, optional): the sample time. Defaults to 1e-2.
        """
        if x0 is None: 
            x0 = np.ones((self.dimension[0],1))
        assert type(x0) is np.ndarray, "Invalid type, x0 must be numpy.ndarray"
        x0.shape = (-1,1)
        assert x0.shape[0]==self._states_shape, f"Invalid dimension, system got the states_shape = {self._states_shape}"
        self._x0 = x0

        self._max_step = max_step

        assert algo in ['RK45', 'RK23', 'DOP853'], "Invalid keyworks, algo must be in ['RK45', 'RK23', 'DOP853']"
        self.algo = algo
        self.t_sim = t_sim
        if np.all(x0==None):
            self.x0 = np.ones((self.dimension[0],1))
        else: 
            self._x0 = x0
        assert type(x0) is np.ndarray, "x0 must be np.ndarray"
        assert x0.shape==(self.dimension[0],1), "invalid shape of"
        self.sample_time = sample_time

    def step_response(self,input_function=None,logs_file =None):
        sample_time = self.sample_time 
        time_start = self.t_sim[0]
        time_stop = self.t_sim[-1]      

        if input_function is None:
            input_function = lambda t: np.zeros((self._inputs_shape,1))
        x0 = self._x0
        def function_x (t,x):
            out = self._A @ x.reshape(-1,1) + self._B @ (input_function(t).reshape(-1,1))
            return out.ravel()
        def function_out(t,x):      
            return self._C @ x + self._D @ input_function(t) 

        i_old = time_start 
        time_array = np.linspace(time_start+sample_time,time_stop,int((time_stop-time_start)//sample_time))
        
        x_out = []
        for i in time_array:
            #print(x0.shape)
            result = scipy.integrate.solve_ivp(function_x, t_span=(i_old,i), y0 = x0.reshape(-1),max_step=self.max_step,method=self.algo)
            x0 = result.y.T[-1]  #x0 shape (n,)
            i_old = i
            x_out.append(x0) 
        if self._C is not None:
            y_out = [function_out(i,x) for x in x_out]
            y_out = np.array(y_out)
        else: 
            y_out = None

        x_out = np.array(x_out)
        x_out = x_out.T
        if logs_file is not None:
            pass

        return x_out ,y_out# ndim, num_time_point
    





        
    def state_feed_back(self,pole,algorithms='Roppernecker', **kwargs):
        """[summary]

        Args:
            pole ([type]): [description]
            algorithms (str, optional): [description]. Defaults to 'Roppernecker'.

        Raises:
            ValueError: [description]

        Returns:
            [type]: [description]
        """
        controlability = self.is_controlable()
        if not controlability:
            print('system is not controlalbe')
            return 
        if self._B is None:
            raise ValueError('matrix B must be provided')
        eigvals,eigvectors = np.linalg.eig(self.A)

       
        a= []
        t = []
        for index,s in enumerate(pole):

            if s in eigvals:
                a.append(eigvectors[:,index].reshape(-1,1))
                t_index = np.zeros((self.inputs_shape,1))
                #t_index[index] = 1
                t.append(t_index)
                #print('yes:', eigvectors[:,index].reshape(-1,1))
            else:
                a.append( np.linalg.inv(s*np.eye(self.states_shape)- self.A)@self.B)
                t_index = np.ones((self.inputs_shape,1))
                t.append(t_index)
                #print('yes:', eigvectors[:,index].reshape(-1,1))
                
        a = np.concatenate(a,axis=1)
        t = np.concatenate(t,axis=1)
        print(t)
        print(a)
        R = - t @ np.linalg.inv(a)
        return R
        

    def apply_state_feedback(self,R,t_sim = (0,10), sample_time=0.1,save_fig='apply_state_feedback'):
        """[summary]

        Args:
            R ([type]): [description]
            time_start (int, optional): [description]. Defaults to 0.
            time_stop (int, optional): [description]. Defaults to 10.
            sample_time (float, optional): [description]. Defaults to 0.1.
            save_fig (str, optional): [description]. Defaults to 'apply_state_feedback'.

        Returns:
            [type]: [description]
        """
        time_start = t_sim[0]
        time_stop = t_sim[-1]
        A_old = self._A 
        self._A = self._A - self._B@R
        out =  self.step_response(time_start=time_start, time_stop=time_stop, sample_time=sample_time,file_png=save_fig)
        self._A = A_old 
        return out
        
