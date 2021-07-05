import numpy as np
from numpy.testing._private.utils import assert_array_max_ulp 
from scipy import integrate
from . import bibo
import matplotlib.pyplot as plt
class LinearSystem():


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
            assert A.shape[0] == A.shape[1] and A.shape[0] ==B.shape[0]  , f'Invalid shape of matrix A,B'
            self._inputs_shape = B.shape[1]
        self._A = A 
        self._B = B
        self._C = C 
        self._D = D
        self._states_shape = A.shape[0]
        
        #self._outputs_shape = C.shape[0]
        self._u = kwargs.get('u')
        self._x0 = kwargs.get('x0')
        if self._x0 is not None:
            self._x0 = self._x0.reshape(-1,1)
        
        
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


    def get_dimension(self,) -> list:
        """[summary]

        Returns:
            list: [description]
        """
        return self.states_shape, self.inputs_shape, self.outputs_shape

    def eigvals(self):
        return scipy.linalg.eigvals(self._A)
    
    def is_stable(self,algorimth='hurwitz', **kwagrs) -> int:
        assert algorimth in LinearSystem.bibo_algorithm, f"Invalid algorithm, must be  \
                                                        in {LinearSystem.bibo_algorithm}"
        if algorimth=='gerschgorin': #Gerschgorin
            std = bibo.Gerschgorin(self._A)
            result = std.conclusion()
            print(LinearSystem.bibo_result[result])     
            return result
        if algorimth=='lyapunov':
            P = kwagrs.get('P')
            Q = kwagrs.get('Q')
            std = bibo.Lyapunov(A=self._A, P=P, Q=Q)
            result = std.conclusion()
            print(LinearSystem.bibo_result[result])     
            return result
        if algorimth=='hurwitz':
            std = bibo.Hurwitz(A=self._A)
            result = std.conclusion()
            print(LinearSystem.bibo_result[result])     
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
    
    def set_input_function(self, function):
        self._u = function

    def initialize_state(self, x0):
        assert type(x0) is np.ndarray, "Invalid type, x0 must be numpy.ndarray"
        x0.shape = (-1,1)
        assert x0.shape[0]==self._states_shape, f"Invalid dimension, system got the states_shape = {self._states_shape}"
        self._x0 = x0

    def simulink(self,time_start=0, time_stop=10, time_step=0.1,file_png ='similink'):
        assert self._x0 is not None, "please set initial state x0"
        #assert self._u is not None, "please set input before running simulink"
        if self._u is None:
            self._u = lambda t: np.zeros((self._inputs_shape,1))
        x0 = self._x0
        def function_x (t,x):
            out = self._A @ x.reshape(-1,1) + self._B @ (self._u(t).reshape(-1,1))
            return out.ravel()
        def function_out(t,x):
            
            return self._C @ x + self._D @ self._u(t) 

        i_old = time_start 
        time_array = np.linspace(time_start+time_step,time_stop,int((time_stop-time_start)//time_step))
        
        x_out = []
        for i in time_array:
            #print(x0.shape)
            result = scipy.integrate.solve_ivp(function_x, t_span=(i_old,i), y0 = x0.reshape(-1))
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
        print(x_out.shape)
        for num_state in range(len(x_out)):
            plt.plot(time_array.ravel(),x_out[num_state].ravel())
            plt.savefig(file_png+'_{:03d}'.format(num_state))
        return x_out ,y_out# ndim, num_time_point

    def plot_state():
        pass
        
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
        #print(t)
        #print(a)
        R = - t @ np.linalg.inv(a)
        return R
        

    def apply_state_feedback(self,R,time_start=0, time_stop=10, time_step=0.1,save_fig='apply_state_feedback'):
        """[summary]

        Args:
            R ([type]): [description]
            time_start (int, optional): [description]. Defaults to 0.
            time_stop (int, optional): [description]. Defaults to 10.
            time_step (float, optional): [description]. Defaults to 0.1.
            save_fig (str, optional): [description]. Defaults to 'apply_state_feedback'.

        Returns:
            [type]: [description]
        """
        A_old = self._A 
        self._A = self._A - self._B@R
        out =  self.simulink(time_start=time_start, time_stop=time_stop, time_step=time_step,file_png=save_fig)
        self._A = A_old 
        return out
        
