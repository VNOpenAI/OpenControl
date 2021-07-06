import numpy as np
from scipy import integrate
from control import lqr
from ..visualize import Logger
from .system import LTI,NonLin

class LTIController():
    """This present continuous controller for LTI System
    
        Attributes:
            system (LTI class): the object of LTI class
            logX (Logger class): the object of Logger class, use for logging state signals
            K0 (mxn array, optional): The initial value of K matrix. Defaults to np.zeros((m,n)).
            Q (nxn array, optional): The Q matrix. Defaults to 1.
            R (mxm array, optional): The R matrix. Defaults to 1.
            data_eval (float, optional): data_eval x num_data = time interval for each policy updation. Defaults to 0.1.
            num_data (int, optional): the number of data for each learning iteration. Defaults to 10.
            explore_noise (func(t), optional): The exploration noise within the learning stage. Defaults to lambda t:2*np.sin(100*t).
            logK (Logger class): logger of the K matrix
            logP (Logger class): logger of the P matrix
            t_plot, x_plot (float, array): use for logging, plotting simulation result
            viz (boolean): True for visualize results on ``Tensorboard``. Default to True
    """
    def __init__(self, system):
        """Design a controller for the system input.

        Args:
            system (LTI class): Call this class after system.setSimulationParams()
        """
        self.system = system
        self.model = system.model
        self.A = self.model['A']
        self.B = self.model['B']
        self.dimension = self.model['dimension']
        self.logX = Logger(log_dir='results')
        
    def step(self, x0, u, t_span):
        """Step respond of the system.

        Args:
            x0 (1D array): initial state for simulation
            u (1D array): the value of input within t_span 
            t_span (list): (t_start, t_stop)

        Returns:
            list, 2D array: t_span, state at t_span (x_start, x_stop)
        """
        # u is the function of t and x (feedback law)
        dx_dt = lambda t,x: self.A.dot(x) + self.B.dot(u(t,x))
        result = integrate.solve_ivp(fun=dx_dt, y0=x0, t_span=t_span, method=self.system.algo, max_step=self.system.max_step, dense_output=True)
        return result.t, result.y.T        
        
    def LQR(self, Q=None, R=None):
        """This function solve Riccati function with defined value function

        Args:
            Q (nxn array optional): the Q matrix. Defaults to 1.
            R (mxm arary, optional): the R matrix. Defaults to 1.

        Returns:
            mxn array, nxn array: the K, P matrix
        """
        if np.all(Q==None):
            Q = np.eye(self.A.shape[0])
 
        if np.all(R==None):
            R = np.eye(self.B.shape[1])
            
        K,P,E = lqr(self.A, self.B, Q, R)
        
        return K, P
        
    def _isStable(self, A):
        eig = np.linalg.eig(A)[0].real
        return np.all(eig<0)
            
    def onPolicy(self, stop_thres=1e-3, viz=True):
        """Using On-policy approach to find optimal adaptive feedback controller, requires only the dimension of the system 

        Args:
            stop_thres (float, optional): threshold value to stop iteration. Defaults to 1e-3.
            viz (bool, optional): True for logging data. Defaults to True.

        Raises:
            ValueError: raise when the user-defined number of data is not enough, make rank condition unsatisfied

        Returns:
            mxn array, nxn array: the optimal K, P matrix
        """
        #online data collection
        self.viz = viz
        x_plot = [self.system.x0]       # list of array
        t_plot = [self.system.t_sim[0]]
        save_P = []
        save_K = [self.K0]
        K = save_K[-1]
        iter = 0
        
        while t_plot[-1] < self.system.t_sim[1]:
            u = lambda t,x: -K.dot(x) + self.explore_noise(t)
            
            theta = []
            xi = []
            # collect data_per_eval x data, iterate within an eval
            for i in range(self.num_data):
                x_sample = [x_plot[-1]]
                t_sample = [t_plot[-1]]
                t_collect = t_plot[-1]
                
                # collect online data, iterate within a sample time
                while t_plot[-1] < t_collect + self.data_eval:
                    t_temp, x_temp = self.step(x0=x_plot[-1], u=u, t_span=(t_plot[-1], t_plot[-1] + self.system.sample_time))
                    if self.viz:
                        self.logX.log('states_onPolicy', x_temp[-1], int(t_temp[-1]/self.system.sample_time))
                        
                    x_sample.append(x_temp[-1])
                    t_sample.append(t_temp[-1])
                    
                    x_plot.extend(x_temp[1:].tolist())
                    t_plot.extend(t_temp[1:].tolist())   
                    
                thetaRow, xiRow = self._rowGainOnPloicy(K, x_sample, t_sample)
                theta.append(thetaRow)           
                xi.append(xiRow)
            
            theta = np.array(theta)
            xi = np.array(xi)
            
            # check rank condition
            n,m = self.dimension
            if np.linalg.matrix_rank(theta) < m*n + n*(n+1)/2:
                print('not enough number of data, rank condition unsatisfied!')
                raise ValueError
            
            # solve P, K matrix
            PK = np.linalg.pinv(theta).dot(xi)
            P = PK[:n*n].reshape((n,n))
            if self.viz:
                self.logP.log('P_onPolicy', P, iter)
                self.logK.log('K_onPolicy', K, iter)
            save_P.append(P)
            # check stopping criteria
            try:
                err = np.linalg.norm(save_P[-1]-save_P[-2], ord=2)
                if err < stop_thres:
                    self.t_plot, self.x_plot = self._afterGainKopt(t_plot, x_plot, K, 'states_onPolicy')
                    break
            except: pass
            K = PK[n*n:].reshape((n,m)).T
            save_K.append(K)
            iter += 1
        return save_K[-1], save_P[-1]

    def _afterGainKopt(self, t_plot, x_plot, Kopt, section):
        # remove explore noise
        u = lambda t,x: -Kopt.dot(x)
        sample_time = self.system.sample_time
        start = t_plot[-1]
        stop = self.system.t_sim[1] 
        N = int((stop - start)/sample_time)
        for i in range(N):
            t_temp, x_temp = self.step(x0=x_plot[-1], u=u, t_span=(t_plot[-1], t_plot[-1] + sample_time))
            if self.viz:
                self.logX.log(section, x_temp[-1], int(t_temp[-1]/self.system.sample_time))
            x_plot.extend(x_temp[1:].tolist())
            t_plot.extend(t_temp[1:].tolist())  
                
        return t_plot, x_plot
                
    def _rowGainOnPloicy(self, K, x_sample, t_sample):
        xx = np.kron(x_sample[-1], x_sample[-1]) - np.kron(x_sample[0], x_sample[0])
        xeR = []
        xQx = []
        Qk = self.Q + K.T.dot(self.R.dot(K))
        # print(np.array(x_sample).shape)
        for i, xi in enumerate(x_sample):
            # print(type(xi))
            xi = np.array(xi)
            ei = self.explore_noise(t_sample[i])
            xeR.append(np.kron(xi, np.dot(ei,(self.R)).squeeze()))
            xQx.append(xi.dot(Qk.dot(xi)))
        
        xeR = -2*integrate.simpson(xeR, t_sample, axis=0)
        # xeR = -2*np.trapz(xeR, dx=sample_time, axis=0)
        _thetaRow = np.hstack((xx, xeR))
        _xiRow = -integrate.simpson(xQx, t_sample, axis=0)
        # _xiRow = -np.trapz(xQx, dx=sample_time, axis=0)
        return _thetaRow, _xiRow
        
    def setPolicyParam(self, K0=None, Q=None, R=None, data_eval=0.1, num_data=10, explore_noise=lambda t: 2*np.sin(100*t)):
        """Setup policy parameters for both the On (Off) policy algorithms. Initalize logger for K, P matrix

        Args:
            K0 (mxn array, optional): The initial value of K matrix. Defaults to np.zeros((m,n)).
            Q (nxn array, optional): The Q matrix. Defaults to 1.
            R (mxm array, optional): The R matrix. Defaults to 1.
            data_eval (float, optional): data_eval x num_data = time interval for each policy updation. Defaults to 0.1.
            num_data (int, optional): the number of data for each learning iteration. Defaults to 10.
            explore_noise (func(t), optional): The exploration noise within the learning stage. Defaults to lambda t:2*np.sin(100*t).

        Raises:
            ValueError: raise when the initial value of the K matrix is not admissible
            
        Note: 
            - The K0 matrix must be admissible
            - data_eval must be larger than the sample_time
            - num_data >= n(n+1) + 2mn
        """
        if np.all(Q==None):
            Q = np.eye(self.A.shape[0])
            
        if np.all(R==None):
            R = np.eye(self.B.shape[1])
        
        if np.all(K0==None):
            K0 = np.zeros(self.dimension).T
        if len(K0.shape)==1:
            K0 = np.expand_dims(K0, axis=0)
        # check stable of K0 
        if not self._isStable(self.A - self.B.dot(K0)):
            print('the inital K0 matrix is not stable, try re-initialize K0')
            raise ValueError
        
        self.K0 = K0
        self.Q = Q
        self.R = R
        self.data_eval = data_eval
        self.num_data = num_data
        self.explore_noise = explore_noise
        
        self.logK = Logger('results')
        self.logP = Logger('results')
            
    def offPolicy(self, stop_thres=1e-3, max_iter=30, viz=True):
        """Using Off-policy approach to find optimal adaptive feedback controller, requires only the dimension of the system 

        Args:
            stop_thres (float, optional): threshold value to stop iteration. Defaults to 1e-3.
            viz (bool, optional): True for logging data. Defaults to True.
            max_iter (int, optional): the maximum number of policy iterations. Defaults to 30.

        Raises:
            ValueError: raise when the user-defined number of data is not enough, make rank condition unsatisfied

        Returns:
            mxn array, nxn array: the optimal K, P matrix
        """
        self.viz = viz
        self.stop_thres = stop_thres 
        self.max_iter = max_iter
        save_K = [self.K0]
        save_P = []

        x_plot = [self.system.x0]       # list of array
        t_plot = [self.system.t_sim[0]]
        
        u = lambda t,x: -self.K0.dot(x) + self.explore_noise(t)
        dxx = []
        Ixx = []
        Ixu = []
        for i in range(self.num_data):
            x_sample = [x_plot[-1]]
            t_sample = [t_plot[-1]]
            #collect data, iterate within data eval 
            t_collect = t_plot[-1]
            while t_plot[-1] < t_collect + self.data_eval:
                t_temp, x_temp = self.step(x0=x_plot[-1], u=u, t_span=(t_plot[-1], t_plot[-1] + self.system.sample_time))
                if self.visulize:
                    self.logX.log('states_offPolicy', x_temp[-1], int(t_temp[-1]/self.system.sample_time))
                
                x_sample.append(x_temp[-1])
                t_sample.append(t_temp[-1])
                
                x_plot.extend(x_temp[1:].tolist())
                t_plot.extend(t_temp[1:].tolist()) 
        
            dxx_ , Ixx_, Ixu_ = self._getRowOffPolicyMatrix(t_sample, x_sample)
            dxx.append(dxx_)
            Ixx.append(Ixx_)
            Ixu.append(Ixu_)
            
        # check rank condition
        test_matrix = np.hstack((Ixx, Ixu))
        n,m = self.dimension
        if np.linalg.matrix_rank(test_matrix) < m*n + n*(n+1)/2:
            print('not enough data, rank condition is not satisfied')
            raise ValueError
        
        # find optimal solution
        save_K, save_P = self._policyEval(dxx, Ixx, Ixu)
        self.Kopt = save_K[-1]
        self.t_plot, self.x_plot = self._afterGainKopt(t_plot, x_plot, self.Kopt, 'states_offPolicy')        
        # return optimal policy
        return save_K[-1], save_P[-1]
            
    def _policyEval(self, dxx, Ixx, Ixu):
        dxx = np.array(dxx)     # n_data x (n_state^2)
        Ixx = np.array(Ixx)     # n_data x (n_state^2)
        Ixu = np.array(Ixu)     # n_data x (n_state*n_input)
        
        save_K = [self.K0]
        save_P = []
        n,m = self.dimension
        K = save_K[-1]      # mxn
        for i in range(self.max_iter):
            temp = -2*Ixx.dot(np.kron(np.eye(n), K.T.dot(self.R)))  - 2*Ixu.dot(np.kron(np.eye(n), self.R))
            theta = np.hstack((dxx, temp))
            Qk = self.Q + K.T.dot(self.R.dot(K))
            xi = -Ixx.dot(Qk.ravel())
            
            PK = np.linalg.pinv(theta).dot(xi)
            P = PK[:n*n].reshape((n,n))
            if self.viz:
                self.logP.log('P_offPolicy', P, i)
                self.logK.log('K_offPolicy', K, i)
            save_P.append(P)
            # check stopping criteria
            try: 
                err = np.linalg.norm(save_P[-1] - save_P[-2], ord=2)
                if err < self.stop_thres:
                    break 
            except: pass
            K = PK[n*n:].reshape((n,m)).T
            save_K.append(K)
            
        return save_K, save_P   
        
    def _getRowOffPolicyMatrix(self, t_sample, x_sample):
        u = lambda t,x: -self.K0.dot(x) + self.explore_noise(t)

        dxx_ = np.kron(x_sample[-1], x_sample[-1]) - np.kron(x_sample[0], x_sample[0])
        xx = []
        xu = []
        for i, xi in enumerate(x_sample):
            xx.append(np.kron(xi, xi))
            ui = u(t_sample[i], xi)
            xu.append(np.kron(xi, ui))
            
        Ixx_ = integrate.simpson(xx, t_sample, axis=0)
        Ixu_ = integrate.simpson(xu, t_sample, axis=0)
        return dxx_, Ixx_, Ixu_


class NonLinController():
    """This present continuous controller for Non-Linear System
    
        Attributes:
            system (nonLin class): the object of nonLin class
            logX (Logger class): the object of Logger class, use for logging state signals
            u0 (func(x), optional): The initial feedback control policy. Defaults to 0.
            q_func (func(x), optional): the function :math:`q(x)`. Defaults to nonLinController.default_q_func.
            R (mxm array, optional): The R matrix. Defaults to 1.
            phi_func (list of func(x), optional): the sequences of basis function to approximate critic, :math:`\phi_j(x)`. Defaults to nonLinController.default_phi_func
            psi_func (list of func(x), optional): the sequences of basis function to approximate actor, :math:`\psi_j(x)`. Defaults to nonLinController.default_psi_func
            data_eval (float, optional): data_eval x num_data = time interval for each policy updation. Defaults to 0.1.
            num_data (int, optional): the number of data for each learning iteration. Defaults to 10.
            explore_noise (func(t), optional): The exploration noise within the learning stage. Defaults to lambda t:2*np.sin(100*t).
            logWa (Logger class): logging to value of the weight of the actor 
            logWc (Logger class): logging to value of the weight of the critic
            t_plot, x_plot (float, array): use for logging, plotting simulation result      
    """
    def __init__(self, system):
        """Design a controller for the system 

        Args:
            system (nonLin class): Call this class after system.setSimulationParams()
        """
        self.system = system
        self.dot_x = self.system.dot_x
        self.logX = Logger('results')
        
    def setPolicyParam(self, q_func=None, R=None, phi_func=None, psi_func=None, u0=lambda x: 0, data_eval=0.1, num_data=10, explore_noise=lambda t: 2*np.sin(100*t)):
        """Setup policy parameters for both the On (Off) policy algorithms. Initalize logger for K, P matrix

        Args:
            q_func (func(x), optional): the function :math:`q(x)`. Defaults to nonLinController.default_q_func
            R (mxm array, optional): The R matrix. Defaults to 1.
            phi_func (list of func(x), optional): the sequences of basis function to approximate critic, :math:`\phi_j(x)`. Defaults to nonLinController.default_phi_func
            psi_func (list of func(x), optional): the sequences of basis function to approximate actor, :math:`\psi_j(x)`. Defaults to nonLinController.default_psi_func
            u0 (func(x), optional): The initial feedback control policy. Defaults to 0.
            data_eval (float, optional): data_eval x num_data = time interval for each policy updation. Defaults to 0.1.
            num_data (int, optional): the number of data for each learning iteration. Defaults to 10.
            explore_noise (func(t), optional): The exploration noise within the learning stage. Defaults to lambda t:2*np.sin(100*t).
            
        Note:
            - u0 must be admissible controller
            - the squences of basis functions :math:`\phi_j(x), \psi_j(x)` should be in the form of *linearly independent smooth* 
            - data_eval must be larger than the sample_time
            - num_data >= n(n+1) + 2mn
        """
        if q_func==None:
            self.q_func = NonLinController.default_q_func
        else:   self.q_func = q_func                # positive definite function
        if R==None:
            self.R = np.eye(self.system.dimension[1])
        else:   self.R = R                # symmetric and positive definite matix (1x(mxm))
        if phi_func==None:
            self.phi_func = NonLinController.default_phi_func
        else:   self.phi_func = phi_func    # basis function for value function
        if psi_func==None:
            self.psi_func = NonLinController.default_psi_func
        else:   self.psi_func = psi_func    # basis function for policy function        
        self.u0 = u0                
        self.data_eval = data_eval
        self.num_data = num_data
        self.explore_noise = explore_noise   
        
        self.logWa = Logger('results')   
        self.logWc = Logger('results')
    
    def step(self, dot_x, x0, t_span):
        """Step respond of the no-input system

        Args:
            dot_x (func(x)): no-input ODEs function
            x0 (1D array): the initial state
            t_span (tuple): (t_start, t_stop)

        Returns:
            list, 2D array: t_span, state at t_span (x_start, x_stop)
        """
        result = integrate.solve_ivp(fun=dot_x, t_span=t_span, y0=x0, method=self.system.algo, max_step=self.system.max_step, dense_output=True)
        return result.t, result.y.T
      
    def offPolicy(self, stop_thres=1e-3, max_iter=30, visualize=True):
        """Using Off-policy approach to find optimal adaptive feedback controller, requires only the dimension of the system 

        Args:
            stop_thres (float, optional): threshold value to stop iteration. Defaults to 1e-3.
            viz (bool, optional): True for logging data. Defaults to True.
            max_iter (int, optional): the maximum number of policy iterations. Defaults to 30.

        Returns:
            array, array: the final updated weight of critic, actor neural nets.  
        """
        self.visualize = visualize
        self.stop_thres = stop_thres
        self.max_iter = max_iter
        # collect data
        u = lambda t,x: self.u0(x) + self.explore_noise(t)
        dot_x = lambda t,x: self.dot_x(t, x, u(t,x))
        
        x_plot = [self.system.x0]
        t_plot = [self.system.t_sim[0]]
        
        dphi = [] 
        Iq = []
        Iupsi = []
        Ipsipsi = []
        for i in range(self.num_data):
            x_sample = [x_plot[-1]]
            t_sample = [t_plot[-1]]
            t_collect = t_plot[-1]
            while t_plot[-1] < t_collect + self.data_eval:
                t_span = (t_plot[-1], t_plot[-1] + self.system.sample_time)
                t_temp, x_temp = self.step(dot_x, x_plot[-1], t_span)
                if self.visualize:
                    self.logX.log('states_offPolicy', x_temp[-1], int(t_temp[-1]/self.system.sample_time))
                    
                t_sample.append(t_temp[-1])
                x_sample.append(x_temp[-1])
                
                x_plot.extend(x_temp[1:].tolist()),
                t_plot.extend(t_temp[1:].tolist())
                
            dphi_, Iq_, Iupsi_, Ipsipsi_ = self._getRowOffPolicyMatrix(np.array(t_sample), np.array(x_sample))
            dphi.append(dphi_)
            Iq.append(Iq_)
            Iupsi.append(Iupsi_)
            Ipsipsi.append(Ipsipsi_)

        # solve policy 
        save_Wc, save_Wa = self._policyEval(np.array(dphi), np.array(Iq), np.array(Iupsi), np.array(Ipsipsi))
        Waopt = save_Wa[-1]
        t_plot, x_plot = self._afterGainWopt(t_plot, x_plot, Waopt, 'states_offPolicy')

        return save_Wc[-1], save_Wa[-1]
        
    def _afterGainWopt(self, t_plot, x_plot, Waopt, section):
        u = lambda t,x: Waopt.dot(self.psi_func(x))
        dot_x = lambda t,x: self.dot_x(t, x, u(t,x))
        sample_time = self.system.sample_time
        start = t_plot[-1]
        stop = self.system.t_sim[1]
        N = int((stop - start)/sample_time)
        for i in range(N):
            t_temp, x_temp = self.step(dot_x, x_plot[-1], (t_plot[-1], t_plot[-1]+sample_time))
            if self.visualize:
                self.logX.log(section, x_temp[-1], int(t_temp[-1]/self.system.sample_time))
            t_plot.extend(t_temp[1:].tolist())
            x_plot.extend(x_temp[1:].tolist())
        
        return t_plot, x_plot
            
    def _policyEval(self, dphi, Iq, Iupsi, Ipsipsi):
        n_psi = len(self.psi_func(self.system.x0))
        n_phi = len(self.phi_func(self.system.x0))
        n_input = self.system.dimension[1]
        Wa = np.zeros((n_input, n_psi))
        
        save_Wc = []
        save_Wa = [Wa]
        for i in range(self.max_iter): 
            temp = -2*(Iupsi - Ipsipsi.dot(np.kron(Wa.T, np.eye(n_psi))))
            A = np.hstack((dphi, temp))
            B = Iq + Ipsipsi.dot(Wa.T.dot(self.R.dot(Wa)).flatten()) 
            Wca = np.linalg.pinv(A).dot(B)
            Wc = Wca[:n_phi]
            if self.visualize:
                self.logWc.log('offPolicy_Wc', Wc, i)
                self.logWa.log('offPolicy_Wa', Wa, i)
            try: 
                err = np.linalg.norm(save_Wc[-1] - save_Wc[-2], ord=2)
                if err < self.stop_thres:
                    break
            except: pass
            Wa = Wca[n_phi:].reshape((n_input, n_psi)).T.dot(np.linalg.inv(self.R)).T
            save_Wc.append(Wc)
            save_Wa.append(Wa)
        
        return save_Wc, save_Wa
        
    def _getRowOffPolicyMatrix(self, t_sample, x_sample):
        dphi_ = self.phi_func(x_sample[-1]) - self.phi_func(x_sample[0])
        Iq_ = []
        Iupsi_ = []
        Ipsipsi_ = []
        u = lambda t,x: self.u0(x) + self.explore_noise(t)
        for i, xi in enumerate(x_sample):
            Iq_.append(self.q_func(xi))
            Iupsi_.append(np.kron(u(t_sample[i], xi), self.psi_func(xi)))
            Ipsipsi_.append(np.kron(self.psi_func(xi), self.psi_func(xi)))
            
        Iq_ = integrate.simpson(Iq_, t_sample, axis=0)
        Iupsi_ = integrate.simpson(Iupsi_, t_sample, axis=0)
        Ipsipsi_ = integrate.simpson(Ipsipsi_, t_sample, axis=0)
        return dphi_, Iq_, Iupsi_, Ipsipsi_
        
               
    @staticmethod    
    def default_psi_func(x):
        """The default sequences of basis functions to approximate actor

        Args:
            x (1xn  array): the state vector

        Returns:
            list func(x): see the **Controller Design / Problem statements**
        """
        psi = []
        for i in range(len(x)):
            psi.append(x[i])
            for j in range(i, len(x)):
                for k in range(j, len(x)):
                    psi.append(x[i]*x[j]*x[k])
        return np.array(psi)
    @staticmethod
    def default_phi_func(x):
        """The default sequences of basis functions to approximate critic

        Args:
            x (1xn  array): the state vector

        Returns:
            list func(x): see the **Controller Design / Problem statements**
        """
        phi = []
        for i in range(len(x)):
            for j in range(i, len(x)):
                phi.append(x[i]*x[j])
                phi.append(x[i]**2*x[j]**2)
        return np.array(phi)
    @staticmethod
    def default_q_func(x):
        """The default function of the q(x) function

        Args:
            x (1D array): the state vector

        Returns:
            float: :math:`x^Tx`
        """
        return np.sum(x*x, axis=0)