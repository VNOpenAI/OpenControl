********************************
Classic Control
********************************

About
================================================================

module classiccontrol is the fundamental package needed for implementation control algorithms in  with Python.

the main object of module is LTI system and relative algorithms, controller,...
The Linear time-invariant system formulate in:

.. math:: 
    \dot{x} = Ax + Bu \\\\
    y = Cx + Du


This package contains:
    - **linearsystem.py**: implement class LTI and basic attributes of system, 
            - system matrix
            - dimension
            - poles of system 
            - controllability and observability:
                - Kalman standard
                - Hatus standard
            - simulation with controller and observer   
                

    - **bibo.py**: implement algorithms for determining stability of LTI system
            - Gerschgorin
            - Lyapunov
            - Hurwitz

    - **controller.py**: Design controller for LTI
            - Pole statement:
                - Roppernecker method
                - Arckerman method
                - Modal method
            - Optimal control:
                - LQR
    
    - **observer.py**: Design observer for LTI 
            - Luenberger observer
            - Kalman observer
    
    - **utils.py**: collection of small and common Python functions which re-use a lots in difference package 


This manual contains many examples of use, usually prefixed with the Python prompt >>> (which is not a part of the example code). The examples assume that you have first entered:

.. code-block:: python

    from OpenControl import classiccontrol

Declare LTI system:

.. code-block:: python

    import numpy as np 

    A = np.array([[0,          1,    0,     0],
                  [28.0286,    0,    0,     0],
                  [0,          0,    0,     1],
                  [-1.4014,    0,    0,     0]])

    B = np.array([[    0     ],
                    -1.5873  ],
                       0     ],
                     0.6349  ]])

    C = np.array([[1,    0,    0,     0],
                  [0,    1,    0,     0],
                  [0,    0,    1,     0],
                  [0,    0,    0,     1]])

    sys = classiccontrol.linearsystem.LTI(A=A, B=B, C=C)

Controllability and observability
================================================================

Kalman standard:
    **Theorem**: The linear continuous-time system is controllable if and only if the controllability matrix has full rank.

    The observability matrix, in this case, defined by:



    **Theorem**: The linear continuous-time system is controllable if and only if the controllability matrix has full rank.

    The controllability matrix, in this case, defined by:


Hatus standard: 
    System is controllable if matrix: is full rank with any s.

    System is observable if matrix:  is full rank 

Stability
================================================================

Gerschgorin: 
    A is system matrix. Elements of A, aij. 
    Ri(A) = sum(abs(aij)) 

Hurwitz: 
    system is stable if all the eigen values of system matrix on the left of complex coordinate

Lyapunov:


Stability of system:

.. code-block:: python

    #select the algorithms 

    sys.is_stable(algorimth='gerschgorin')
    sys.is_stable(algorimth='hurwitz')
    sys.is_stable(algorimth='lyapunov')
    

Code examples
================================================================

This experiment from the paper 'Observer-Based Controllers for Two-Wheeled Inverted Robots with Unknown Input Disturbance and Model Uncertainty'
Declare LTI system:

.. code-block:: python

    import numpy as np 

    A = np.array([[0,          1,    0,     0],
                  [28.0286,    0,    0,     0],
                  [0,          0,    0,     1],
                  [-1.4014,    0,    0,     0]])

    B = np.array([[    0     ],
                    -1.5873  ],
                       0     ],
                     0.6349  ]])

    C = np.array([[1,    0,    0,     0],
                  [0,    1,    0,     0],
                  [0,    0,    1,     0],
                  [0,    0,    0,     1]])

    sys = classiccontrol.linearsystem.LTI(A=A, B=B, C=C)

Basic attribute of LTI system.

.. code-block:: python

    stability = sys.is_stable()
    controllability =sys.controllable()
    observability = sys.observable()
    poles = sys.eigvals()


simulate Open-loop system.

.. code-block:: python

    sys.setup_simulink(max_step=1e-3, algo='RK45', t_sim=(0,10), x0=None, sample_time = 1e-2,z0=None)
    time_array,state,output = sys.step_response()

Design controller

.. code-block:: python
    controller = classiccontrol.controller.PoleStatement(pole=[-3,-4,-5,-6], system=sys)
    R = controller.compute()

Design observer 

.. code-block:: python
    observer = classiccontrol.observer.Luenberger(pole=[-3,-4,-5,-6], system=sys)
    L = observer.compute()

simulate Closed-loop system with state-feedback controller R

.. code-block:: python
    sys.setup_simulink()
    time_array,state,output,state_obs = sys.apply_state_feedback(R)

simulate Closed-loop system with output-feedback L-R 

.. code-block:: python
    sys.setup_simulink()
    time_array,state,output,state_obs = sys.apply_output_feedback(L,R)



