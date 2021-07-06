*************************
System representation
*************************       

LTI system
================================================================

Consider a continuous linear time invariant system described by:
    
.. math:: 
    \dot{x} = Ax + Bu \\
    y = Cx + Du
    :label: LTI

where :math:`x\in\mathbb{R}^n` is the state, :math:`u\in\mathbb{R}^m` is the control input, 
:math:`A\in\mathbb{R}^{nn}` is the state matrix, :math:`B\in\mathbb{R}^{nm}` is the control matrix 

To create a LTI system use :class:`OpenControl.ADP_control.LTI` 

.. code-block:: python

    import numpy as np
    from ADP_control import LTI

    A = np.eye(3); B = np.ones((3,1))
    sys = LTI(A, B) # or sys = LTI(A, B, C, D) 

then _`setup a simulation section`, use :func:`OpenControl.ADP_control.LTI.setSimulationParam`

.. code-block:: python

    t_start = 0; t_stop = 10
    x0 = np.array([1,2,3])
    sample_time = 0.01

    sys.setupSimulationParam(t_sim=(t_start, t_stop), x0=x0, sample_time=sample_time)

Non-linear System
===============================================================

Consider a continuous-time affine nonlinear dynamical system described by: 

.. math:: \dot{x} = f(x) + g(x)u
    :label: nonLin

where :math:`x\in\mathbb{R}^n` is the state, :math:`u\in\mathbb{R}^m` is the control input, :math:`f:\mathbb{R}^n \to \mathbb{R}^n` and :math:`g:\mathbb{R}^n \to \mathbb{R}^{nm}` are locally Lipschitz mappings with :math:`f(0)=0`

To create a nonlinear system use :class:`OpenControl.ADP_control.NonLin` 

.. code-block:: python

    from ADP_control import nonLin

    dot_x = lambda t,x,u: 3x + np.array([0 0 1]).dot(u)
    dimension = (3,1)
    sys = nonLin(dot_x, dimension)

then setup a simulation section, see `setup a simulation section`_



.. autoclass:: OpenControl.ADP_control.LTI
    :members:
    :undoc-members:
    :show-inheritance:
    :special-members: __init__

.. autoclass:: OpenControl.ADP_control.NonLin
    :members:
    :undoc-members:
    :show-inheritance:
    :special-members: __init__



 