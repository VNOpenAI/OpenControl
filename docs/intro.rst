************
Introduction
************

OpenControl is an open-source library that support rapid development of algorithms in modern control theory. This source code is written by python for robotic and control system simulation purposes. We welcome contributions from the open-source community.

Over view of the toolbox
================================================================
The package contains a list of algorithms that cover the topic of classical feedback and adaptive optimal control for continuous system. The initial goal is to provide an analysis of the feedback control systems throung simulations. In particular, we focus on the **Adaptive/Approximate Dynamic Programming**, a powerful methodology that integrates the idea of **Reinforcement Learning** (RL_)  and with **Control Theory** in a model-free, data-driven approaches.

.. _RL: https://en.wikipedia.org/wiki/Reinforcement_learning#:~:text=Reinforcement%20learning%20(RL)%20is%20an,supervised%20learning%20and%20unsupervised%20learning. 

With OpenControl you can design the control system through simulating:
------------------------------------------------------------------------------------------------
    - Classic feedback control design
    - Policy Iteration (PI) algorithms for both linear (LTI) and non-linear systems.
    - Compare with LQR (in case of LTI) results.
    - Analysis On-policy Learning versus Off-policy Learning.
    - Customize the value function and time-interval for learning
    - Customize the basis function of neural network used for approximating actor/critic
    - Reconstruct trending papers' result, explore new control algorithms
    - Visualize the simulation results by Tensorboard_

.. _Tensorboard: https://www.tensorflow.org/tensorboard

Installation
================================================================

The `OpenControl` package can be installed using pip, support Linux, MacOS and Window 10 (64-bit) with python 3.8 ::

    pip install OpenControl

Getting started
================================================================

::

    >>> import OpenControl


