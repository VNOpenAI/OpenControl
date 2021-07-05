.. Approximate Dynamic Programming documentation master file, created by
   sphinx-quickstart on Mon Jun 28 11:02:46 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Python Approximate Dynamic Programming for Control System
===========================================================

The Python Approximate Dynamic Programming for Control System (`python-adp`) is the python package 
that implement basic algorithms for analysis and design of optimal feedback controllers.

.. rubric:: Features:

- Linear quadratic regulator (LQR) computation
- Integral reinforcement learning algorithms for linear/nonlinear systems
- Off-policy learning algorithms for linear/nonlinear systems
- On-policy learning (Q-Learning) algorithms for linear/nonlinear systems
- Experience replay algorithms for linear/nonlinear systems
  
.. rubric:: Documentation
.. toctree::
   :maxdepth: 2
   
   intro
   system
   linCon
   nonLinCon
   visualize
   examples

* :ref:`genindex`

.. rubric:: Development

The :class:`ADP_control.system.LTI` :func:`ADP_control.system.LTI.integrate`

You can check out the latest version of the source code with the command::

   git clone https://github.com/PhiDCH/ADP

You can run the unit tests with `python`_ to make sure that everything is
working correctly.  Inside the source directory, run::

   python linearSystem.py

.. _python: https://www.python.org

Your contributions are welcome!  Simply fork the `GitHub repository <https://github.com/PhiDCH/ADP>`_ and send a
`pull request`_.

.. _pull request: https://github.com/PhiDCH/ADP/pulls

Please see the `Developer's Wiki`_ for detailed instructions.

.. _Developer's Wiki: https://github.com/PhiDCH/ADP/wiki

.. rubric:: Links

- Issue tracker: https://github.com/PhiDCH/ADP/issues


Indices and tables
==================


* :ref:`modindex`
* :ref:`search`
