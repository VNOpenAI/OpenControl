.. Approximate Dynamic Programming documentation master file, created by
   sphinx-quickstart on Mon Jun 28 11:02:46 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Python Library for Control System
===========================================================

The OpenControl is a python package that implement basic algorithms for analysis and design of optimal feedback controllers.

.. rubric:: Features:

- Classical control methods
- Linear quadratic regulator (``LQR``) computation
- Robust Adaptive Dynamic Programming (``ADP``) for optimal linear/nonlinear control systems
- Off-policy, On-policy learning algorithms for linear/nonlinear systems
- Experience replay algorithms for linear/nonlinear systems (TODO)
  
.. rubric:: Documentation
.. toctree::
   :maxdepth: 2
   
   intro
   classic
   system
   linCon
   nonLinCon
   visualize
   examples

* :ref:`genindex`

.. rubric:: Development

You can check out the latest version of the source code with the command::

   git clone https://github.com/VNOpenAI/OpenControl

You can run the unit tests with `python`_ to make sure that everything is
working correctly.  Inside the source directory, run::

   python test.py

.. _python: https://www.python.org

Your contributions are welcome!  Simply fork the `GitHub repository <https://github.com/VNOpenAI/OpenControl>`_ and send a
`pull request`_.

.. _pull request: https://github.com/VNOpenAI/OpenControl/pulls

Please see the `Developer's Wiki`_ for detailed instructions.

.. _`Developer's Wiki`: https://github.com/VNOpenAI/OpenControl/wiki

.. rubric:: Links

- Issue tracker: https://github.com/VNOpenAI/OpenControl/issues


Indices and tables
==================


* :ref:`modindex`
* :ref:`search`
