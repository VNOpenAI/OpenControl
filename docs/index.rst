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

.. rubric:: Installation
   
Before install OpenControl, ensure that all the dependent packages in the `requirements`_ is instelled. Then

   pip install OpenControl
   
.. _`requirements`: https://github.com/VNOpenAI/OpenControl/blob/master/requirements.txt

Your contributions are welcome!  Simply fork the `GitHub repository <https://github.com/VNOpenAI/OpenControl>`_ and send a
`pull request`_.

.. _`pull request`: https://github.com/VNOpenAI/OpenControl/pulls

Please see the `Developer's Wiki`_ for detailed instructions.

.. _`Developer's Wiki`: https://github.com/VNOpenAI/OpenControl/wiki

.. rubric:: Links

- Issue tracker: https://github.com/VNOpenAI/OpenControl/issues


Quick-start
============================

For quick tutorials and application, please review the `Colab Notebook`_

.. _`Colab Notebook`: https://colab.research.google.com/drive/10mYMDliuOZD5i-YqmD9noOL8JDhC6t3x?usp=sharing

Indices and tables
==================


* :ref:`modindex`
* :ref:`search`
