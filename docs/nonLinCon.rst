*************************
Non-Linear ADP Controller
*************************

Problems statements
================================================================

Given the nonlinear system equation :eq:`nonLin`, design a control policy :math:`u` that minimize the following cost function:

.. math::
    V(x) = \int_0^\infty r(x(t),u(t))dt, \hspace{1cm} x(0)=x_0

where :math:`r(x,u)=q(x)+u^TR(x)u` with :math:`q(x)` a positive definite function, and :math:`R(x)` is symmetric and positive definite for all :math:`x \in \mathbb{R}^n`

