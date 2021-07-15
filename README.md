# Python Library for Control System

![](docs/_static/robotics.svg)

The OpenControl is a python package that implement basic algorithms for analysis and design of optimal feedback controllers.

### Features:

- Classical control methods
- Linear quadratic regulator (``LQR``) computation
- Adaptive Dynamic Programming (``ADP``) for optimal linear/nonlinear control systems
- Off-policy, On-policy learning algorithms for linear/nonlinear systems
- Experience replay algorithms for linear/nonlinear systems (TODO)

# Learn Reinforcement Learning by the tutorial [notebook](https://colab.research.google.com/drive/10mYMDliuOZD5i-YqmD9noOL8JDhC6t3x#scrollTo=E7MYyIKnQDjr) and [examples](https://github.com/VNOpenAI/OpenControl/tree/master/examples) code.

OpenControl provide algorithms for wide range of systems dynamics, include classical control module and reinforcement learning control systems using the ADP module. For learning purpose, we prefer users to review our examples code that will instruct to [define a system](https://opencontrol.readthedocs.io/en/latest/system.html) and [run simulation](https://opencontrol.readthedocs.io/en/latest/linCon.html).

## Development

You can check out the latest version of the source code with the command

   `git clone https://github.com/VNOpenAI/OpenControl`

## Installation

Before all, create a new virtual conda environment and install requirements 

    pip install requirements.txt

You can do minimal installation of OpenControl with

    pip install OpenControl

Check version

    python
    >>> import OpenControl 
    >>> OpenControl.__version__


## Documentation 

Please check the lastest version of [document](https://opencontrol.readthedocs.io/en/latest/intro.html)

## How to contact us

For any question, drop an email at phi9b2@gmail.com

Follow us on website https://vnopenai.org/