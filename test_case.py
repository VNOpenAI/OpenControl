import numpy as np
from OpenControl import classiccontrol
from OpenControl import visualize
from testcase_linearSystem import *
def main():
    system = classiccontrol.linearsystem.LTI(A=A0,B=B0)
    #print(f'is_stable = {system.is_stable()}')
    print(system.dimension)


if __name__ =='__main__':
    main()

