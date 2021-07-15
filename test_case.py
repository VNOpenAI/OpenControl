import numpy as np 
from OpenControl import classiccontrol
import matplotlib.pyplot as plt
import random
def main():
    state_shape = random.randint(2,10)
    intput_shape = random.randint(1,state_shape)
    output_shape = random.randint(1,state_shape)
    A = np.random.randint(0,100,(state_shape,state_shape))
    B = np.random.randint(0,100,(state_shape,intput_shape))
    C = np.random.randint(0,100,(output_shape,state_shape))
    sys = classiccontrol.linearsystem.LTI(A=A,B=B,C=C)
    pole = [random.randint(-10,-1) for _ in range(state_shape)]
    #controllability = sys.is_controlable()
    #if controllability==False:
    #    continue
    ober = sys.is_observable()

    ctl = classiccontrol.controller.PoleStatement(pole=pole,system=sys)
    obs = classiccontrol.observer.Luenberger(pole=pole,system=sys)
    R = ctl.compute()
    print('-------------------')
    # print(A)
    # print(B)
    pole.sort()
    ld,v = np.linalg.eig(A-R@C)
    ld.sort()
    print(pole,'\n',ld)
    sys.setup_simulink()
    time_array,x,y = sys.apply_state_feedback(R)
    for i in range(state_shape):
        

if __name__ =='__main__':
    main()