from binascii import Error
import numpy as np 
from OpenControl import classiccontrol
import matplotlib.pyplot as plt
import random


def random_system():
    state_shape = random.randint(2,10)
    intput_shape = random.randint(1,state_shape)
    output_shape = random.randint(1,state_shape)
    A = np.random.randint(0,100,(state_shape,state_shape))
    B = np.random.randint(0,100,(state_shape,intput_shape))
    C = np.random.randint(0,100,(output_shape,state_shape))
    sys = classiccontrol.linearsystem.LTI(A=A,B=B,C=C)
    pole = [random.randint(-100,-1) for _ in range(state_shape)]
    pole.sort()
    return sys,pole

def test_controller_state_feedback():
    for i in range(100):
        sys,pole = random_system()
        controllability = sys.is_controlable()
        if controllability==False:
            continue
        ctl = classiccontrol.controller.PoleStatement(pole=pole,system=sys,)
        try:
            R = ctl.compute()
        except:
            continue
        pole = np.array(pole)
        ld,_ = np.linalg.eig(sys.A-sys.B@R)
        ld = ld.real
        ld.sort()
        ld = ld.reshape(-1)
        if (ld - pole).sum()>0.1:
            raise Error("Incorrect controller")

def test_observer_output_feedback():
    for i in range(100):
        sys,pole = random_system()
        observability = sys.is_observable()
        if observability==False:
            continue
        ob = classiccontrol.observer.Luenberger(pole=pole,system=sys)
        try:
            L = ob.compute()
        except:
            continue

        pole = np.array(pole)
        ld,_ = np.linalg.eig(sys.A-L@sys.C)
        ld.sort()
        ld = ld.reshape(-1)
        if (ld - pole).sum()>0.1:
            raise Error("Incorrect observer")
        
if __name__ =='__main__':
    test_controller_state_feedback()
    test_observer_output_feedback()
    print('no error')