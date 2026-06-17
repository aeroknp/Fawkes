import numpy as np
import matplotlib.pyplot as plt
from include.integrators import rk4_step

# This function is used to define the motion of the target for simulation purposes
def target_derivatives(t,state,extra_paras=None):
    x,y,z,vx,vy,vz=state

    omega=0.2

    dxdt=vx
    dydt=vy
    dzdt=vz

    dvxdt=-(omega**2)*x
    dvydt=-(omega**2)*y
    dvzdt=1.0

    # dvxdt=0.0
    # dvydt=0.0
    # dvzdt=0.0

    return np.array([dxdt,dydt,dzdt,dvxdt,dvydt,dvzdt])

# Updates the target state
def run_target(t,prev_target_state,dt,extra_paras=None):
    target_state = rk4_step(prev_target_state,t,dt,target_derivatives)

    return target_state 





