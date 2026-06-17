import numpy as np
from include import constants as cons
from include.state import State

# Getting the drag on the body
def drag(t,state):
    vx=state.vx
    vy=state.vy
    vz=state.vz

    v=np.sqrt(vx**2+vy**2+vz**2)

    fd=0.5*(cons.rho_surface)*(v**2)*(cons.Cd)*(cons.Area)

    if v!=0:
        dvxdt=-(fd/cons.mass)*(vx/v)
        dvydt=-(fd/cons.mass)*(vy/v)
        dvzdt=-(fd/cons.mass)*(vz/v)
    else:
        dvxdt=0.0   
        dvydt=0.0   
        dvzdt=0.0

    return State([0.0,0.0,0.0,dvxdt,dvydt,dvzdt,0.0,0.0,0.0,0.0,0.0,0.0])    
