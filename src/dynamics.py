from src.kinematics import kinematics
from src.gravity import gravity
from src.environment import drag
from src.actuators import actuator_derivatives

# Getting the total derivatives function
def derivatives(t,state,extra_paras=None):
    
    total = kinematics(t,state)+gravity(t,state)+actuator_derivatives(state,extra_paras[0],extra_paras[1])

    return total