import numpy as np
from include import constants as cons
from include.state import State
from include import thruster

K=1.0


# This is the actuator responsible for commanding the motors with the RPM
def actuator_derivatives(state,RPMs,motors):

    # Getting the acceleration from the required RPM
    thrust=thruster.calc_thrust(RPMs,motors)
    acc=thrust/cons.mass

    # Transforming to global frame
    global_acc=cons.body_to_global_matrix(state.phi,state.theta,state.psi)@acc

    # Getting the torque from the required RPM
    torque=thruster.calc_torque(RPMs,motors)

    # Getting angular accelerations from torque using euler's equations
    pdot=K*(torque[0]+(cons.Iyy-cons.Izz)*state.q*state.r)/cons.Ixx
    qdot=K*(torque[1]+(cons.Izz-cons.Ixx)*state.p*state.r)/cons.Iyy
    rdot=K*(torque[2]+(cons.Ixx-cons.Iyy)*state.p*state.q)/cons.Izz

    return State([0.0,0.0,0.0,global_acc[0],global_acc[1],global_acc[2],0.0,0.0,0.0,pdot,qdot,rdot])
