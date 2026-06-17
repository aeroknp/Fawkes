import numpy as np
from include import constants as cons
from include.mixer import get_rpm
from src.gravity import gravity
from src.guidance import guidance

# This is the controller responsible to calculate the required RPMs at any point of time
def contorller_output(t,state,extra_paras=None):

    extra=extra_paras[0:3]
    dt=extra_paras[2]
    motors=extra_paras[3]
    Cascade=extra_paras[4]

    # Getting the net acceleration required in the global frame
    req_state_deri=guidance(t,state,extra)-gravity(t,state)
    anet=np.array([req_state_deri.vx,req_state_deri.vy,req_state_deri.vz])

    if t<0.01:
        print(f"t={t} anet=[{anet[0]}, {anet[1]}, {anet[2]}]")


    # Converting to body frame
    Anet=cons.global_to_body_matrix(state.phi,state.theta,state.psi)@anet

    # Calculate magnitude of the required acceleration
    anet_mag = np.linalg.norm(Anet)

    if anet_mag > 1e-6: # Prevent divide by zero
        anet_dir = Anet / anet_mag
    else:
        anet_dir = np.array([0.0, 0.0, 0.0])

    # Getting the error vector
    error_vec = np.array([-Anet[1], Anet[0], 0.0])

    # Getting required torque based on the error vector
    Torque=Cascade.run_cascade(error_vec,state,dt)

    # Getting the required RPMs based on the required torque and thrust
    Thrust=anet_mag*cons.mass
    RPMs=get_rpm(Thrust,Torque,motors)

    return RPMs
