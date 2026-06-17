import csv
import numpy as np
from include.state import State
from include.integrators import rk4_step
from include.target import run_target
from include import thruster
from include.cascade import Cascade
from src.dynamics import derivatives
from src.controller import contorller_output
from include import logger


# Initialising the state variable
state=State([0.0,0.0,0.0,5.0,5.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
t=0.0
t_final=1000.0
dt=0.01

# Initialising the variables for target object 
target_state=np.array([100.0,100.0,1.0,0.0,0.0,0.0])
prev_target_state=target_state.copy()

# Initialising the drone motors 
motors=thruster.initialise_drone(0.2,2.5e-7,5e-9,8000)

# Initialising the Cascade Controller
cascadeController=Cascade(np.array([
    [0.12,0.12,0.12],#0.04
    [2.0,2.0,2.0],#1.6
    [0.01,0.01,0.01],#0.01
    [0.02,0.02,0.02]#0.02
]),np.array([0.1,0.1,0.1]))#0.02

# Initialising the csv file to store data 
filename=input("Enter the filename to store the simulation data(without extension): ")
file=open(f"data/{filename}.csv","w",newline="")
writer=csv.writer(file)
logger.init_file(writer)

# Propagation loop
while t<t_final:
    
    # Updating the target object
    prev_target_state=target_state.copy()
    target_state=run_target(t,prev_target_state,dt).copy()

    # Getting the RPMs from the controller based on the relative target location
    RPMs=contorller_output(t,state,[target_state,prev_target_state,dt,motors,cascadeController])

    # Updating fawkes and appending the data
    state=rk4_step(state,t,dt,derivatives,[RPMs,motors])
    t+=dt
    logger.log_state(writer,t,state,target_state,RPMs)

    # If strikes ground or target then stop the simulation
    if state.z<0.0:
        print("Crashed")
        break
    elif np.sqrt((state.x-target_state[0])**2+(state.y-target_state[1])**2+(state.z-target_state[2])**2)<0.1:
        print("Target Intercepted")
        break

# Closing the file
file.close()
print("Simulation Ended")