# Creating the file and initialsing the columns
def init_file(writer):

    writer.writerow(["Time","X","Y","Z","Vx","Vy","Vz",
                     "Phi","Theta","Psi","P","Q","R","X_Target","Y_Target","Z_Target","fRPM","rRPM","bRPM","lRPM"])

# Logging the state in the file
def log_state(writer,t,state,target_state,RPMs):
    
    writer.writerow([t,state.x,state.y,state.z,state.vx,state.vy,state.vz,
                    state.phi,state.theta,state.psi,state.p,state.q,state.r,
                    target_state[0],target_state[1],target_state[2],RPMs[0],RPMs[1],RPMs[2],RPMs[3]])
