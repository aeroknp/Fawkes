import numpy as np

# This is the motor class which will create the motors
class Motor:

    # Initialising the motors
    def __init__(self,motor_specs=None):

        if motor_specs is None:
            motor_specs=[0.0]*4

        self.position=np.array(motor_specs[0:3])    # Creates the position array of individual motor
        self.kf=motor_specs[3]                      # Getting the kf value
        self.km=motor_specs[4]                      # Getting the km value
        self.spin=motor_specs[5]                    # +1 for CW and -1 for ACW
        self.max_RPM=motor_specs[6]                 # Setting the max RPM limits

    def create_motor(self,motor_specs):
        self.position=np.array(motor_specs[0:3])
        self.kf=motor_specs[3]
        self.km=motor_specs[4]
        self.spin=motor_specs[5]
        self.max_RPM=motor_specs[6]

    # Running the motor on the RPM
    def run_motor(self,RPM):
        return np.array([0.0,0.0,self.kf*(RPM**2)])

# Initialising the drone by creating all four motors
def initialise_drone(arm_length,kf,km,max_rpm):

    front_motor=Motor([0.0,arm_length,0.0,kf,km,1.0,max_rpm])
    right_motor=Motor([arm_length,0.0,0.0,kf,km,-1.0,max_rpm])
    back_motor=Motor([0.0,-arm_length,0.0,kf,km,1.0,max_rpm])
    left_motor=Motor([-arm_length,0.0,0.0,kf,km,-1.0,max_rpm])

    return [front_motor,right_motor,back_motor,left_motor]

# Calculating the net thrust by providing the RPM and motor data
def calc_thrust(RPMs,motors):
    fRPM,rRPM,bRPM,lRPM=RPMs
    fmotor,rmotor,bmotor,lmotor=motors

    fT=fmotor.run_motor(fRPM)
    rT=rmotor.run_motor(rRPM)
    bT=bmotor.run_motor(bRPM)
    lT=lmotor.run_motor(lRPM)

    return fT+rT+bT+lT

# Calculating the net required torque by providing the RPM and motor data
def calc_torque(RPMs,motors):

    fRPM,rRPM,bRPM,lRPM=RPMs
    fmotor,rmotor,bmotor,lmotor=motors

    fT=fmotor.run_motor(fRPM)
    rT=rmotor.run_motor(rRPM)
    bT=bmotor.run_motor(bRPM)
    lT=lmotor.run_motor(lRPM)

    fTou=np.cross(fmotor.position,fT)   
    rTou=np.cross(rmotor.position,rT)   
    bTou=np.cross(bmotor.position,bT)   
    lTou=np.cross(lmotor.position,lT)

    resfTou=np.array([0.0,0.0,(fmotor.spin*fmotor.km*(fRPM**2))])   
    resrTou=np.array([0.0,0.0,(rmotor.spin*rmotor.km*(rRPM**2))])   
    resbTou=np.array([0.0,0.0,(bmotor.spin*bmotor.km*(bRPM**2))])   
    reslTou=np.array([0.0,0.0,(lmotor.spin*lmotor.km*(lRPM**2))])   

    return fTou+rTou+bTou+lTou+resfTou+resrTou+resbTou+reslTou



