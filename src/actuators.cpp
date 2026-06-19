#include "actuators.h"
#include "transformations.h"
#include "constants.h"

State actuatorDerivatives(const State& state,const std::array<double,4>& RPMs,const MotorSet& motors){

    // Getting the acceleration from the required RPMs
    Vector3 thrust = calcThrust(RPMs,motors);
    Vector3 acc;
    for(int i=0;i<3;i++)
        acc[i]=thrust[i]/mass;

    // Transforming to global frame
    Vector3 globalAcc=bodyToGlobal(state.phi,state.theta,state.psi)*acc;

    // Getting the toruque from the required RPM
    Vector3 torque = calcTorque(RPMs,motors);

    // Getting the angular accelerations from the torque 
    double pdot=(torque[0]+(Iyy-Izz)*state.q*state.r)/Ixx;
    double qdot=(torque[1]+(Izz-Ixx)*state.p*state.r)/Iyy;
    double rdot=(torque[2]+(Ixx-Iyy)*state.q*state.p)/Izz;

    return {0.0,0.0,0.0,globalAcc[0],globalAcc[1],globalAcc[2],0.0,0.0,0.0,pdot,qdot,rdot};
}