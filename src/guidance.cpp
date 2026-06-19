#include <cmath>
#include <stdexcept>
#include "guidance.h"
#include "Vector3.h"

// Constant
double N=10.0;
double liftOffAcc=2.0;
double purePursuitAcc=2.0;

// Check the current mission phase and call the respective guidance function
State guidance(const std::string& missionPhase,double t,const State& state,const targetParas& target){

    if(missionPhase=="propNav")
        return propNavGuidance(t,state,target);
    else if (missionPhase=="rise")
        return liftOffGuidance(t,state,target);
    else if (missionPhase=="purePursuit")
        return purePursuitGuidance(t,state,target);
    else
        throw std::runtime_error("Incorrect guidance phase");
}

// Guidance for lift off phase
State liftOffGuidance(double t,const State& state,const targetParas& target){
    return {0.0,0.0,0.0,0.0,0.0,liftOffAcc,0.0,0.0,0.0,0.0,0.0,0.0};
}

// Guidance for pure pursuit phase
State purePursuitGuidance(double t,const State& state,const targetParas& target){

    Vector3 rT=target.targetState;
    Vector3 rM(state.x,state.y,state.z);
    
    Vector3 rTMhat=(rT - rM).normalise();

    double dvxdt=purePursuitAcc*rTMhat[0];
    double dvydt=purePursuitAcc*rTMhat[1];
    double dvzdt=purePursuitAcc*rTMhat[2];

    return {0.0,0.0,0.0,dvxdt,dvydt,dvzdt,0.0,0.0,0.0,0.0,0.0,0.0};
}

// Guidance for proportional navigation phase
State propNavGuidance(double t,const State& state, const targetParas& target){

    // Calculating the position and velocity of target and missile
    Vector3 rT = target.targetState;
    Vector3 vT = (rT-target.prevTargetState)/target.dt;
    Vector3 rM(state.x,state.y,state.z);
    Vector3 vM(state.vx,state.vy,state.vz);

    // Position and velocity of target wrt satellite
    Vector3 rTM = rT - rM;
    Vector3 vTM = vT - vM;

    // Representing lambda xy, xz and yz in this order
    Vector3 LOS(atan2(rTM[1],rTM[0]),atan2(rTM[2],rTM[0]),atan2(rTM[2],rTM[1]));

    // Getting lamda dot xy, xz and yz in this order
    Vector3 LOSDot(
        ((rTM[0]*vTM[1]-rTM[1]*vTM[0])/std::max(rTM[0]*rTM[0]+rTM[1]*rTM[1],denMin)),
        ((rTM[0]*vTM[2]-rTM[2]*vTM[0])/std::max(rTM[0]*rTM[0]+rTM[2]*rTM[2],denMin)),
        ((rTM[1]*vTM[2]-rTM[2]*vTM[1])/std::max(rTM[1]*rTM[1]+rTM[2]*rTM[2],denMin))
    );

    // Getting the closing velocity in xy, xz and yz planes
    Vector3 vC(
        (-(rTM[0]*vTM[0]+rTM[1]*vTM[1])/std::max(std::sqrt(rTM[0]*rTM[0]+rTM[1]*rTM[1]),denMin)),
        (-(rTM[0]*vTM[0]+rTM[2]*vTM[2])/std::max(std::sqrt(rTM[0]*rTM[0]+rTM[2]*rTM[2]),denMin)),
        (-(rTM[1]*vTM[1]+rTM[2]*vTM[2])/std::max(std::sqrt(rTM[1]*rTM[1]+rTM[2]*rTM[2]),denMin))
    );

    // Getting the command accelerations for xy, xz and yz planes
    Vector3 nC(
        N*vC[0]*LOSDot[0],
        N*vC[1]*LOSDot[1],
        N*vC[2]*LOSDot[2]
    );

    double dvxdt = - nC[0]*sin(LOS[0]) - nC[1]*sin(LOS[1]);
    double dvydt =  nC[0]*cos(LOS[0]) - nC[2]*sin(LOS[2]);
    double dvzdt =  nC[1]*cos(LOS[1]) + nC[2]*cos(LOS[2]);

    return {0.0,0.0,0.0,dvxdt,dvydt,dvzdt,0.0,0.0,0.0,0.0,0.0,0.0};
}