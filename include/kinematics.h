// This file contains the kinematics function used in the simulation
#pragma once
#include "state.h"
#include "Vector3.h"
#include "transformations.h"

inline State kinematics(double t, const State& state){

    double dxdt=state.vx;
    double dydt=state.vy;
    double dzdt=state.vz;

    // Getting the rotational derivatives
    Vector3 rollRate = rollRateBodyToEuler(state.phi,state.theta,state.psi)*Vector3(state.p,state.q,state.r);

    return {dxdt,dydt,dzdt,0.0,0.0,0.0,rollRate[0],rollRate[1],rollRate[2],0.0,0.0,0.0};
}